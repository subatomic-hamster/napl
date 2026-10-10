#!/usr/bin/env python3
"""G0b spike (docs/gpu-physics-g0.md): MJX on JAX-ROCm vs CPU MuJoCo on the bare napl MJCF of x500v2 / asv. Physics only, no devices.

Run in the throwaway venv (jax ROCm + mujoco + mujoco-mjx), never in the project venv:
  export XLA_FLAGS="--xla_gpu_enable_command_buffer=" XLA_PYTHON_CLIENT_MEM_FRACTION=0.75
  python tools/mjx_bench.py prep   --xml <napl built x500v2.xml> --out DIR      # stripped MJCF + 4 thrust motors
  python tools/mjx_bench.py cpu    --model DIR/x500v2.xml                       # mujoco.rollout, 1 and 16 threads
  python tools/mjx_bench.py mjx    --model DIR/x500v2.xml --n 1 256 4096 16384  # one subprocess per (N, dtype)
  python tools/mjx_bench.py acc    --model DIR/x500v2.xml                       # 5 s trajectory: MJX f32/f64 vs MuJoCo f64
  python tools/mjx_bench.py det    --model DIR/x500v2.xml                       # 3 repeats bitwise, with/without deterministic flag
  python tools/mjx_bench.py sweep  --model DIR/x500v2.xml                       # (napl venv) C++ Sweep, 1 and 16 threads
Every subcommand prints JSON lines. Single-N children are `_mjx` / `_acc` (internal)."""

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

G = 9.81
STEPS = 1000
NOISE = (
    1e-3  # per-env qvel noise of the throughput runs (--noise 0 for the accuracy runs)
)


def prep(a):
    """napl XML -> bare dynamics: drop visual geoms (mass 0, no collision), meshes, sites, tendons (wires), custom devices."""
    root = ET.parse(a.xml).getroot()
    for parent in list(root.iter()):
        for ch in list(parent):
            if ch.tag in ("tendon", "custom", "sensor", "contact", "site"):
                parent.remove(ch)
            elif ch.tag == "geom":
                vis = ch.get("contype") == "0" and ch.get("conaffinity") == "0"
                if float(ch.get("mass", "0")) == 0 and (
                    vis or ch.get("type") == "mesh"
                ):
                    parent.remove(ch)
    used = {g.get("mesh") for g in root.iter("geom") if g.get("mesh")}
    asset = root.find("asset")
    if asset is not None:
        for m in list(asset):
            if m.tag in ("mesh", "material", "texture") and not (
                m.tag == "mesh" and m.get("name") in used
            ):
                asset.remove(m)
        if not len(asset):
            root.remove(asset)
    nprop = 0
    act = ET.SubElement(root, "actuator")
    for b in root.iter("body"):
        if b.find("joint") is not None and b.get("name", "").startswith("prop"):
            nprop += 1
            ET.SubElement(b, "site", name=f"thr{nprop}", pos="0 0 0")
            ET.SubElement(
                act,
                "motor",
                name=f"thr{nprop}",
                site=f"thr{nprop}",
                gear="0 0 1 0 0 0",
                ctrlrange="0 40",
            )
    if not nprop:
        root.remove(act)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dst = out / Path(a.xml).name
    ET.indent(root)
    ET.ElementTree(root).write(dst)
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(dst))
    print(
        json.dumps(
            dict(
                model=str(dst),
                nbody=m.nbody,
                ngeom=m.ngeom,
                nv=m.nv,
                nu=m.nu,
                mass=float(m.body_subtreemass[1]),
                dt=m.opt.timestep,
            )
        )
    )


def _load(path, z=20.0):

    import mujoco

    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    if m.nq >= 7 and m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE:
        d.qpos[2] = z  # high enough that nothing touches the floor in 5 s
        d.qvel[:3] = [0.5, 0.2, 0.0]
        d.qvel[5] = 1.0
    mujoco.mj_forward(m, d)
    return m, d


def _ctrl(m, asym):
    """Constant per-motor thrust: hover share times (1 + asym * pattern), so the vehicle flies a smooth non-trivial arc."""
    import numpy as np

    if m.nu == 0:
        return np.zeros(0)
    pat = np.array([1.0, -1.0, 0.7, -0.7])[: m.nu]
    return (
        float(m.body_subtreemass[1]) * G / m.nu * (1.05 + asym * pat)
    )  # 5 % over hover: climbs and tilts slowly, never reaches the floor in 5 s


def cpu(a):
    import numpy as np

    import mujoco
    from mujoco import rollout

    m, d = _load(a.model)
    ns = mujoco.mj_stateSize(m, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    mujoco.mj_forward(m, d)
    s0 = np.zeros(ns)
    mujoco.mj_getState(m, d, s0, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    c = _ctrl(m, a.asym)
    for nt in a.threads:
        for n in a.n:
            nb = n
            ds = [mujoco.MjData(m) for _ in range(nt)]
            init = np.tile(s0, (nb, 1))
            ctrl = np.tile(c, (nb, STEPS, 1)) if m.nu else None
            with rollout.Rollout(nthread=nt) as r:
                r.rollout(m, ds, init, ctrl, nstep=STEPS)  # warm the pool
                best = 1e9
                for _ in range(a.reps):
                    t = time.perf_counter()
                    r.rollout(m, ds, init, ctrl, nstep=STEPS)
                    best = min(best, time.perf_counter() - t)
            print(
                json.dumps(
                    dict(
                        what="mujoco.rollout",
                        model=Path(a.model).stem,
                        threads=nt,
                        n=nb,
                        steps=STEPS,
                        wall_s=best,
                        env_steps_per_s=nb * STEPS / best,
                        mujoco=mujoco.__version__,
                    )
                ),
                flush=True,
            )


def _traj_cpu(m, d, ctrl, nsteps, every):
    import numpy as np

    import mujoco

    d.ctrl[:] = ctrl
    out = []
    for k in range(nsteps):
        mujoco.mj_step(m, d)
        if (k + 1) % every == 0:
            out.append(d.qpos[:3].copy())
    return np.array(out)


def _mjx_setup(path, n, asym):
    import jax
    import jax.numpy as jnp

    from mujoco import mjx

    m, d = _load(path)
    mx, d0 = mjx.put_model(m), mjx.put_data(m, d)
    rng = jax.random.split(jax.random.key(0), n)
    c = _ctrl(m, asym)

    def init(k):  # tiny per-env velocity noise so envs are not bit-identical
        return d0.replace(
            qvel=d0.qvel + NOISE * jax.random.normal(k, d0.qvel.shape, d0.qvel.dtype),
            ctrl=jnp.asarray(c, d0.ctrl.dtype),
        )

    vstep = jax.vmap(mjx.step, in_axes=(None, 0))

    def scan(db, nsteps=STEPS):
        def body(x, _):
            # with x64 the contact index arrays come back int64 from step but int32 from put_data: cast to the carry dtype
            x = jax.tree.map(lambda n, o: n.astype(o.dtype), vstep(mx, x), x)
            return x, x.qpos[:, :3]

        return jax.lax.scan(body, db, None, length=nsteps)

    return m, jax.vmap(init)(rng), scan


def _mem():
    import jax

    s = jax.devices()[0].memory_stats() or {}
    return s.get("peak_bytes_in_use", 0)


def _mjx(a):
    import jax
    import jax.numpy as jnp

    m, db, scan = _mjx_setup(a.model, a.n[0], a.asym)
    f = jax.jit(lambda x: scan(x)[0].qpos)
    t = time.perf_counter()
    comp = f.lower(db).compile()
    t_compile = time.perf_counter() - t
    comp(db).block_until_ready()  # first execution (autotune, allocation)
    best = 1e9
    for _ in range(a.reps):
        t = time.perf_counter()
        q = comp(db).block_until_ready()
        best = min(best, time.perf_counter() - t)
    n = a.n[0]
    print(json.dumps(dict(
        what="mjx", model=Path(a.model).stem, n=n, steps=STEPS, dtype=str(q.dtype), compile_s=t_compile, wall_s=best, env_steps_per_s=n * STEPS / best,
        peak_vram_gb=_mem() / 2**30, finite=bool(jnp.isfinite(q).all()), jax=jax.__version__, device=jax.devices()[0].device_kind,
        xla_flags=os.environ.get("XLA_FLAGS", ""), x64=bool(jax.config.jax_enable_x64))), flush=True)  # fmt: skip


def _child(sub, a, x64, extra_env=None, args=()):
    env = dict(os.environ, JAX_ENABLE_X64="1" if x64 else "0", **(extra_env or {}))
    cmd = [
        sys.executable,
        __file__,
        sub,
        "--model",
        a.model,
        "--asym",
        str(a.asym),
        *args,
    ]
    r = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=a.timeout)
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith("{")]
    if r.returncode or not lines:
        return dict(error=f"rc={r.returncode}", tail=(r.stderr or r.stdout)[-600:])
    return json.loads(lines[-1])


def mjx(a):
    for x64 in a.dtypes:
        for n in a.n:
            r = _child(
                "_mjx", a, x64 == "f64", args=["--n", str(n), "--reps", str(a.reps)]
            )
            r.update(req_n=n, req_dtype=x64)
            print(json.dumps(r), flush=True)
            if "error" in r:
                break  # larger N will not fit either


def _acc(a):
    """Child: trajectory of one env, every 10 steps, to an .npy (dtype from JAX_ENABLE_X64)."""
    import jax
    import numpy as np

    m, db, scan = _mjx_setup(a.model, 1, a.asym)
    _, tr = jax.jit(scan, static_argnums=1)(db, a.steps)
    np.save(a.save, np.asarray(tr[9::10, 0, :], dtype=np.float64))
    print(json.dumps(dict(saved=a.save, dtype=str(db.qpos.dtype))))


def acc(a):
    import numpy as np

    m, d = _load(a.model)
    nsteps = int(a.seconds / m.opt.timestep)
    p0 = d.qpos[:3].copy()
    ref = _traj_cpu(m, d, _ctrl(m, a.asym), nsteps, 10)
    got = {}
    for x64 in (False, True):
        f = Path(a.out) / f"traj_{'f64' if x64 else 'f32'}.npy"
        r = _child(
            "_acc",
            a,
            x64,
            args=["--steps", str(nsteps), "--save", str(f), "--noise", "0"],
        )
        if "error" in r:
            print(json.dumps(r))
            continue
        t = got[x64] = np.load(f)
        e = np.linalg.norm(t - ref, axis=1)
        trav = float(
            np.linalg.norm(np.diff(np.vstack([p0, ref]), axis=0), axis=1).sum()
        )
        dt10 = 10 * m.opt.timestep
        div = lambda thr: (
            float((np.argmax(e > thr) + 1) * dt10) if (e > thr).any() else None
        )
        print(json.dumps(dict(
            what="accuracy", model=Path(a.model).stem, dtype="f64" if x64 else "f32", seconds=a.seconds, asym=a.asym, travelled_m=trav,
            pos_rms_m=float(np.sqrt((e**2).mean())), pos_max_m=float(e.max()), final_err_m=float(e[-1]), final_err_pct=100 * float(e[-1]) / trav,
            max_err_pct=100 * float(e.max()) / trav, t_err_gt_1cm_s=div(0.01), t_err_gt_10cm_s=div(0.1),
            displacement_m=float(np.linalg.norm(ref[-1] - p0)))), flush=True)  # fmt: skip
    if len(got) == 2:  # precision alone: MJX f32 vs MJX f64
        e = np.linalg.norm(got[False] - got[True], axis=1)
        print(
            json.dumps(
                dict(
                    what="accuracy_f32_vs_mjx_f64",
                    asym=a.asym,
                    max_err_m=float(e.max()),
                    max_err_pct=100 * float(e.max()) / trav,
                )
            )
        )


def _det(a):
    import jax
    import numpy as np

    m, db, scan = _mjx_setup(a.model, a.n[0], a.asym)
    f = jax.jit(lambda x: scan(x)[0].qpos)
    h = [hashlib.sha256(np.asarray(f(db)).tobytes()).hexdigest()[:16] for _ in range(3)]
    print(json.dumps(dict(hashes=h)))


def det(a):
    for flag in (
        "",
        "--xla_gpu_exclude_nondeterministic_ops=true",
        "--xla_gpu_deterministic_ops=true",
    ):
        base = os.environ.get("XLA_FLAGS", "")
        env = dict(XLA_FLAGS=(base + " " + flag).strip())
        runs = [
            _child("_det", a, False, env, args=["--n", str(a.n[0])]) for _ in range(3)
        ]  # 3 processes: autotune/launch variation
        inproc = runs[0]
        t = time.perf_counter()
        r1 = _child("_mjx", a, False, env, args=["--n", str(a.n[0]), "--reps", "2"])
        hs = [h for r in runs for h in r.get("hashes", [])]
        print(json.dumps(dict(what="determinism", flag=flag or "(none)", in_process=inproc, hashes_across_processes=[r.get("hashes", [None])[0] for r in runs],
                              all_equal=len(set(hs)) == 1 and len(hs) == 9, env_steps_per_s=r1.get("env_steps_per_s"), error=r1.get("error"), tail=r1.get("tail"))), flush=True)  # fmt: skip


def sweep(a):
    """napl venv: the kernel's own C++ Sweep (bare MJCF, no devices) at 1 and 16 threads, runs x duration."""
    import napl

    sim = napl.Sim.load(a.model)
    root = ET.parse(
        a.model
    ).getroot()  # the napl venv has no mujoco python: read nu and dt from the XML
    nu = len(root.findall("./actuator/motor"))
    dt = float(root.find("option").get("timestep"))
    params = [
        {"path": f"/actuators/thr{i + 1}", "attr": "ctrl", "lo": 1.0, "hi": 5.0}
        for i in range(nu)
    ]  # cost does not depend on thrust
    for th in a.threads:
        runs = a.n[0]
        mon = dict(
            name="m",
            op="always",
            lo=-1e9,
            hi=1e9,
            signal={
                "path": "/world/" + root.find("./worldbody/body").get("name"),
                "attr": "pos",
            },
        )
        spec = dict(
            name="b", duration=STEPS * dt, settings=[], monitors=[mon]
        )  # a TestSpec needs one monitor
        t = time.perf_counter()
        rows = sim.sweep(spec, params, mode="random", n=runs, seed=1, threads=th)
        w = time.perf_counter() - t
        print(json.dumps(dict(what="napl.Sweep", model=Path(a.model).stem, threads=th, n=runs, steps=STEPS, wall_s=w, env_steps_per_s=runs * STEPS / w,
                              errors=sum(bool(r["error"]) for r in rows))), flush=True)  # fmt: skip


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "cmd",
        choices=["prep", "cpu", "mjx", "acc", "det", "sweep", "_mjx", "_acc", "_det"],
    )
    p.add_argument("--xml")
    p.add_argument("--out", default=".")
    p.add_argument("--model")
    p.add_argument("--n", type=int, nargs="+", default=[1, 256, 4096, 16384])
    p.add_argument("--threads", type=int, nargs="+", default=[1, 16])
    p.add_argument("--dtypes", nargs="+", default=["f32", "f64"])
    p.add_argument("--reps", type=int, default=3)
    p.add_argument("--asym", type=float, default=0.002)
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--steps", type=int, default=5000)
    p.add_argument("--save")
    p.add_argument("--noise", type=float, default=1e-3)
    p.add_argument("--timeout", type=int, default=1500)
    a = p.parse_args()
    globals()["NOISE"] = a.noise
    {
        "prep": prep,
        "cpu": cpu,
        "mjx": mjx,
        "acc": acc,
        "det": det,
        "sweep": sweep,
        "_mjx": _mjx,
        "_acc": _acc,
        "_det": _det,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
