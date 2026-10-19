"""Scenes of the dog's locomotion runs (SP4, HARNESS side): the nominal dog built once and cached, then per run a scene = the nominal MJCF
with the lidar and depth camera removed (sim budget, docs/models.md "Ground sim budget"), a course (flat floor / terrain field), the start
pose and an optional dispersion payload. course.json beside scene.xml records what the harness needs (the nominal model is what the
CONTROLLER's model is built from: it never sees the payload or the terrain)."""

import hashlib
import heapq
import json
import os
import shutil
import tempfile
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import dogsim as D
import numpy as np

REPO = Path(__file__).resolve().parents[2]
CACHE = Path(
    os.environ.get("NAPL_DOG_CACHE", Path(tempfile.gettempdir()) / "napl-dog-build")
)


ASM_ENV = "NAPL_DOG_ASM"  # hook (leg study): path of a variant dog.toml (tools/dog_sweep.py write_variant) built instead of assemblies/dog


def _asm():
    return Path(os.environ[ASM_ENV]) if os.environ.get(ASM_ENV) else D.ASM


def _key():
    h = hashlib.sha256()
    for f in sorted(
        [
            *_asm().parent.glob("*.toml"),
            *(REPO / "parts").rglob("*.toml"),
        ]
    ):
        h.update(f.read_bytes())
    return h.hexdigest()[:16]


def base_model():
    """Nominal dog MJCF (dogsim.build), cached by the content of assemblies/dog and parts; built in a temp dir and renamed into place, so
    concurrent MC workers never read a half-written build."""
    d = CACHE / _key()
    xml = d / "dog.xml"
    if not xml.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(dir=CACHE))
        x, rep = D.build(tmp, _asm())
        (tmp / "dog.report.json").write_text(json.dumps(rep))
        tree = ET.parse(x)
        for e in tree.getroot().iter():  # the builder writes mesh paths relative to its own xml: absolute, so scenes anywhere can use them
            if e.get("file"):
                f = (Path(x).parent / e.get("file")).resolve()
                if f.is_relative_to(
                    tmp.resolve()
                ):  # the builder's own visuals/ dir moves with the rename below
                    f = CACHE.resolve() / d.name / f.relative_to(tmp.resolve())
                e.set("file", str(f))
        tree.write(tmp / "dog.xml")
        if Path(x) != tmp / "dog.xml":
            Path(x).unlink()
        try:
            tmp.rename(d)
        except OSError:  # another worker won the race: same content
            shutil.rmtree(tmp, ignore_errors=True)
    return xml


def strip_lidar(root):
    t = root.find("custom/text[@name='napl.devices']")
    d = json.loads(t.get("data"))
    d["rangefinders"] = []
    t.set("data", json.dumps(d))


FIELD_SIZE = 30.0  # m: 3 x 3 patches of 10 m (napl.terrain.field), the 100 m statistics on a ~9x smaller load
COURSES = ("flat", "criterion", "slip", "rubble", "slopes", "scree", "ruins")
FAMILIES = ("rubble", "slopes", "scree", "ruins")
FOOTPRINT = (
    0.7,
    0.5,
)  # m: the dog's footprint at the start pose (the root is lifted above the highest ground under it)
START_CLEAR = 0.01  # m


RAMP_DEG = 25.0  # spec s7; the capability sweep in docs/dog/locomotion.md overrides it


def criterion_course():
    """Spec s7 '25-degree slopes and 15 cm steps traversed', along +x at y in [-15, 15]: flat to x = -9, a 25 deg ramp up 3 m (1.40 m), 2 m
    plateau, two 15 cm steps up and two down 2 m apart (one-cell risers, as napl.terrain's own steps), 2 m, a 25 deg ramp down, flat. Rock."""
    from napl.terrain import Field, Patch

    res, n = 0.05, int(FIELD_SIZE / 0.05) + 1
    x = -FIELD_SIZE / 2 + res * np.arange(n)
    t25 = np.tan(np.radians(RAMP_DEG))
    top = 3.0 * t25
    z = np.select(
        [x < -9, x < -6, x < -4, x < -2, x < 0, x < 2, x < 4, x < 6, x < 9],
        [0 * x, (x + 9) * t25, top + 0 * x, top + 0.15 + 0 * x, top + 0.30 + 0 * x, top + 0.15 + 0 * x, top + 0 * x, top + 0 * x,
         top - (x - 6) * t25],
        0 * x,
    )  # fmt: skip
    h = np.tile(z, (n, 1))
    pc = int(10.0 / res)
    patches = [Patch(-15 + a * 10, -15 + b * 10, -5 + a * 10, -5 + b * 10, "rock", a * pc, (a + 1) * pc, b * pc, (b + 1) * pc)
               for b in range(3) for a in range(3)]  # fmt: skip
    f = Field(
        "slopes",
        FIELD_SIZE,
        res,
        0,
        h,
        patches,
        [],
        None,
        [],
        10.0,
        {"criterion": True},
    )
    f.grid = np.full((3, 3), "rock", dtype=object)
    return f


def slip_course():
    """Flat ground whose middle patch column (x in [-5, 5]) is mud (mu 0.25), rock (0.8) elsewhere: the slip-detection course (Task 9)."""
    from napl.terrain import Field, Patch

    res, n = 0.05, int(FIELD_SIZE / 0.05) + 1
    pc = int(10.0 / res)
    grid = np.full((3, 3), "rock", dtype=object)
    grid[:, 1] = "mud"
    patches = [Patch(-15 + a * 10, -15 + b * 10, -5 + a * 10, -5 + b * 10, str(grid[b, a]), a * pc, (a + 1) * pc, b * pc, (b + 1) * pc)
               for b in range(3) for a in range(3)]  # fmt: skip
    f = Field(
        "slopes",
        FIELD_SIZE,
        res,
        0,
        np.zeros((n, n)),
        patches,
        [],
        None,
        [],
        10.0,
        {"slip": True},
    )
    f.grid = grid
    return f


@lru_cache(maxsize=8)
def _field(course, seed):
    if course == "criterion":
        return criterion_course()
    if course == "slip":
        return slip_course()
    from napl.terrain import field

    s = (
        FIELD_SIZE / 100.0
    ) ** 2  # the SP2 object counts are per 100 x 100 m: keep their per-area statistics
    kw = dict(
        rubble=dict(rocks=int(2500 * s), boulders=round(40 * s)),
        scree=dict(rocks=int(6000 * s)),
    ).get(course, {})
    return field(course, size=FIELD_SIZE, seed=int(seed), **kw)


def field_of(inf):
    """The napl.terrain Field of a scene's course (None on the flat floor): a pure function of (course, seed)."""
    return None if inf["course"] == "flat" else _field(inf["course"], inf["seed"])


def route(f, rng, length=(15.0, 20.0), res=0.1, max_step=0.20, inflate=0.3, tries=50):
    """Truth-traversable route for the HARNESS (SP5 plans on its own map): (n, 2) waypoints every 0.5 m. Cells are blocked where any
    8-neighbour differs by more than max_step (boulders, ruin walls; the 10-18 cm steps of `slopes` and rubble rocks stay open), dilated
    by `inflate`; start and goal are free cells whose distance is within `length`; A* with cost distance + 2 |dz|."""
    m = f.size / 2 - 1.0
    xs = np.arange(-m, m + 1e-9, res)
    X, Y = np.meshgrid(xs, xs)
    H = f.height(X, Y)
    nb = [(a, b) for a in (-1, 0, 1) for b in (-1, 0, 1) if (a, b) != (0, 0)]
    d = np.max([np.abs(H - np.roll(H, (a, b), (0, 1))) for a, b in nb], axis=0)
    blocked = d > max_step
    k = int(round(inflate / res))
    grown = blocked.copy()
    for a in range(-k, k + 1):
        for b in range(-k, k + 1):
            if a * a + b * b <= k * k:
                grown |= np.roll(blocked, (a, b), (0, 1))
    grown[[0, -1], :] = grown[:, [0, -1]] = True
    free = np.argwhere(~grown)
    for _ in range(tries):
        s, g = free[rng.integers(len(free))], free[rng.integers(len(free))]
        if length[0] <= np.hypot(*(s - g)) * res <= length[1]:
            path = _astar(grown, H, s, g, res)
            if path is not None:
                p = np.column_stack([xs[path[:, 1]], xs[path[:, 0]]])
                cum = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
                if cum[-1] > 2.0 * length[1]:
                    continue  # a maze detour: draw another pair
                t = np.append(np.arange(0.0, cum[-1], 0.5), cum[-1])
                return np.column_stack(
                    [np.interp(t, cum, p[:, 0]), np.interp(t, cum, p[:, 1])]
                )
    raise RuntimeError(f"no route of length {length} on this field after {tries} draws")


def _astar(blocked, H, s, g, res):
    n = blocked.shape[0]
    steps = [
        (a, b, np.hypot(a, b) * res)
        for a in (-1, 0, 1)
        for b in (-1, 0, 1)
        if (a, b) != (0, 0)
    ]
    s, g = tuple(int(v) for v in s), tuple(int(v) for v in g)
    best, prev, heap = {s: 0.0}, {}, [(0.0, 0.0, s)]
    while heap:
        _, c0, u = heapq.heappop(heap)
        if u == g:
            out = [u]
            while out[-1] in prev:
                out.append(prev[out[-1]])
            return np.array(out[::-1])
        if c0 > best[u]:
            continue
        for a, b, dist in steps:
            v = (u[0] + a, u[1] + b)
            if not (0 <= v[0] < n and 0 <= v[1] < n) or blocked[v]:
                continue
            c1 = c0 + dist + 2.0 * abs(H[v] - H[u])
            if c1 < best.get(v, np.inf):
                best[v], prev[v] = c1, u
                heapq.heappush(
                    heap, (c1 + np.hypot(v[0] - g[0], v[1] - g[1]) * res, c1, v)
                )
    return None


class Follower:
    """Pure pursuit of a route on the TRUE pose (route planning and localization are SP5): cmd(pos, yaw) -> (vx, vy, wz)."""

    def __init__(self, route, v, lookahead=0.8):
        self.r, self.v, self.la = np.asarray(route, float), float(v), lookahead
        self.cum = np.r_[
            0.0, np.cumsum(np.linalg.norm(np.diff(self.r, axis=0), axis=1))
        ]
        self.length, self.i, self.n = float(self.cum[-1]), 0, 0
        self.dist, self.done, self.t_done = self.length, False, None

    def cmd(self, pos, yaw):
        self.n += 1
        xy = np.asarray(pos, float)[:2]
        w = np.linalg.norm(self.r[self.i : self.i + 12] - xy, axis=1)
        self.i += int(np.argmin(w))
        self.dist = float(
            self.length - self.cum[self.i] + np.linalg.norm(self.r[self.i] - xy)
        )
        if not self.done and np.linalg.norm(self.r[-1] - xy) < 0.5:
            self.done, self.t_done = True, self.n * 0.002
        if self.done:
            self.dist = float(np.linalg.norm(self.r[-1] - xy))
            return (0.0, 0.0, 0.0)
        far = np.flatnonzero(np.linalg.norm(self.r[self.i :] - xy, axis=1) >= self.la)
        tgt = self.r[self.i + (far[0] if len(far) else len(self.r) - self.i - 1)]
        e = (np.arctan2(tgt[1] - xy[1], tgt[0] - xy[0]) - yaw + np.pi) % (
            2 * np.pi
        ) - np.pi
        return (
            self.v * max(np.cos(e), 0.0) ** 2,
            0.0,
            float(np.clip(2.0 * e, -1.5, 1.5)),
        )


def _place(root, x, y, yaw, z):
    b = next(
        b for b in root.find("worldbody").findall("body") if b.get("name") == "deck_top"
    )
    b.set("pos", f"{x:.6g} {y:.6g} {z:.6g}")
    b.set("quat", f"{np.cos(yaw / 2):.9g} 0 0 {np.sin(yaw / 2):.9g}")


def scene(out, course, seed=1, payload=None):
    """-> scene.xml path in `out`. course: one of COURSES. payload = (kg, dx, dy) or None."""
    if course not in COURSES:
        raise ValueError(f"course {course!r}: choose from {COURSES}")
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    nominal = base_model()
    root = ET.parse(nominal).getroot()
    strip_lidar(root)
    inf = dict(
        course=course,
        seed=int(seed),
        nominal=str(nominal),
        start=[0.0, 0.0, 0.0],
        route=None,
        field=None,
    )
    if course != "flat":
        from napl.terrain import add_field

        f = _field(course, seed)
        wb = root.find("worldbody")
        for g in [g for g in wb.findall("geom") if g.get("type") == "plane"]:
            wb.remove(g)  # the field is the floor now
        add_field(root, f)
        if course in ("criterion", "slip"):
            r = np.column_stack([np.arange(-12.0, 12.0 + 1e-9, 0.5), np.zeros(49)])
        else:
            r = route(f, np.random.default_rng([int(seed), 31]))
        yaw = float(np.arctan2(r[1, 1] - r[0, 1], r[1, 0] - r[0, 0]))
        u, v = np.meshgrid(
            np.linspace(-FOOTPRINT[0] / 2, FOOTPRINT[0] / 2, 7),
            np.linspace(-FOOTPRINT[1] / 2, FOOTPRINT[1] / 2, 5),
        )
        c, s = np.cos(yaw), np.sin(yaw)
        top = float(np.max(f.height(r[0, 0] + c * u - s * v, r[0, 1] + s * u + c * v)))
        z0 = float(
            root.find("worldbody").find("body[@name='deck_top']").get("pos").split()[2]
        )
        _place(root, float(r[0, 0]), float(r[0, 1]), yaw, z0 + top + START_CLEAR)
        inf.update(
            start=[float(r[0, 0]), float(r[0, 1]), yaw], route=r.tolist(), field=course
        )
    if payload:
        _add_payload(root, *payload)
    path = out / "scene.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    (out / "course.json").write_text(json.dumps(inf))
    return path


def _add_payload(root, kg, dx, dy):
    trunk = next(b for b in root.iter("body") if b.get("name") == "deck_top")
    b = ET.SubElement(trunk, "body", name="payload", pos=f"{dx:.6g} {dy:.6g} 0.03")
    ET.SubElement(
        b,
        "geom",
        type="box",
        size="0.03 0.03 0.02",
        mass=f"{kg:.6g}",
        contype="0",
        conaffinity="0",
        group="1",
    )


def info(model):
    return json.loads((Path(model).parent / "course.json").read_text())


def build_for(scen):
    """-> build(out) of a walk scenario's [model] script: that scenario's scene at its parameter defaults."""
    from napl import params as nparams

    def build(out):
        p = nparams.resolve(scen)
        return str(scene(out, p["course"], p["seed"]))

    return build
