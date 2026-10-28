"""Recording harness of the cuas2 end-to-end video (media is never committed: frames and the webm stay on the host). Exact commands: docs/cuas2/video.md.

Same pipeline as tools/record_cuas_lite.py, which this imports instead of copying (Writer, make_plan, the web capture, the build-frame reader, the lite HUD and replay player);
only what is cuas2-specific lives here: the lockstep run with the cuas2 harness (examples/cuas2/run.py: App2 + Sim2RP2040 + the Nerf-flywheel weapon + Referee2), the extra HUD
(Ph, wheel READY, gate state, balls in the hopper), the op log of assemblies/cuas2.toml, the gate state machine walk and the encode.
  build / wiring / program  tools/record_turret.py build --open assemblies/cuas2.toml with the op log made by `oplog`
  sim                       tools/record_sim.py frames --lite with NAPL_PLAYER=record_cuas2:Cuas2Player
  web                       the lite operator page (python/napl/lite/web.py; cuas2 has no page of its own) fed from the run's recorded snapshots
  encode                    tools/videnc.py (GPU encode; libx264 only with --gpl)

    run    SCENARIO OUT_DIR [--seed 1]    the cuas2 lockstep scenario + OUT_DIR/lite_hud.jsonl (App.snapshot + Ph / wheels / gate every 50 ms) + lite_meta.json (file names as lite's,
                                          so make_plan / LitePlayer read them unchanged); OUT_DIR/model/ is the scene, replay.naplog / native.json as any napl run
    verify RUN_DIR                        the replay of the run's log ends on the run's state hash
    plan   RUN_DIR                        the auto-plan (sim windows, speed, camera), printed as JSON
    sim    RUN_DIR OUT_DIR                the planned chunks through record_sim.py frames (one xvfb viewer per chunk, GPU)
    oplog  OUT.json                       the build / wiring / program op log generated from the assembly
    web    RUN_DIR OUT_DIR                the operator page captured in Chromium (L.cmd_web)
    encode ROOT OUT.webm                  ROOT = the work dir holding build/ web/ runs/<scenario>/ sim/<scenario>/ (docs/cuas2/video.md)
    png    FRAMES_DIR OUT_DIR IDX...      frames of a store as PNG, to look at
"""

import argparse
import bisect
import json
import os
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(
    0, str(REPO / "examples" / "cuas2")
)  # before cuas-lite's: `import scene` / `import run` must be the cuas2 ones
sys.path.insert(0, str(TOOLS))
import record_cuas_lite as L
import videnc

ASM = REPO / "assemblies" / "cuas2.toml"
W, H, FPS, HO = L.W, L.H, L.FPS, L.HO
GATE = (
    "UNKNOWN",
    "HOMING",
    "LOADING",
    "TO_BORE",
    "STAGED",
    "PUSH",
    "EXIT_WAIT",
    "RETRACT",
    "TO_LOAD",
    "EMPTY",
    "JAM",
)
NOT_REAL = "Not real: stand-in detector (ground-truth projection + noise), simulated RP2040 (python twin of the C core), estimated firmware timings (bench item B-TIME)."


# ---- run ---------------------------------------------------------------------------------------------------------------------------------------
class _Tee:
    """HudLog's file: every record gets the cuas2 fields (Ph, wheels, gate, hopper) before it is written."""

    def __init__(self, f, extra):
        self.f, self.extra = f, extra

    def write(self, line):
        r = json.loads(line)
        r.update(self.extra())
        self.f.write(json.dumps(r, separators=(",", ":"), default=float) + "\n")

    def close(self):
        self.f.close()


class HudLog2(L.HudLog):
    """lite's HudLog + what is new in cuas2. Observation only (the replay hash must not move). gate_tr = every change of the feeder's gate state, sampled each control period."""

    def __init__(self, path):
        super().__init__(path)
        self.f = _Tee(self.f, self.extra)
        self.gate_tr, self._g, self._ph = [], None, (None, -9.0)

    def extra(self):
        s = self.last
        fo, fp = s.twin.fo, s.weapon.feeder
        sf = fo.status_fields if fo is not None else {}
        return dict(
            ph=None if self._ph[0] is None else round(self._ph[0], 3), ph_t=round(self._ph[1], 3), ph_tau=s.app.fc.cfg.ph_tau, ph_gate=bool(s.app.fc.cfg.ph_gate),
            gate=int(sf.get("gate_state", 0)), ready=int(sf.get("ready_flags", 0)), wheels=bool(sf.get("wheels_ready", False)), rpm=[sf.get("rpm_top"), sf.get("rpm_bot")],
            rpm_set=sf.get("rpm_set"), hopper=int(fp.hopper), pocket=bool(fp.pocket), rail=bool(s.twin.rail_hw), fired=len(s.app.fc.ph_shots),
        )  # fmt: skip

    def __call__(self, s):
        self.last = s
        d = getattr(s.app, "dec", None)
        if d is not None and getattr(d, "ph", None) is not None:
            self._ph = (float(d.ph), s.world.t)
        fo = s.twin.fo
        g = int(fo.status_fields.get("gate_state", 0)) if fo is not None else 0
        if g != self._g:
            self.gate_tr.append([round(s.world.t, 3), g])
            self._g = g
        super().__call__(s)


def cmd_run(a):
    sys.path.insert(
        0, str(REPO / "examples" / "cuas2")
    )  # record_cuas_lite put examples/cuas-lite in front: `run` / `scene` must be the cuas2 ones
    import run as r2  # examples/cuas2/run.py (cuas2 harness)
    import scene as sc
    from napl.cuas2 import protocol as p2  # noqa: F401
    from napl.lite import protocol as p
    from napl.lite.sim_io import build_scene

    os.environ["CUAS_SCENARIO"], os.environ["CUAS_SEED"] = a.scenario, str(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    sp = sc.make(a.scenario, a.seed)
    man = build_scene(
        out / "model",
        sp.targets,
        env=sp.env,
        seed=a.seed,
        launcher=sp.launcher,
        base=sc.cached_base(),
    )
    log = HudLog2(out / "lite_hud.jsonl")
    orig, t0 = r2.lrun.run_episode, time.time()

    def with_hook(
        model, run_dir, sp_, seed, **kw
    ):  # cuas2's run_episode installs its own hook: chain ours behind it
        mine, theirs = log, kw.pop("hook", None)
        return orig(
            model,
            run_dir,
            sp_,
            seed,
            hook=lambda s: (theirs and theirs(s), mine(s)),
            **kw,
        )

    r2.lrun.run_episode = with_hook
    try:
        res = r2.run_episode(man["model"], out, sp, a.seed)
    finally:
        r2.lrun.run_episode = orig
    s = log.last
    log.f.close()
    meta = dict(
        scenario=a.scenario, seed=a.seed, arm_at=res["arm_at"], shot_t=res["shot_t"], popped_t=res["popped_t"], hit_log=[list(x) for x in s.hit_log],
        transitions=[[t, p.STATE_NAMES[st], r] for t, st, r in s.twin.transitions], gate_transitions=[[t, GATE[g]] for t, g in log.gate_tr], t_cut=res.get("t_cut"),
        fault=sp.fault, targets=[dict(id=t.id, cls=t.klass, size=t.size_m, kind=getattr(t, "kind", "")) for t in sp.targets], sector=sc.SECTOR.asdict(),
        sim_s=res["sim_s"], ok=res["ok"], checks=res["checks"], state_hash=res["state_hash"], rounds=res["rounds"], hits_on_targets=res["hits_on_targets"],
        violations=res["violations"], ph_fired=res.get("ph_fired"), cfg=res.get("cfg"), hopper_end=int(s.weapon.feeder.hopper), wall_s=round(time.time() - t0, 1),
    )  # fmt: skip
    (out / "lite_meta.json").write_text(json.dumps(meta, indent=1, default=float))
    print(
        json.dumps(
            {
                k: meta[k]
                for k in (
                    "scenario",
                    "ok",
                    "state_hash",
                    "rounds",
                    "hits_on_targets",
                    "sim_s",
                    "wall_s",
                    "ph_fired",
                )
            },
            default=float,
        )
    )
    sys.exit(0 if res["ok"] else 1)


# ---- HUD + replay player -----------------------------------------------------------------------------------------------------------------------
def cuas2_items(rec, t, H_, vp):
    """The cuas2 panel (bottom left) and the not-real line: Ph of the last decision, wheel READY, gate state, balls fed / in the hopper."""
    Wv, Hv = vp
    ui, m = Hv / 720.0, 16 * Hv / 720.0
    if rec is None:
        return []
    s = rec["snap"]
    ph = rec.get("ph")
    ph_txt = "-" if ph is None or t - rec["ph_t"] > 1.5 else f"{ph:.2f}"
    if rec["ph_gate"] and ph_txt != "-":
        ph_txt += f"  (gate {rec['ph_tau']:.2f}: {'pass' if ph >= rec['ph_tau'] else 'BLOCK'})"
    rt, rb = rec["rpm"]
    rdy = bool(rec["ready"] & 1)
    rows = [
        ("Ph (hit probability)", ph_txt),
        ("WHEELS top / bottom rpm", "off" if rt in (None, 0xFFFF) else f"{rt} / {rb}"),
        ("WHEELS READY (both in window)", "READY" if rec["wheels"] else "no"),
        ("FEED READY (ball staged + wheels)", "READY" if rdy else "no"),
        ("GATE", GATE[rec["gate"]] + ("  ball in pocket" if rec["pocket"] else "")),
        ("BALLS fed / hopper left", f"{s['shots']} / {rec['hopper']}"),
        ("WEAPON RAIL", "on" if rec["rail"] else "CUT"),
    ]
    return [
        H_.kv(rows, (m, Hv - m - 24 * ui), a="bl", size=12 * ui, pad=9 * ui, gap=20 * ui),
        H_.text("STAND-IN DETECTOR  -  SIMULATED RP2040  -  ESTIMATED FIRMWARE TIMINGS", (m, Hv - m), a="bl", size=10.5 * ui, col=H_.AMBER, bg=(0, 0, 0, 220), pad=5 * ui),
    ]  # fmt: skip


class Cuas2Player(L.LitePlayer):
    """LitePlayer (the replay of replay.naplog, lite HUD) + the cuas2 panel. record_sim.py picks it up through NAPL_PLAYER=record_cuas2:Cuas2Player."""

    def document(self, seq, size=(L.W, L.H), dt=1 / 20):
        d = json.loads(super().document(seq, size, dt))
        t = self.t
        j = bisect.bisect_right(self.hud_t, t) - 1
        d["items"] += cuas2_items(self.hud[j] if j >= 0 else None, t, self.H, size)
        return json.dumps(d, separators=(",", ":"))


def cmd_verify(a):
    h, ok = Cuas2Player(a.run_dir).finish()
    print(f"replay hash {h} {'MATCHES' if ok else 'DIFFERS FROM'} native.json")
    sys.exit(0 if ok else 1)


def cmd_sim(a):
    os.environ["NAPL_PLAYER"] = (
        "record_cuas2:Cuas2Player"  # inherited by the xvfb viewer subprocess of record_sim.py
    )
    L.cmd_sim(a)


# ---- the op log: build / wiring / program ------------------------------------------------------------------------------------------------------
BUILD_STAGES = [  # (title, why, instance ids)
    ("Base plate, four legs, lower deck, power box",
     "Printed in PLA/PETG from cad/cuas2 (build123d: one source for the STL, the STEP and the simulation meshes). The base plate is the root: frame B, pan axis through its datum. Four legs, the lower deck (Jetson, fuse, XT60) and the power box with its lid under the plate.",
     ["base", "leg1", "leg2", "leg3", "leg4", "deck", "box", "lid"]),
    ("Pan drive: NEMA 17 below the plate, 20T GT2 pulley, hall switch",
     "17HS4401S stepper under the base plate, shaft up through the slot, 20-tooth GT2 pulley, A3144 hall switch for homing. The belt (a closed GT2 loop, not modelled) drives the printed pulley on the rotor.",
     ["nema_pan", "pulley_pan", "hall_pan"]),
    ("Lazy susan, rotor plate, yokes, bearings, tilt drive",
     "The lazy susan carries the rotor plate, the two printed yokes and their bearings: the pan group, +-170 deg. The second NEMA 17 with its pulley, a hall switch and a magnet drive and home the tilt axis (-15 to +70 deg).",
     ["lazy_susan", "rotor", "mag_pan", "yoke_l", "yoke_r", "brg_l", "brg_r", "nema_tilt", "pulley_tilt", "hall_tilt", "mag_tilt"]),
    ("Launcher cartridge: cage, gap plate, two brushless flywheels",
     "Module M3: the cage and gap plate hold two wheel hubs with TPU tyres (wheel-surface gap 19.2 mm for the 22.2 mm foam ball), each on a brushless 2205 motor with its own ESC (DShot, eRPM telemetry): closed-loop rpm per wheel.",
     ["cradle", "cage", "gap_plate", "hub_t", "hub_b", "tyre_t", "tyre_b", "motor_t", "motor_b", "esc_t", "esc_b", "ballast_cup"]),
    ("One-ball feeder: hopper, plug valve, pusher",
     "Module M4: a 24-ball hopper on the tilt head with a one-ball-wide throat, a plug valve whose 26 mm bore holds exactly one ball, two SG92R servos (gate and pusher), two IR beam pairs (breech, muzzle) and hall switches. One FIRE advances the gate through LOAD, BORE, PUSH and back: one ball.",
     ["valve", "plug", "rod", "hopper", "hopper_lid", "servo_gate", "servo_push", "hall_gate", "mag_gate", "hall_push", "mag_push", "led_mz", "pt_mz", "led_br", "pt_br", "launcher"]),
    ("Sensor head: Logitech Brio",
     "Module M5: the sensor bar and the Brio 4K, co-aligned with the barrel on the tilt head.",
     ["sensor_bar", "brio"]),
    ("Compute and motion electronics: Jetson, Pico, two TMC2209",
     "Jetson Orin Nano (the app), RP2040 Pico (the firmware, USB serial to the Jetson), two TMC2209 drivers for the steppers, the status LED.",
     ["jetson", "pico", "tmc_pan", "tmc_tilt", "led_status"]),
    ("Weapon-rail safety: XT60, fuse, key, E-stop, relay, supervisor, limiter, shot counter",
     "XT60 inlet and 20 A fuse; arm key and E-stop in series drive a 40 A relay coil; the relay contact feeds both ESCs and the 5 V buck. A 74HCT123 supervisor (kicked by the Pico) and a 74HCT4017 hardware shot counter hold a MOSFET that opens the rail without any firmware. Then the pull-ups, dividers and the rest of the passives.",
     ["xt60", "fuse", "key", "estop", "relay", "buck", "reg", "d_fly", "q_sup", "q_p", "q_clamp", "ic_sup", "ic_lim", "cnt_u", "cnt_t"]),
]  # fmt: skip
SHEETS = [  # (title, why, instances)
    ("Power: XT60, fuse, 12 V bus, key, E-stop and the relay coil",
     "Sheet 1. XT60 inlet to the 20 A fuse and the 12 V bus: Jetson barrel, both TMC2209 VM pins and the key. Key (COM-NO) then the NC E-stop drive the relay coil: key off or E-stop pressed cuts the weapon rail in hardware. Every connector shows its real pins.",
     ["psu", "xt60", "fuse", "jetson", "tmc_pan", "tmc_tilt", "key", "estop", "relay", "reg", "d_fly"]),
    ("Weapon rail: relay contact, two ESCs, 5 V buck, servos, pusher power window",
     "Sheet 2. The relay contact feeds the two ESCs (brushless flywheel motors) and the LM2596 buck; the buck powers the gate servo and, through the P-FET window of the limiter, the pusher servo; ESC signal and telemetry go to the Pico.",
     ["relay", "xt60", "esc_t", "esc_b", "motor_t", "motor_b", "buck", "servo_gate", "servo_push", "q_p", "r_pg", "q_sup", "r_gate", "r_gpd", "pico"]),
    ("Hardware supervisor, feed limiter and shot counter",
     "Sheet 3. 74HCT123 supervisor kept alive only by the Pico's 100 Hz KICK, second 74HCT123 limiting one FEED_REQ to a 254 ms pusher window, 74HCT4017 counting balls on the muzzle beam and clamping the MOSFET gate at N = 10; reset only by a key power cycle.",
     ["ic_sup", "ic_lim", "cnt_u", "cnt_t", "q_clamp", "q_sup", "q_p", "pico", "reg", "c_on", "c_hold", "c_sup", "c_por", "r_on", "r_hold", "r_clr_lim", "r_fr", "r_sup", "r_sup_up",
      "r_sup_dn", "r_gate", "r_gpd", "r_cb", "r_ce", "r_por", "r_pt_mz"]),
    ("Pico to the TMC2209 drivers and the NEMA 17 motors",
     "Sheet 4. STEP / DIR / EN and the MS1 / MS2 straps from the Pico, the single-wire UART through 1 k resistors, and the four motor leads of each stepper.",
     ["pico", "tmc_pan", "tmc_tilt", "nema_pan", "nema_tilt", "r_en", "r_ms_pan", "r_ms_tilt", "r_su_pan", "r_su_tilt"]),
    ("Pico sensing: halls, beam pairs, key and E-stop dividers, USB",
     "Sheet 5. Hall outputs with pull-ups (pan, tilt, gate, pusher), the IR LED / phototransistor pairs on the breech and muzzle, the 33k / 10k dividers that let the Pico read the key and E-stop nodes, the status LED, and the USB links.",
     ["pico", "jetson", "brio", "hall_pan", "hall_tilt", "hall_gate", "hall_push", "r_hp", "r_ht", "r_hg", "r_hpu", "led_mz", "led_br", "pt_mz", "pt_br", "r_led_mz", "r_led_br",
      "r_pt_mz", "r_pt_br", "key", "estop", "r_kt", "r_kb", "r_et", "r_eb", "led_status"]),
]  # fmt: skip
SRC_PAGES = [  # (title, why, file, [regex or scroll fraction])
    ("CAD: every dimension a named parameter", "cad/cuas2/params.py. The wheel gap, pocket bore, hopper and stroke are parameters; build.py checks C-GAP, C-POCKET, C-FEED, C-NIP, C-INTERLOCK, C-BAL, C-DOCK, C-PATH, C-FIT and exports STL, STEP and sim meshes.", "cad/cuas2/params.py", [0.0, 0.3]),
    ("Firmware: the one-ball gate state machine", "firmware/cuas2-rp2040/src/feeder.c (C on pico-sdk). fdr_step() walks HOMING, LOADING, TO_BORE, STAGED, PUSH, EXIT_WAIT, RETRACT, TO_LOAD for exactly one ball per FIRE; EMPTY and JAM are the exits. Timings in include/fw_core.h are estimates.", "firmware/cuas2-rp2040/src/feeder.c", [r"static void enter", r"static uint8_t fire_result", r"void fdr_step"]),
    ("Firmware: flywheel closed loop and the READY window", "firmware/cuas2-rp2040/src/launcher.c. lau_step() runs the per-wheel rpm loop on eRPM telemetry (filtered, tau 8 ms); READY needs both wheels inside the rpm window.", "firmware/cuas2-rp2040/src/launcher.c", [r"int32_t fdr_rpm_cap", r"void lau_step"]),
    ("Firmware: the safety state machine", "firmware/cuas2-rp2040/src/fsm.c. to_state() is the one place a transition happens: SAFE, IDLE, ARMED, ENGAGING, FIRING, FAULT; cmd_fire() hands one FIRE to the feeder. E-stop, key and heartbeat watchdog send everything to SAFE.", "firmware/cuas2-rp2040/src/fsm.c", [r"static void to_state", r"static void cmd_fire", r"static void cmd_arm"]),
    ("Jetson app: hit probability gate and shoot-look-shoot", "python/napl/cuas2/app.py. hit_probability() samples the tracker's predictive distribution through the launcher model (Ph); FireControl2._step() fires only above the Ph threshold, with wheels READY and no person in view.", "python/napl/cuas2/app.py", [r"^def hit_probability", r"def _step"]),
    ("The lockstep harness", "examples/cuas2/run.py. The real App2, the simulated RP2040 and the cuas2 weapon, judged ball by ball by Referee2 (person, armed session, cap, E-stop, geofence).", "examples/cuas2/run.py", [r"^class Referee2", r"^def run_episode"]),
]  # fmt: skip


def cmd_oplog(a):
    """Op log for tools/record_turret.py build --open assemblies/cuas2.toml. Needs the Design (run it on the host); the card centres cost a full design build and are cached beside the output."""
    import hashlib
    import tomllib

    from napl.design import Design
    from napl.design.viewer import Backend

    doc = tomllib.loads(ASM.read_text())
    ids = [i["id"] for i in doc["instances"]]
    on = [i["id"] for i in doc["instances"] if not i.get("offboard")]
    staged = [x for st in BUILD_STAGES for x in st[2]]
    assert len(staged) == len(set(staged)), "an instance is in two build stages"
    assert set(staged) <= set(ids), f"unknown ids {set(staged) - set(ids)}"
    stages = [list(s) for s in BUILD_STAGES]
    stages[-1][2] = (
        stages[-1][2] + [x for x in on if x not in staged]
    )  # what the stages did not name (resistors, caps) is shown with the safety electronics
    R = lambda tool, wait=0.0, **args: {"tool": tool, "args": args, "wait": wait}
    nwire, nmate = len(doc["wires"]), len(doc["mates"])
    steps = [
        dict(seg="CAD", title="The parts are code: build123d parameters", speed=1.0,
             why="cad/cuas2/params.py: every dimension is a named parameter; modules/*.py turn them into the six modules; build.py checks the geometry (wheel gap, one-ball pocket, interlock, balance) and exports the STL, STEP and simulation meshes from the same source.",
             reqs=[R("ui.mode", 1.0, name="program"), R("ui.program", 3.5, file="cad/cuas2/params.py", scroll=0.1), R("ui.program", 4.0, file="cad/cuas2/modules/feeder.py", scroll=0.2)]),
        dict(seg="BUILD", title="The finished design, every part hidden", speed=1.0,
             why=f"assemblies/cuas2.toml: {len(ids)} instances ({len(on)} on the turret), {nmate} mates, {nwire} wires. The BROWSER's eye hides parts in the 3D view; the parts now come back in the order they are built.",
             reqs=[R("ui.mode", 1.0, name="build"), R("select", 0.3, instances=[]), R("ui.visibility", 1.2, visible=False), R("ui.camera", 0.8, preset="iso")]),
    ]  # fmt: skip
    shown = []
    for n, (title, why, grp) in enumerate(stages, 1):
        grp = [x for x in grp if x in ids]
        shown += grp
        reqs = [
            R("ui.visibility", 0.6, instances=grp, visible=True),
            R("ui.frame", 1.2, instances=shown),
            R("select", 0.2, instances=[]),
        ]  # ui.frame selects what it frames
        reqs += [
            R(
                "camera.orbit",
                4.2,
                yaw_deg=75,
                pitch_deg=8 if n == 1 else 0,
                seconds=4.0,
                relative=True,
            )
        ]
        steps.append(
            dict(
                seg="BUILD",
                title=f"{n}/{len(stages)}  {title}",
                why=why,
                speed=1.0,
                dwell=0.4,
                reqs=reqs,
            )
        )
    steps.append(dict(seg="BUILD", title="Mass and DRC", speed=1.0, dwell=1.0,
                      why="The Design reports mass and the design-rule check: no wire to an unknown pin, no supply fed twice, no connector mismatch.",
                      reqs=[R("ui.visibility", 0.5, visible=True), R("ui.frame", 1.0, instances=[]), R("design_check", 2.5)]))  # fmt: skip
    cpath = Path(a.out).with_suffix(".cards.json")
    sha = hashlib.sha1(ASM.read_bytes() + repr(SHEETS).encode()).hexdigest()
    cached = json.loads(cpath.read_text()) if cpath.exists() else {}
    cached = cached if cached.get("sha") == sha else {"sha": sha}
    d = Design.open(ASM)
    b = Backend(lambda: d, Path("/tmp/napl-oplog"))
    have = set(ids)
    for n, (title, why, inst) in enumerate(SHEETS, 1):
        inst = [i for i in inst if i in have]
        ports = sorted(
            {
                w[e]
                for w in doc["wires"]
                for e in ("from", "to")
                if w["from"].partition(".")[0] in inst
                and w["to"].partition(".")[0] in inst
            }
        )
        if title not in cached:
            r = json.loads(
                b(
                    json.dumps(
                        {
                            "tool": "flatten.begin",
                            "args": {"instances": inst, "ports": ports},
                        }
                    )
                )
            )
            if not r["ok"]:
                raise SystemExit(f"sheet {n}: {r.get('error')}")
            cards = r["flatten"]["cards"]
            wired = lambda c: sum(1 for p in c["pins"] if p.get("wired"))
            key = sorted(
                sorted(cards, key=wired, reverse=True)[:3], key=lambda c: c["x"]
            )
            cached[title] = [
                [round(c["x"] + c["w"] / 2), round(c["y"] + c["h"] / 2)] for c in key
            ]
            b(json.dumps({"tool": "flatten.end", "args": {}}))
            cpath.write_text(json.dumps(cached))
        reqs = [
            R("ui.mode", 1.4, name="wiring"),
            R("select", 1.0, instances=inst),
            R("flatten.begin", 3.0, instances=inst, ports=ports),
            R("ui.wire_view", 3.0, fit=True),
        ]
        reqs += [
            R("ui.wire_view", 4.4, zoom=0.9, center=c, seconds=2.4)
            for c in cached[title]
        ]
        reqs += [R("ui.wire_view", 2.4, fit=True, seconds=1.6), R("flatten.end", 1.0)]
        steps.append(
            dict(seg="WIRING", title=title, why=why, speed=1.0, dwell=0.6, reqs=reqs)
        )
    steps.append(dict(seg="WIRING", title=f"Back to 3D: the {nwire} wires are tubes routed around the parts", speed=1.0, dwell=0.6,
                      why="The wires are already in the design. In 3D each conductor is a tube with its gauge's diameter, leaving its connector along the connector axis and routed through free space (a distance field of the assembly, A*), never through a part.",
                      reqs=[R("ui.mode", 1.2, name="build"), R("ui.view", 0.5, emphasise_wires=True), R("select", 0.5, instances=[]), R("ui.frame", 1.2, instances=[]),
                            R("camera.orbit", 8.5, yaw_deg=300, pitch_deg=12, seconds=8.0, relative=True)]))  # fmt: skip
    for title, why, f, pats in SRC_PAGES[1:]:  # the CAD page was shown first
        steps.append(
            dict(
                seg="PROGRAM",
                title=title,
                why=why,
                speed=1.0,
                dwell=0.4,
                reqs=[R("ui.mode", 1.0, name="program")]
                + [R("ui.program", 4.2, file=f, scroll=L._scroll(f, p)) for p in pats],
            )
        )
    Path(a.out).write_text(
        json.dumps(
            {
                "about": "generated by tools/record_cuas2.py oplog from assemblies/cuas2.toml",
                "steps": steps,
            },
            indent=1,
        )
    )
    print(f"{len(steps)} steps -> {a.out}")


# ---- encode ------------------------------------------------------------------------------------------------------------------------------------
SIM_SCENES = [  # (scenario, title, what it is)
    ("s1_balloon_static", "S1  A hung balloon", "One static balloon. The app tracks it, plans the lead, computes Ph with the launcher model, and the feeder meters one ball per FIRE."),
    ("s2_balloon_drift", "S2  Balloons drifting in the wind", "Balloons carried back and forth by a 1 m/s wind; the app corrects for the wind it estimates."),
    ("s5_mix", "S5  A mix: hung balloon, drifting balloon, 4 m/s quad", "Nearest-first engagement under a shot cap; the Ph gate withholds shots it expects to miss."),
    ("s6_person_walk", "S6  Safety: a person walks through the view", "Hung balloons, ARM at 8 s. A person crosses the image: FIRING INHIBITED while the person is in view or in the 1 s hold after; the wheels stay spun."),
    ("s7_fault_estop", "S7  Fault: the E-stop is pressed mid-engagement", "The E-stop opens: the weapon rail drops in hardware, the MCU goes SAFE, and it stays safe until the operator re-arms."),
]  # fmt: skip
CH = {
    "build": ("1  Build", ["The parametric CAD (build123d) and the parts of assemblies/cuas2.toml appearing in build order, the camera orbiting: the napl viewer's CAD-style build workspace on the finished design."]),
    "wiring": ("2  Wiring", ["Pin-level wiring sheets of the real assembly (Pico, two TMC2209, ESCs, supervisor, limiter, shot counter, halls, beam pairs), then the 153 wires as tubes routed around the parts in 3D."]),
    "program": ("3  Program", ["The firmware (gate state machine, flywheel loop, safety FSM), the Jetson app's hit-probability gate, the operator page, and the gate state machine walked through a real run."]),
}  # fmt: skip


def _gate_walk(w, root, scn, until, label):
    """The two state machines (RP2040 safety FSM above, one-ball gate below) walked through the transitions of a real run: lite_meta.json transitions / gate_transitions,
    wheel / Ph numbers from the run's HUD log at each transition. until(meta) -> (t0, t1) window."""
    run = root / "runs" / scn
    if not (run / "lite_meta.json").exists():
        return
    meta = json.loads((run / "lite_meta.json").read_text())
    hud = [
        json.loads(x)
        for x in (run / "lite_hud.jsonl").read_text().splitlines()
        if x.strip()
    ]
    hts = [r["t"] for r in hud]
    t0, t1 = until(meta)
    ev = [(t, "mcu", st, r) for t, st, r in meta["transitions"]] + [
        (t, "gate", st, 0) for t, st in meta["gate_transitions"]
    ]
    ev = sorted(e for e in ev if t0 <= e[0] <= t1)
    if not ev:
        return
    mcu = ("SAFE", "IDLE", "ARMED", "ENGAGING", "FIRING", "FAULT")
    px = {n: (120 + 208 * i, 140) for i, n in enumerate(mcu)}
    gx = {"HOMING": (110, 290), "LOADING": (330, 290), "TO_BORE": (550, 290), "STAGED": (770, 290), "PUSH": (990, 290), "EXIT_WAIT": (990, 410), "RETRACT": (770, 410),
          "TO_LOAD": (550, 410), "EMPTY": (330, 410), "JAM": (1170, 350)}  # fmt: skip
    pos = {
        **{("mcu", k): v for k, v in px.items()},
        **{("gate", k): v for k, v in gx.items()},
    }
    cyc = list(
        zip(
            [
                "HOMING",
                "LOADING",
                "TO_BORE",
                "STAGED",
                "PUSH",
                "EXIT_WAIT",
                "RETRACT",
                "TO_LOAD",
            ],
            [
                "LOADING",
                "TO_BORE",
                "STAGED",
                "PUSH",
                "EXIT_WAIT",
                "RETRACT",
                "TO_LOAD",
                "LOADING",
            ],
        )
    )
    edges = (
        [("mcu", a_, b_) for a_, b_ in zip(mcu, mcu[1:5])]
        + [("gate", a_, b_) for a_, b_ in cyc]
        + [("gate", "LOADING", "EMPTY"), ("gate", "PUSH", "JAM")]
    )
    F, cur = w.F, {"mcu": "SAFE", "gate": "UNKNOWN"}
    for i, (t, kind, st, rsn) in enumerate(ev):
        src = cur[kind]
        cur[kind] = st
        j = max(0, bisect.bisect_right(hts, t) - 1)
        r = hud[j]
        for _ in range(int(1.2 * FPS)):
            img = w.Image.new("RGB", (W, HO), (10, 11, 13))
            dr = w.Draw.Draw(img, "RGBA")
            dr.text(
                (24, 16),
                f"PROGRAM   {label}: RP2040 safety FSM (fsm.c) and the one-ball gate (feeder.c), walked through the transitions of the run {scn}",
                font=F["h"],
                fill=(255, 196, 60),
            )
            dr.text((24, 36), "safety FSM", font=F["sm"], fill=(150, 155, 165))
            dr.text(
                (24, 214),
                "one-ball gate (one FIRE = one trip round the ring)",
                font=F["sm"],
                fill=(150, 155, 165),
            )
            for k_, a_, b_ in edges:
                (x0, y0), (x1, y1) = pos[(k_, a_)], pos[(k_, b_)]
                dr.line([(x0, y0), (x1, y1)], fill=(120, 125, 135, 255), width=2)
            if src != st and (kind, src) in pos:
                (x0, y0), (x1, y1) = pos[(kind, src)], pos[(kind, st)]
                dr.line([(x0, y0), (x1, y1)], fill=(110, 235, 140, 255), width=5)
            for (k_, n_), (x, y) in pos.items():
                on = cur[k_] == n_
                bad = n_ in ("SAFE", "FAULT", "JAM", "EMPTY")
                dr.rounded_rectangle([x - 88, y - 24, x + 88, y + 24], radius=10, fill=((150, 50, 45, 255) if bad else (40, 120, 70, 255)) if on else (28, 30, 36, 255),
                                     outline=(255, 255, 255, 220 if on else 70), width=3 if on else 1)  # fmt: skip
                dr.text(
                    (x - dr.textlength(n_, font=F["b"]) / 2, y - 12),
                    n_,
                    font=F["b"],
                    fill=(255, 255, 255),
                )
            rt, rb = r["rpm"]
            ph = "-" if r.get("ph") is None else f"{r['ph']:.2f}"
            dr.text(
                (24, 480),
                f"sim t = {t:6.2f} s   {kind}: {src} -> {st}"
                + (f"   reason: {L.REASON_NAMES[rsn]}" if kind == "mcu" else ""),
                font=F["m"],
                fill=(230, 235, 245),
            )
            dr.text(
                (24, 506),
                f"wheels {rt}/{rb} rpm  window {'READY' if r['wheels'] else 'not ready'}   Ph {ph}   balls fed {r['snap']['shots']}  hopper {r['hopper']}   rail {'on' if r['rail'] else 'CUT'}",
                font=F["m"],
                fill=(190, 200, 215),
            )
            for jj, (t2, k2, s2, r2) in enumerate(ev[max(0, i - 4) : i + 1]):
                dr.text(
                    (24, 540 + 22 * jj),
                    f"{t2:6.2f} s  {k2:4s} {s2:10s}",
                    font=F["m"],
                    fill=(150, 235, 170) if t2 == t and s2 == st else (150, 155, 165),
                )
            note = (
                "Any state goes to SAFE when the heartbeat is missing > 300 ms, the E-stop opens, the key goes off, or a new session starts. A FIRE moves the gate once round the ring: HOMING and LOADING put one ball in the pocket, TO_BORE and STAGED line it up with the rod and the wheels, PUSH to EXIT_WAIT is the shot (muzzle beam confirms), RETRACT and TO_LOAD close the gate. "
                + NOT_REAL
            )
            for k2, sub in enumerate(w.wrap(note, F["s"], W - 48, dr)[:4]):
                dr.text((24, H + 8 + 19 * k2), sub, font=F["s"], fill=(190, 196, 208))
            w.emit(img)


def cmd_encode(a):
    root = Path(a.root)
    enc = videnc.Encoder(Path(a.out), W, HO, FPS, gpl=a.gpl)
    w = L.Writer(enc)
    w.card("cuas2: a Nerf-flywheel counter-drone turret, from CAD to simulated engagement", [
        "A modular 3D-printable pan-tilt turret that fires Nerf Rival foam balls from a custom dual brushless flywheel launcher, one ball per FIRE through a single-pocket plug-valve gate (Jetson Orin Nano, Logitech Brio, RP2040 + two TMC2209, relay weapon rail with a hardware supervisor, feed limiter and shot counter).",
        "Everything on screen is the napl code path: the Design backend, the DRC, the build, the MuJoCo-based kernel, the real Jetson app (tracker, lead solver, hit-probability gate) and a simulated RP2040. Targets: balloons, hanging targets and small quads; a person in view always blocks fire. Details and limits: docs/cuas2/video.md",
    ], 7.0, "title")  # fmt: skip
    w.card("What is not real", [
        "Stand-in detector: the sim camera has no pixels; detections are the ground-truth projection with a noise model, not YOLO.",
        "Simulated RP2040: a python twin of the C core (tests compare them tick by tick), not a Pico on a bench.",
        "Estimated firmware timings: gate travel, pusher stroke, feed cycle and the flywheel dip are [est] numbers until the bench items of docs/cuas2/bench_validation.md are done.",
        "The simulated turret body is the cuas-lite CAD (assemblies/cuas2.toml carries no launcher / camera devices into the sim yet); the cuas2 launcher and feeder behind it are the cuas2 model.",
    ], 9.0, "NOT REAL")  # fmt: skip
    L._build_frames(w, root, a.speedup, chapters_over=CH)
    L._web_frames(w, root)
    _gate_walk(
        w,
        root,
        "s1_balloon_static",
        lambda m: (0.0, (m["shot_t"][1] if len(m["shot_t"]) > 1 else m["sim_s"]) + 1.0),
        "two balls",
    )
    w.card(
        "4  Simulate",
        [
            "Each scenario is a lockstep run of the real Jetson app against the simulated RP2040 and the napl kernel; replay.naplog is played back with the HUD (Ph, wheel READY, gate, ball count), real time with slow motion around hits. Every run ends on the replay hash printed on the result card. "
            + NOT_REAL
        ],
        4.5,
        "SIMULATE",
    )
    results = []
    for scn, title, blurb in SIM_SCENES:
        rd, fd = root / "runs" / scn, root / "sim" / scn
        if not (rd / "lite_meta.json").exists() or not fd.exists():
            continue
        meta = json.loads((rd / "lite_meta.json").read_text())
        hud = [
            json.loads(x)
            for x in (rd / "lite_hud.jsonl").read_text().splitlines()
            if x.strip()
        ]
        hts = [r["t"] for r in hud]
        w.card(title, [blurb], 3.0, title)
        if scn == "s7_fault_estop":
            _gate_walk(
                w,
                root,
                scn,
                lambda m: ((m["t_cut"] or 0) - 1.0, (m["t_cut"] or 0) + 2.5),
                "E-stop",
            )
        for mp in sorted(fd.glob("meta_*.json")):
            m = json.loads(mp.read_text())
            slow = m["fps"] / FPS
            for k in range(m["frames"]):
                t = m["t0"] + k / m["fps"]
                rec = hud[max(0, bisect.bisect_right(hts, t) - 1)]
                status, bad = L.status_text(rec)
                s = rec["snap"]
                ph = (
                    "-"
                    if rec.get("ph") is None or t - rec["ph_t"] > 1.5
                    else f"{rec['ph']:.2f}"
                )
                w.emit(w.bar(videnc.load_frame(fd, f"f{m['start_index'] + k:05d}"), "simulate", title, blurb + " " + NOT_REAL,
                             [f"sim t = {t:5.1f} s" + (f"   slow motion 1/{slow:g}" if slow > 1.01 else "") + (f"   {1 / slow:g}x" if slow < 0.99 else ""),
                              f"Ph {ph}  wheels {'READY' if rec['wheels'] else 'no'}  balls {s['shots']}/{s['shot_cap']}  hit {len(rec['popped'])}"],
                             None, status if bad else None))  # fmt: skip
        results.append((scn, meta))
    lines = [
        f"{scn:20s} {'PASS' if m['ok'] else 'FAIL'}  balls {m['rounds']:2d}  hits {m['hits_on_targets']}  violations {sum(m['violations'].values())}  hash {m['state_hash']}"
        for scn, m in results
    ]
    w.card("Result", lines + [
        "Each hash is the state hash at the end of the run; the replay behind the frames above ends on exactly that hash (checked: tools/record_cuas2.py verify). Honest limits: " + NOT_REAL + " Hit probability against a crossing quad is low in every configuration (docs/cuas2/sim_results.md).",
    ], 12.0, "RESULT", mono=len(lines))  # fmt: skip
    enc.close()
    print(
        f"wrote {a.out}: {w.n} frames = {w.n / FPS:.1f} s, {Path(a.out).stat().st_size / 1e6:.1f} MB, encoder {enc.backend}"
    )
    print(
        "timeline (output seconds):", ", ".join(f"{t:.0f}s {lab}" for t, lab in w.marks)
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("run")
    s.add_argument("scenario")
    s.add_argument("out")
    s.add_argument("--seed", type=int, default=1)
    for n in ("plan", "verify"):
        sub.add_parser(n).add_argument("run_dir")
    s = sub.add_parser("sim")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s.add_argument("--chunk", type=int, default=300, help="frames per viewer process")
    sub.add_parser("oplog").add_argument("out")
    s = sub.add_parser("encode")
    s.add_argument("root")
    s.add_argument("out")
    s.add_argument("--speedup", type=float, default=1.0)
    s.add_argument(
        "--gpl",
        action="store_true",
        help="ffmpeg VAAPI / libx264 allowed (GPL); default is GStreamer VA (LGPL)",
    )
    s = sub.add_parser("png")
    s.add_argument("frames_dir")
    s.add_argument("out_dir")
    s.add_argument("idx", nargs="+", type=int)
    s = sub.add_parser("web")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s.add_argument("--fps", type=int, default=10)
    s.add_argument("--speed", type=float, default=1.0)
    s.add_argument("--t0", type=float)
    s.add_argument("--t1", type=float)
    a = ap.parse_args()
    {
        "encode": cmd_encode,
        "png": L.cmd_png,
        "web": L.cmd_web,
        "run": cmd_run,
        "plan": L.cmd_plan,
        "sim": cmd_sim,
        "verify": cmd_verify,
        "oplog": cmd_oplog,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
