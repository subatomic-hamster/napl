"""Wires as 3D tubes (visual only). Each conductor of each assembly wire becomes a MuJoCo spatial tendon through two sites
(one on each end instance, at the port position), so the tube follows its bodies; the renderer draws tendons as tubes
(Solver::tubes) and sags the slack. The tendons have no stiffness, damping, limit or actuator: they add no force, the
dynamics are unchanged (tests/python/test_wire_tubes.py compares state hashes with and without)."""

import numpy as np

# Insulated (silicone hook-up wire) outer diameter by AWG, mm. Compiled from vendor listings of silicone hook-up wire,
# +-0.2 mm between manufacturers: 30 .. 16 AWG and 14 / 10 AWG from the Adafruit 26 AWG silicone page and the openelab /
# Progressive RC silicone wire listings (https://www.progressiverc.com/prc-silicone-wire-10-awg.html: 10 AWG ~5 mm);
# 12 and 8 AWG are interpolated / extrapolated from their neighbours (fidelity: estimated). Cosmetic only: no physics reads it.
AWG_OD_MM = {
    30: 1.1,
    28: 1.2,
    26: 1.3,
    24: 1.4,
    22: 1.6,
    20: 1.8,
    18: 2.0,
    16: 2.4,
    14: 3.5,
    12: 4.2,
    10: 5.0,
    8: 6.2,
}
MAX_TUBES = 8  # conductors drawn per wire (a ribbon of more is drawn as 8)

RED, BLACK = (0.82, 0.08, 0.08), (0.04, 0.04, 0.04)
PHASES = [
    (0.95, 0.78, 0.10),
    (0.10, 0.62, 0.25),
    (0.15, 0.40, 0.90),
]  # motor phases A B C: yellow, green, blue
SIGNAL = [
    (0.95, 0.78, 0.10),
    (0.92, 0.92, 0.92),
    (0.15, 0.40, 0.90),
    (0.10, 0.62, 0.25),
    (0.95, 0.45, 0.10),
    (0.55, 0.25, 0.75),
]


def od_m(awg, conductor_d=None) -> float:
    """Insulated outer diameter in metres: the AWG table (nearest gauge), else conductor diameter + 2 x 0.4 mm of wall."""
    if awg is not None:
        return AWG_OD_MM[min(AWG_OD_MM, key=lambda a: abs(a - awg))] * 1e-3
    return (conductor_d + 0.8e-3) if conductor_d else 1.3e-3


def colors(kind: str, n: int, port_name: str) -> list:
    """Function colours of the n conductors of a wire: power pair red / black, a 3-conductor power cable carries motor
    phases, a lone conductor is black on a ground-like port else red, signal wires cycle yellow / white / blue / ..."""
    n = min(n, MAX_TUBES)
    if kind == "power":
        if n == 1:
            return [
                BLACK
                if any(
                    s in port_name.lower() for s in ("gnd", "neg", "minus", "return")
                )
                else RED
            ]
        if n == 2:
            return [RED, BLACK]
        if n == 3:
            return PHASES
        return [RED, BLACK] + SIGNAL[: n - 2]
    return [SIGNAL[i % len(SIGNAL)] for i in range(n)]


def wire_tubes(wires: list, T: dict, sites: dict):
    """Adds the conductor sites to `sites` ({inst: {name: (pos, R)}}, instance-local) and returns the <tendon> lines."""
    lines = []
    for w in wires:
        ia, ib = w["_ia"], w["_ib"]
        if ia.id not in T or ib.id not in T:
            continue  # an offboard end has no pose: nothing to draw
        pa, pb = (
            np.array(p.get("pos", [0, 0, 0]), dtype=float) for p in (w["_pa"], w["_pb"])
        )
        wa, wb = (T[i.id][:3, :3] @ p + T[i.id][:3, 3] for i, p in ((ia, pa), (ib, pb)))
        awg = w["_part"].value("awg")
        d = od_m(awg, w["_part"].value("conductor_diameter"))
        n = min(w["conductors"], MAX_TUBES)
        chord = wb - wa
        ln = np.linalg.norm(chord)
        if ln < 1e-9 and ia.id == ib.id:
            continue
        ax = chord / ln if ln > 1e-9 else np.array([1.0, 0, 0])
        u = np.cross(ax, [0.0, 0.0, 1.0])
        u = (
            u / np.linalg.norm(u)
            if np.linalg.norm(u) > 1e-6
            else np.array([0.0, 1.0, 0.0])
        )
        v = np.cross(ax, u)
        ring = (
            d / (2 * np.sin(np.pi / n)) if n > 1 else 0.0
        )  # touching tubes: n = 2 -> +-d/2
        cols = colors(w["kind"], n, w["_pa"]["name"])
        for c in range(n):
            ang = 2 * np.pi * c / n + np.pi / 2
            off = ring * (np.cos(ang) * u + np.sin(ang) * v)
            names = []
            for inst, p, tag in ((ia, pa, "a"), (ib, pb, "b")):
                R = T[inst.id][:3, :3]
                s = f"wire:{w['index']}:{c}:{tag}"
                sites.setdefault(inst.id, {})[s] = (p + R.T @ off, np.eye(3))
                names.append(s)
            rgba = " ".join(f"{x:.3f}" for x in (*cols[c], 1.0))
            L = max(w["length"], ln)  # a wire shorter than its chord is taut
            lines.append(
                f'    <spatial name="wire:{w["index"]}:{c}" width="{d / 2:.6g}" rgba="{rgba}"'  # stiffness / damping default to 0: visual only
                f' springlength="{L:.6g} {L:.6g}">'
            )
            lines += [f'      <site site="{s}"/>' for s in names]
            lines.append("    </spatial>")
    return lines
