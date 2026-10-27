"""cuas2 launcher calibration (docs/cuas2/spec.md 4.3): the one file the sim, the fire control and the bench fitter share.

    c = calib.load()                       # ~/.config/cuas2/calib.toml if it exists, else the built-in [est] defaults
    c = calib.load("calib.toml")           # explicit file (a missing explicit path is an error, no silent defaults)
    c.launcher.k_slip, c.src["launcher.k_slip"]      # 0.8, "est"   (flips to "bench" when the file says k_slip_src = "bench")
    open(p, "w").write(calib.dumps(c))     # round-trips: loads(dumps(c)) == c
    python -m napl.cuas2.calib fit shots.csv [--calib in.toml] [--out out.toml]    # bench fitter (end of this file)

Sections: [ball], [launcher], [launcher.wheel], [launcher.feed]. Every key has a default and may carry `<key>_src = "est" | "bench"`.
Unknown keys, wrong types and out-of-range values raise ValueError; `v_cap_mps` > 38 is rejected (the firmware cap is absolute, I13).
"""

import argparse
import csv
import math
import tomllib
from dataclasses import dataclass, field, fields, replace
from pathlib import Path

V_CAP_MAX = 38.0  # m/s, hard cap (spec R1 / I13)
DEFAULT_PATH = Path.home() / ".config" / "cuas2" / "calib.toml"


@dataclass(frozen=True)
class Ball:
    d_mm: float = 22.2
    m_g: float = 1.86
    cd: float = 0.47
    cd_sigma: float = 0.05
    d_sigma_mm: float = 0.3
    m_sigma_g: float = 0.08


@dataclass(frozen=True)
class Wheel:
    J_kgm2: float = 4.0e-6  # wheel + rotor inertia
    kv: float = 1800.0
    pole_pairs: int = 7
    tau_motor_s: float = 0.04
    p_max_w: float = 100.0
    eta_transfer: float = 0.6
    rpm_sigma_pm: float = 3.0  # telemetry noise, permille
    v_bus: float = 12.0  # [est] not in the spec table, 3S under load; section 7.1 "V_bus 12 V (a parameter)"
    droop: float = 0.10  # [est] fractional loss of the target speed at full mechanical load (ball in the nip)
    a_brake_rads2: float = 20000.0  # [est] braking limit of the ESC (active freewheel)


@dataclass(frozen=True)
class Feed:
    t_push_ms: float = 80.0
    t_commit_ms: float = 45.0
    t_exit_ms: float = 8.0
    t_exit_sigma_ms: float = 2.0


@dataclass(frozen=True)
class Launcher:
    wheel_d_mm: float = 40.0
    gap_mm: float = 19.2
    k_slip: float = 0.80
    k_slip_pm_per_krpm: float = (
        0.0  # k_slip(rpm) = k_slip + slope * (mean rpm - rpm_max) / 1000
    )
    sigma_v_rel: float = 0.03
    sigma_v_floor_mps: float = 0.5
    spread_mrad: tuple = (10.0, 10.0)  # (horizontal, vertical) 1-sigma
    bias_mrad: tuple = (
        0.0,
        0.0,
    )  # mean direction offset (horizontal left+, vertical up+) relative to the barrel/boresight
    rpm_max: float = 17500.0
    rpm_idle: float = 4500.0
    diff_pm: float = 15.0
    k_spin: float = 0.80
    cl_k: float = 0.75
    spin_tau_s: float = 6.0
    v_cap_mps: float = 38.0
    k_slip_gap: tuple = ()  # optional bench table ((gap_mm, k), ...): overrides k_slip by linear interpolation at gap_mm


@dataclass(frozen=True)
class Calib:
    ball: Ball = field(default_factory=Ball)
    launcher: Launcher = field(default_factory=Launcher)
    wheel: Wheel = field(default_factory=Wheel)
    feed: Feed = field(default_factory=Feed)
    src: dict = field(
        default_factory=dict
    )  # "section.key" -> "est" | "bench" (absent = "est")

    def provenance(self):
        """-> ["section.key = value  [est|bench]", ...] for the app start-up banner."""
        out = []
        for sec, obj in _sections(self):
            for f in fields(obj):
                out.append(
                    f"{sec}.{f.name} = {getattr(obj, f.name)}  [{self.src.get(f'{sec}.{f.name}', 'est')}]"
                )
        return out

    @property
    def rpm_cap(self):
        """Firmware rpm cap from the velocity cap and the wheel diameter (I13): floor(v_cap * 60 / (pi * D))."""
        return rpm_cap(self.launcher.v_cap_mps, self.launcher.wheel_d_mm)

    def firmware_keys(self):
        """The two entries the MCU enforces on its own (CFG 23, 24): {23: wheel_d_um, 24: v_cap_dmps}."""
        return {
            23: round(self.launcher.wheel_d_mm * 1000),
            24: round(self.launcher.v_cap_mps * 10),
        }

    def k_slip_at(self, rpm_mean):
        lo = self.launcher
        if lo.k_slip_gap:
            g = sorted(lo.k_slip_gap)
            xs, ys = [a for a, _ in g], [b for _, b in g]
            if lo.gap_mm <= xs[0]:
                return ys[0]
            for i in range(1, len(xs)):
                if lo.gap_mm <= xs[i]:
                    f = (lo.gap_mm - xs[i - 1]) / (xs[i] - xs[i - 1])
                    return ys[i - 1] + f * (ys[i] - ys[i - 1])
            return ys[-1]
        return lo.k_slip + lo.k_slip_pm_per_krpm * (rpm_mean - lo.rpm_max) / 1000.0


def rpm_cap(v_cap_mps, wheel_d_mm):
    return math.floor(v_cap_mps * 60.0 / (math.pi * wheel_d_mm / 1000.0))


def _sections(c):
    return (
        ("ball", c.ball),
        ("launcher", c.launcher),
        ("launcher.wheel", c.wheel),
        ("launcher.feed", c.feed),
    )


_TABLES = {
    "ball": Ball,
    "launcher": Launcher,
    "launcher.wheel": Wheel,
    "launcher.feed": Feed,
}
_POSITIVE = {
    "ball": ("d_mm", "m_g"),
    "launcher": ("wheel_d_mm", "gap_mm", "k_slip", "rpm_max", "v_cap_mps"),
    "launcher.wheel": (
        "J_kgm2",
        "kv",
        "pole_pairs",
        "tau_motor_s",
        "p_max_w",
        "eta_transfer",
        "v_bus",
        "a_brake_rads2",
    ),
    "launcher.feed": ("t_push_ms", "t_commit_ms"),
}


def _num(sec, key, v, default):
    if isinstance(default, tuple):
        if not isinstance(v, (list, tuple)) or any(
            isinstance(x, bool) or not isinstance(x, (int, float)) for x in v
        ):
            raise ValueError(f"[{sec}] {key}: expected a list of numbers")
        v = tuple(float(x) for x in v)
        if key in ("spread_mrad", "bias_mrad") and len(v) != 2:
            raise ValueError(
                f"[{sec}] {key}: expected 2 numbers (horizontal, vertical)"
            )
        if key == "k_slip_gap":
            raise ValueError(f"[{sec}] {key}: expected a list of [gap_mm, k] pairs")
        return v
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f"[{sec}] {key}: expected a finite number, got {v!r}")
    return int(v) if isinstance(default, int) and float(v).is_integer() else float(v)


def _build(sec, cls, table, src):
    base, kw = cls(), {}
    names = {f.name for f in fields(cls)}
    for key, v in table.items():
        if key.endswith("_src") and key[:-4] in names:
            if v not in ("est", "bench"):
                raise ValueError(f'[{sec}] {key}: must be "est" or "bench", got {v!r}')
            if v == "bench":  # `src` holds only the bench keys; absent = "est"
                src[f"{sec}.{key[:-4]}"] = v
        elif key in names:
            if key == "k_slip_gap":
                if not isinstance(v, list) or any(
                    not isinstance(r, (list, tuple)) or len(r) != 2 for r in v
                ):
                    raise ValueError(f"[{sec}] k_slip_gap: expected [[gap_mm, k], ...]")
                kw[key] = tuple((float(a), float(b)) for a, b in v)
            else:
                kw[key] = _num(sec, key, v, getattr(base, key))
        else:
            raise ValueError(f"[{sec}] unknown key {key!r}")
    obj = replace(base, **kw)
    for key in _POSITIVE.get(sec, ()):
        if not getattr(obj, key) > 0:
            raise ValueError(f"[{sec}] {key} must be > 0")
    return obj


def loads(text):
    doc = tomllib.loads(text)
    extra = set(doc) - {"ball", "launcher"}
    if extra:
        raise ValueError(f"unknown table(s) {sorted(extra)}")
    lau = dict(doc.get("launcher", {}))
    sub = {"launcher.wheel": lau.pop("wheel", {}), "launcher.feed": lau.pop("feed", {})}
    for k, v in sub.items():
        if not isinstance(v, dict):
            raise ValueError(f"[{k}] must be a table")
    src, parts = {}, {}
    for sec, tab in (("ball", doc.get("ball", {})), ("launcher", lau), *sub.items()):
        parts[sec] = _build(sec, _TABLES[sec], tab, src)
    c = Calib(
        ball=parts["ball"],
        launcher=parts["launcher"],
        wheel=parts["launcher.wheel"],
        feed=parts["launcher.feed"],
        src=src,
    )
    lo = c.launcher
    if lo.v_cap_mps > V_CAP_MAX:
        raise ValueError(
            f"[launcher] v_cap_mps {lo.v_cap_mps} > {V_CAP_MAX} (the firmware cap is absolute)"
        )
    if not 0 < lo.k_slip <= 1 or not 0 < c.wheel.eta_transfer <= 1:
        raise ValueError("[launcher] k_slip and eta_transfer must be in (0, 1]")
    if (
        lo.sigma_v_rel < 0
        or lo.sigma_v_floor_mps < 0
        or any(s < 0 for s in lo.spread_mrad)
    ):
        raise ValueError("[launcher] scatter sigmas must be >= 0")
    return c


def load(path=None):
    """None -> DEFAULT_PATH if it exists, else the defaults. An explicit path must exist."""
    if path is None:
        return loads(DEFAULT_PATH.read_text()) if DEFAULT_PATH.exists() else Calib()
    return loads(Path(path).read_text())


def _fmt(v):
    if isinstance(v, tuple):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    if isinstance(v, int):
        return str(v)
    return repr(float(v))


def dumps(c):
    """TOML text of every key with its `_src` provenance (round-trips through loads)."""
    out = []
    for sec, obj in _sections(c):
        out.append(f"[{sec}]")
        for f in fields(obj):
            out.append(f"{f.name} = {_fmt(getattr(obj, f.name))}")
            out.append(f'{f.name}_src = "{c.src.get(f"{sec}.{f.name}", "est")}"')
        out.append("")
    return "\n".join(out)


# ---- bench fitter (spec 10: B2 k_slip, sigma_v, B3 spread / bias, B4 cl_k, B5 cd) --------------------------------------------------------
# The log is a CSV, one row per shot, blank = not measured. Columns (docs/cuas2/bench_validation.md):
#   step     B2 | B3 | B4 | B5 (informational; the fitter keys on which columns are filled)
#   gap_mm   wheel-surface gap used
#   diff_pm  commanded differential (bottom faster = positive), permille
#   rpm_top, rpm_bot    wheel rpm at the feed (STATUS rpm_top / rpm_bot, the mean over the window), rpm
#   v_mps    external chronograph speed at the muzzle, m/s
#   range_m  distance muzzle -> target sheet (sheet perpendicular to the level barrel axis), m
#   dx_mm, dy_mm        impact point on the sheet relative to the barrel-axis point (x left+, y up+), mm
COLUMNS = (
    "step",
    "gap_mm",
    "diff_pm",
    "rpm_top",
    "rpm_bot",
    "v_mps",
    "range_m",
    "dx_mm",
    "dy_mm",
)


def read_log(path):
    """-> {column: list of float (nan if blank)} (step stays a list of str)."""
    rows = list(csv.DictReader(open(path, newline="")))
    out = {}
    for k in COLUMNS:
        if k == "step":
            out[k] = [r.get(k, "") or "" for r in rows]
        else:
            out[k] = [
                float(r[k]) if r.get(k) not in (None, "") else math.nan for r in rows
            ]
    return out


def write_log(path, log):
    n = len(log["rpm_top"])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for i in range(n):
            w.writerow(
                [
                    ""
                    if (isinstance(log[k][i], float) and math.isnan(log[k][i]))
                    else log[k][i]
                    for k in COLUMNS
                ]
            )


def _np():
    import numpy as np

    return np


def fit_speed(log, c):
    """B2. From rows with v_mps and rpm_top/rpm_bot: k_slip (at rpm_max, plus the rpm slope when >= 3 distinct speed levels), sigma_v_rel,
    sigma_v_floor_mps. -> dict of the fitted launcher keys. k = v / (v_mean (1 - dip/2)) per shot (the model's own dip factor is divided out)."""
    from .launcher import LauncherModel

    np = _np()
    rt, rb, v = (np.array(log[k], float) for k in ("rpm_top", "rpm_bot", "v_mps"))
    ok = np.isfinite(rt) & np.isfinite(rb) & np.isfinite(v)
    if ok.sum() < 5:
        raise ValueError("fit_speed: need >= 5 rows with rpm_top, rpm_bot, v_mps")
    m = LauncherModel(c)
    rt, rb, v = rt[ok], rb[ok], v[ok]
    vn = np.array(
        [m.nominal_v(a, b) for a, b in zip(rt, rb)]
    )  # v_mean * (1 - dip/2): k_slip = 1 speed
    k = v / vn
    rm = (rt + rb) / 2
    levels = np.unique(np.round(rm / 250.0))  # distinct set points (250 rpm bins)
    slope = 0.0
    if len(levels) >= 3:
        slope, k0 = np.polyfit(
            (rm - c.launcher.rpm_max) / 1000.0, k, 1
        )  # k = k0 + slope x
    else:
        k0 = float(k.mean())
    res = v - (k0 + slope * (rm - c.launcher.rpm_max) / 1000.0) * vn
    # variance model per set point: var = (sigma_rel v)^2 + floor^2; least squares over the level means, both >= 0
    lv = np.round(rm / 250.0)
    xs, ys = [], []
    for lvl in np.unique(lv):
        s = lv == lvl
        if s.sum() >= 3:
            xs.append(float(np.mean((k0 * vn[s]) ** 2)))
            ys.append(float(np.var(res[s], ddof=1)))
    if len(xs) >= 2:
        A = np.array([[x, 1.0] for x in xs])
        a, b = np.linalg.lstsq(A, np.array(ys), rcond=None)[0]
        if a < 0 or b < 0:  # a negative variance component: refit the other alone
            if a < 0:
                a, b = 0.0, float(np.mean(ys))
            else:
                a, b = (
                    float(
                        np.sum(np.array(xs) * np.array(ys)) / np.sum(np.array(xs) ** 2)
                    ),
                    0.0,
                )
        rel, floor = math.sqrt(max(a, 0.0)), math.sqrt(max(b, 0.0))
    else:  # one set point: the split is not identifiable. Keep the floor, put the rest into sigma_rel.
        var = float(np.var(res, ddof=1))
        fl = c.launcher.sigma_v_floor_mps
        rel, floor = math.sqrt(max(var - fl**2, 0.0)) / float(np.mean(k0 * vn)), fl
    out = dict(k_slip=float(k0), sigma_v_rel=rel, sigma_v_floor_mps=floor)
    if slope:
        out["k_slip_pm_per_krpm"] = float(slope)
    return out


def _row_v(log, i, c, m):
    """Muzzle speed of row i: the chronograph reading if present, else the model's mean shot at the logged wheel speeds."""
    v = log["v_mps"][i]
    if math.isfinite(v):
        return v
    rt, rb = log["rpm_top"][i], log["rpm_bot"][i]
    return m.nominal_v(rt, rb) * c.k_slip_at((rt + rb) / 2)


def _drops(c, range_m, v, omega_y=0.0, cd=None, cl_k=None):
    """Model impact height (m, relative to the level barrel axis) at range_m for muzzle speeds v (array), interpolated (quadratic) on a 7-point
    speed grid so a log with one chronograph reading per shot costs 7 flights per range, not one per shot."""
    from . import ballistics

    np = _np()
    v = np.asarray(v, float)
    lo, hi = float(v.min()) - 0.5, float(v.max()) + 0.5
    grid = np.linspace(lo, hi, 7)
    z = np.array(
        [
            ballistics.impact_height(c, g, omega_y, range_m, cd=cd, cl_k=cl_k, dt=0.004)
            for g in grid
        ]
    )
    return np.polyval(np.polyfit(grid, z, 2), v)


def _golden(f, lo, hi, tol=1e-4):
    g = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    c_, d_ = b - g * (b - a), a + g * (b - a)
    fc, fd = f(c_), f(d_)
    while b - a > tol:
        if fc < fd:
            b, d_, fd = d_, c_, fc
            c_ = b - g * (b - a)
            fc = f(c_)
        else:
            a, c_, fc = c_, d_, fd
            d_ = a + g * (b - a)
            fd = f(d_)
    return (a + b) / 2


def fit_aim(log, c):
    """B3 + B5 (the same shots: a hit grid at >= 2 ranges, diff_pm 0). Rows need range_m, dx_mm, dy_mm and v_mps (or rpm_top / rpm_bot).
    dy(R) = drop(R; cd, v) + R b_v, so with >= 2 distinct ranges cd and the vertical bias b_v separate (b_v is profiled out in closed form for each
    cd; golden search on cd). With one range cd is kept. Spread = std of the residual angles. -> {"cd", "bias_mrad", "spread_mrad"} (cd is in
    the [ball] section; fit_all routes it)."""
    from . import ballistics
    from .launcher import LauncherModel

    np = _np()
    f = lambda k: np.array(log[k], float)
    d0 = np.nan_to_num(np.abs(f("diff_pm")), nan=0.0) < 1e-9
    idx = np.nonzero(
        d0
        & np.isfinite(f("range_m"))
        & np.isfinite(f("dx_mm"))
        & np.isfinite(f("dy_mm"))
        & (np.isfinite(f("v_mps")) | np.isfinite(f("rpm_top")))
    )[0]
    if len(idx) < 5:
        raise ValueError(
            "fit_aim: need >= 5 rows at diff_pm 0 with range_m, dx_mm, dy_mm and v_mps"
        )
    m = LauncherModel(c)
    R = f("range_m")[idx]
    v = np.array([_row_v(log, i, c, m) for i in idx])
    dy, dx = f("dy_mm")[idx] / 1000.0, f("dx_mm")[idx] / 1000.0
    ranges = np.unique(np.round(R, 2))

    def drops(cd):
        z = np.empty(len(idx))
        for r in ranges:
            s = np.round(R, 2) == r
            z[s] = _drops(c, r, v[s], cd=cd)
        return z

    def profile(cd):
        z = drops(cd)
        b = float(np.sum(R * (dy - z)) / np.sum(R * R))  # rad
        return b, float(np.sum((dy - z - R * b) ** 2)), z

    cd = c.ball.cd
    if len(ranges) >= 2:
        cd = _golden(lambda x: profile(x)[1], 0.15, 1.2, tol=2e-3)
    b, _, z = profile(cd)
    res_v = (dy - z - R * b) / R * 1000.0
    ang_h = dx / R * 1000.0
    # The vertical scatter also holds what the model already adds itself: ball diameter / mass / cd scatter. Take it out in quadrature (the sim
    # draws it separately), estimated by a small Monte Carlo of the calibrated ball spread at each range's mean speed.
    rng, ball_var = np.random.default_rng(0), np.zeros(len(idx))
    for r in ranges:
        s = np.round(R, 2) == r
        zs = []
        for _ in range(16):
            d = max(rng.normal(c.ball.d_mm, c.ball.d_sigma_mm), 1.0) / 1000
            mm = max(rng.normal(c.ball.m_g, c.ball.m_sigma_g), 0.1) / 1000
            cdd = max(rng.normal(cd, c.ball.cd_sigma), 0.05)
            zs.append(
                ballistics.impact_height(
                    c, float(v[s].mean()), 0.0, r, ball=(d, mm, cdd), dt=0.004
                )
            )
        ball_var[s] = (np.std(zs, ddof=1) / r * 1000.0) ** 2
    spread_v = math.sqrt(max(float(res_v.var(ddof=1)) - float(ball_var.mean()), 0.0))
    return dict(
        cd=float(cd),
        bias_mrad=(float(ang_h.mean()), b * 1000.0),
        spread_mrad=(float(ang_h.std(ddof=1)), spread_v),
    )


def fit_cl(log, c):
    """B4. Rows with range_m, dy_mm, diff_pm != 0 or 0: cl_k (holding k_spin at its calibrated value: the heights identify only the product
    cl_k * k_spin). -> {"cl_k": ..., "best_diff_pm": the measured diff with the smallest vertical scatter}."""
    from .launcher import LauncherModel

    np = _np()
    f = lambda k: np.array(log[k], float)
    idx = np.nonzero(
        np.isfinite(f("range_m"))
        & np.isfinite(f("dy_mm"))
        & np.isfinite(f("rpm_top"))
        & np.isfinite(f("rpm_bot"))
        & np.isfinite(f("diff_pm"))
    )[0]
    if len({log["diff_pm"][i] for i in idx}) < 2:
        raise ValueError(
            "fit_cl: need rows at >= 2 diff_pm values with range_m, dy_mm, rpm_top, rpm_bot"
        )
    m = LauncherModel(c)
    groups = {}
    for i in idx:
        groups.setdefault((round(log["range_m"][i], 2), log["diff_pm"][i]), []).append(
            i
        )
    # Lift is linear in cl_k while C_L = cl_k S stays below the 0.25 saturation (S ~ 0.02), so z(cl_k) = z0 + cl_k z1 per shot and cl_k is a closed-form
    # least squares over every shot, each at its OWN chronograph speed (the speed scatter would otherwise bury the lift).
    # One free height offset per range absorbs the aim bias and its correlation with the drag fit, so cl_k comes only from how the height changes with
    # the wheel difference: y = z0 + cl_k z1 + a_range.
    ranges = sorted({r for r, _ in groups})
    ys, z1s, rows = [], [], []
    for (r, _), ii in groups.items():
        om = float(
            np.mean([m.spin_rad_s(log["rpm_top"][i], log["rpm_bot"][i]) for i in ii])
        )
        v = np.array([_row_v(log, i, c, m) for i in ii])
        z0 = _drops(c, r, v, om, cl_k=0.0)
        ys.append(np.array([log["dy_mm"][i] / 1000 for i in ii]) - z0)
        z1s.append(_drops(c, r, v, om, cl_k=1.0) - z0)
        rows += [ranges.index(r)] * len(ii)
    y, z1, rows = np.concatenate(ys), np.concatenate(z1s), np.array(rows)
    X = np.column_stack([z1] + [(rows == k).astype(float) for k in range(len(ranges))])
    if np.ptp(z1) < 1e-6:
        raise ValueError("fit_cl: no spin difference in the log")
    cl_fit = float(np.linalg.lstsq(X, y, rcond=None)[0][0])

    spread = {}
    for i in idx:
        spread.setdefault(log["diff_pm"][i], []).append(log["dy_mm"][i])
    best = min(
        (d for d in spread if len(spread[d]) >= 3),
        key=lambda d: np.std(spread[d]),
        default=None,
    )
    out = {"cl_k": max(cl_fit, 0.0)}
    if best is not None:
        out["best_diff_pm"] = best
    return out


def fit_all(log, c):
    """Run every fit the log has the columns for, in the order that keeps them independent (speed -> aim -> spin); each result is applied to the
    calibration before the next. -> (new Calib with `_src` = bench on the fitted keys, report dict)."""
    rep, src = {}, dict(c.src)
    for fn in (fit_speed, fit_aim, fit_cl):
        try:
            r = fn(log, c)
        except ValueError as e:
            rep[fn.__name__] = f"skipped: {e}"
            continue
        rep[fn.__name__] = r
        for k, v in r.items():
            if k == "best_diff_pm":
                continue
            sec = "ball" if k == "cd" else "launcher"
            c = replace(c, **{sec: replace(getattr(c, sec), **{k: v})})
            src[f"{sec}.{k}"] = "bench"
        c = replace(c, src=src)
    return c, rep


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="python -m napl.cuas2.calib", description=__doc__.split("\n")[0]
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser(
        "fit", help="fit k_slip, sigma_v, spread/bias, cd, cl_k from a bench CSV log"
    )
    f.add_argument("log")
    f.add_argument(
        "--calib", help="starting calib.toml (default: the built-in defaults)"
    )
    f.add_argument(
        "--out", help="write the updated calib.toml here (default: print only)"
    )
    f.add_argument(
        "--gap",
        type=float,
        help="use only the rows with this gap_mm (one fit per gap), and set launcher.gap_mm to it",
    )
    sub.add_parser("show", help="print the provenance of the active calibration")
    a = ap.parse_args(argv)
    if a.cmd == "show":
        print("\n".join(load().provenance()))  # ttsr-ignore: py/no-print (CLI)
        return 0
    c0 = load(a.calib) if a.calib else Calib()
    log = read_log(a.log)
    if a.gap is not None:
        keep = [i for i, g in enumerate(log["gap_mm"]) if abs(g - a.gap) < 1e-6]
        log = {k: [v[i] for i in keep] for k, v in log.items()}
        c0 = replace(c0, launcher=replace(c0.launcher, gap_mm=a.gap))
    c, rep = fit_all(log, c0)
    for k, v in rep.items():
        print(f"{k}: {v}")  # ttsr-ignore: py/no-print (CLI)
    if a.out:
        Path(a.out).write_text(dumps(c))
        print(f"wrote {a.out}")  # ttsr-ignore: py/no-print (CLI)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
