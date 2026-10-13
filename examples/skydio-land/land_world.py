"""Ground truth of a landing scene: the take-off site, the landing zones (the mission's designated zone, its alternates, home), the obstacles and, on the mountain
families, the terrain. The scene builder (land_scene.py) and the evaluation (trace.csv, monitors, Monte Carlo) read it; the autonomy never does: it gets only
`mission()` (where the zones are, to ~0 m: a surveyed pad list as a mission file) and has to find out from its own sensors whether a zone is safe.

    w = forest_world(seed)            # the Skydio forest (examples/skydio/forest.py) with two carved clearings; the designated one sometimes holds a hazard
    w.zones[0]                        # Zone(name, x, y, z, radius, safe, why): `safe` is the TRUTH (a hazard inside, a slope, a drop-off ...)
    w.clearance(p)                    # distance from p to the nearest obstacle (and terrain) surface, exact for obstacles
    w.mission()                       # [{name, x, y, radius}]: what the vehicle is told
"""

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "skydio"))
import forest as F  # examples/skydio/forest.py (the original, imported)

R_CARVE = 6.0  # m: trees, poles and boulders of the generated forest are removed this far (surface to centre) from a clearing's centre
HAZARD_P = 0.4  # probability that the designated clearing holds a hazard (boulders or a log): the alternate-zone logic is exercised in about this share of seeds
PAD_R = 2.5  # m: marked radius of a forest landing pad
FOOT_R, RING_R = (
    1.5,
    4.0,
)  # m: footprint disc and clear ring the vehicle needs (land.py Params r_foot / r_clear), also the truth test's


@dataclass
class Zone:
    name: str
    x: float
    y: float
    z: float  # ground height at the centre (world z)
    radius: float  # marked radius
    safe: bool = True  # TRUTH: nothing inside the footprint / clear ring that a lander must avoid
    why: str = ""  # what makes it unsafe (evaluation / report only)


class LandingWorld:
    """Obstacles (a `forest.Forest`, so `clearance` and `to_json` are the original's), zones, optional terrain."""

    def __init__(self, family, seed, forest, zones, start, terrain=None, notes=None):
        self.family, self.seed, self.forest, self.zones = family, seed, forest, zones
        self.start = tuple(start)  # (x, y, ground z) of the take-off pad
        self.terrain = terrain  # napl.terrain.Terrain | None
        self.notes = notes or {}

    # ---- queries (evaluation) ---------------------------------------------------------------------------------------------------------
    def ground_z(self, x, y):
        return 0.0 if self.terrain is None else float(self.terrain.height_at(x, y))

    def obstacle_clearance(self, p, cutoff=6.0):
        if not self.forest.obstacles:
            return cutoff
        return self.forest.clearance(p, cutoff)

    def terrain_clearance(self, p):
        """Distance to the terrain surface: the height above it over the tangent plane's cosine (exact on flat ground and on a plane, a lower bound on a convex slope)."""
        if self.terrain is None:
            return float(p[2])
        t = self.terrain
        s = float(t.slope_at(p[0], p[1]))
        d = (p[2] - float(t.height_at(p[0], p[1]))) / math.sqrt(1.0 + s * s)
        for r in t.rocks:
            d = min(d, math.dist(p, (r.x, r.y, r.z)) - max(r.rx, r.ry, r.rz))
        return float(d)

    def clearance(self, p, cutoff=6.0):
        """Distance to the nearest obstacle surface (the original forest's `clearance` interface: used by Skydio._telemetry); the terrain is `terrain_clearance`."""
        return self.obstacle_clearance(p, cutoff)

    def spot_safe(self, x, y):
        """TRUTH: is the ground point (x, y) a safe place to put the gear down? Mountain families: the footprint / ring test of the detector's thresholds on the exact
        surface (pad_truth); forest: no tree, pole, boulder or log within `FOOT_R` (1.5 m) of the point."""
        if self.terrain is not None:
            return bool(pad_truth(self.terrain, x, y)[0])
        return bool(self.obstacle_clearance((x, y, 0.3), FOOT_R + 0.5) >= FOOT_R)

    def nearest_safe_zone(self, xy):
        """(zone, horizontal distance) of the truth-safe zone nearest to xy, or (None, inf)."""
        best, bd = None, math.inf
        for z in self.zones:
            d = math.hypot(xy[0] - z.x, xy[1] - z.y)
            if z.safe and d < bd:
                best, bd = z, d
        return best, bd

    def mission(self):
        return [dict(name=z.name, x=z.x, y=z.y, radius=z.radius) for z in self.zones]

    # ---- persistence ------------------------------------------------------------------------------------------------------------------
    def to_json(self):
        return dict(family=self.family, seed=self.seed, start=list(self.start), notes=self.notes, forest=self.forest.to_json(),
                    zones=[z.__dict__ for z in self.zones], terrain=None if self.terrain is None else self.terrain.params | dict(kind=self.terrain.kind))  # fmt: skip

    def save(self, path):
        Path(path).write_text(json.dumps(self.to_json()))


def load_forest_obstacles(j):
    f = F.Forest(
        j["seed"],
        j["mission"],
        start=tuple(j["start"]),
        goal=tuple(j["goal"]),
        notes=j["notes"],
    )
    f.obstacles = [
        F.Obstacle(
            o["kind"],
            o["name"],
            tuple(o["pos"]),
            tuple(o["size"]),
            o["yaw"],
            tuple(o["rgba"]),
        )
        for o in j["obstacles"]
    ]
    return f


def load_world(j, terrain=None):
    """The inverse of LandingWorld.to_json; the mountain families regenerate their terrain from notes["terrain_args"] (seeded: the same heights)."""
    if terrain is None and j["notes"].get("terrain_args"):
        terrain = build_terrain(
            j["notes"]["terrain_args"], j["notes"].get("extra_rocks", [])
        )
    return LandingWorld(
        j["family"],
        j["seed"],
        load_forest_obstacles(j["forest"]),
        [Zone(**z) for z in j["zones"]],
        j["start"],
        terrain,
        j["notes"],
    )


def _extent(o):
    """Horizontal reach of an obstacle from its centre (m)."""
    if o.kind == "cyl":
        return o.size[0]
    if o.kind == "sphere":
        return o.size[0]
    return math.hypot(o.size[0], o.size[1])


def carve(forest, cx, cy, radius=R_CARVE):
    """Remove every obstacle whose surface is within `radius` of (cx, cy) horizontally (a clearing)."""
    forest.obstacles = [
        o
        for o in forest.obstacles
        if math.hypot(o.pos[0] - cx, o.pos[1] - cy) - _extent(o) >= radius
    ]


def add_hazard(forest, cx, cy, rng, kind=None):
    """Hazards the lidar must find: three boulders (tops 0.5-1.0 m) or a 0.5 m high log across the pad. -> description."""
    kind = kind or ("boulders" if rng.uniform() < 0.5 else "log")
    if kind == "boulders":
        for i in range(3):
            a, d, r = (
                float(rng.uniform(0, 2 * math.pi)),
                float(rng.uniform(0.7, 2.0)),
                float(rng.uniform(0.5, 0.8)),
            )
            forest.obstacles.append(
                F.Obstacle(
                    "sphere",
                    f"hazard_boulder{i}",
                    (cx + d * math.cos(a), cy + d * math.sin(a), 0.2),
                    (r,),
                    0.0,
                    F.ROCK,
                )
            )
        return "three boulders in the pad"
    yaw = float(rng.uniform(0, math.pi))
    forest.obstacles.append(
        F.Obstacle(
            "box",
            "hazard_log",
            (
                cx + float(rng.uniform(-0.6, 0.6)),
                cy + float(rng.uniform(-0.6, 0.6)),
                0.25,
            ),
            (1.8, 0.22, 0.25),
            yaw,
            F.BARK,
        )
    )
    return "a log across the pad"


def forest_world(seed, hazard=None):
    """Forest family: take off at the origin, designated clearing near the goal of `forest.generate(seed, "goal")` (wall doorway, arch, trees), an alternate clearing
    11-15 m from it, home = the take-off pad. `hazard`: None = drawn from the seed (HAZARD_P), True / False to force."""
    f = F.generate(seed, "goal")
    rng = np.random.default_rng(seed + 5000)
    px, py = f.goal
    solid = [
        (f.notes["wall_x"], f.notes["door_y"], 3.5),
        (f.notes["arch_x"], f.notes["arch_y"], 3.0),
    ]
    for _ in range(200):
        ang, r = (
            float(rng.uniform(math.radians(110), math.radians(250))),
            float(rng.uniform(11.0, 15.0)),
        )
        ax, ay = px + r * math.cos(ang), py + r * math.sin(ang)
        if (
            40.0 <= ax <= 60.0
            and abs(ay) <= 11.0
            and all(math.hypot(ax - sx, ay - sy) > sr + R_CARVE for sx, sy, sr in solid)
        ):
            break
    else:
        raise RuntimeError(f"no room for an alternate clearing, seed {seed}")
    for cx, cy in (
        (px, py),
        (ax, ay),
        (0.0, 0.0),
    ):  # designated, alternate, home (the take-off pad)
        carve(f, cx, cy)
    bad = (rng.uniform() < HAZARD_P) if hazard is None else bool(hazard)
    why = add_hazard(f, px, py, rng) if bad else ""
    zones = [Zone("designated", px, py, 0.0, PAD_R, not bad, why), Zone("alternate", ax, ay, 0.0, PAD_R, True),
             Zone("home", 0.0, 0.0, 0.0, 2.0, True)]  # fmt: skip
    f.goal = (px, py)
    f.notes.update(hazard=why, alternate=[ax, ay])
    return LandingWorld("forest", seed, f, zones, (0.0, 0.0, 0.0))


# ---- mountain families: summit, ledge, mesa_brink ------------------------------------------------------------------------------------------
# Terrain-centred frame (napl.terrain); the datum (z = 0) is the take-off pad. Sizes are 520 x 380 m on a 401-sample grid (1.3 m cells): the pads are 3.5 m
# in radius (5.5 m flat with the 1.5-cell margin) and their blend ring is short (6-12 m), so the cliffs and slopes next to a pad stay within the sensor's reach.
TERRAIN_FAMILIES = {
    "summit": dict(kind="ridge", size=(520.0, 380.0), n=401, relief=40.0, roughness=0.8, hurst=0.85, shape=dict(width=90.0), rocks=25, origin_alt=3200.0),
    "ledge": dict(kind="ledge", size=(520.0, 380.0), n=401, relief=30.0, roughness=0.8, hurst=0.85, shape=dict(width=70.0, skew=0.35), rocks=25, origin_alt=3000.0),
    "mesa_brink": dict(kind="mesa", size=(520.0, 380.0), n=401, relief=45.0, roughness=0.6, hurst=0.85, shape=dict(cliff_width=10.0), rocks=20, origin_alt=2600.0),
}  # fmt: skip
PAD_R_T = 3.5  # m: marked radius of a mountain landing pad


def build_terrain(args, extra_rocks=()):
    """napl.terrain.generate from the JSON-able `args` (kind, size, n, relief, roughness, hurst, seed, shape, pads [{name, at, radius, blend}], rocks, zero_at) plus
    the hazard boulders `extra_rocks` [{x, y, z, rx, ry, rz, yaw}]."""
    import napl.terrain as T

    a = dict(args)
    pads = [T.PadSpec(**p) for p in a.pop("pads")]
    t = T.generate(pads=pads, **a.pop("shape", {}), **a)
    t.rocks.extend(T.Rock(**r) for r in extra_rocks)
    return t


def _rock_top(t, X, Y):
    """Height of the highest boulder surface over (X, Y), or -inf: ellipsoids (rx, ry, rz) at (x, y, z), yawed."""
    top = np.full(np.shape(X), -np.inf)
    for r in t.rocks:
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        u, v = (X - r.x) * c + (Y - r.y) * s, -(X - r.x) * s + (Y - r.y) * c
        q = (u / r.rx) ** 2 + (v / r.ry) ** 2
        top = np.where(
            q < 1.0,
            np.maximum(top, r.z + r.rz * np.sqrt(np.clip(1.0 - q, 0.0, 1.0))),
            top,
        )
    return top


def pad_truth(t, x, y, lp=None):
    """TRUTH test of a landing spot with the numbers of land.LandParams (the detector's thresholds, on the exact surface instead of lidar points): footprint disc
    r_foot -> plane, slope and roughness; ring up to r_rise -> nothing taller than step_max above the plane; ring up to r_clear -> nothing deeper than drop_max
    below it. -> (ok, reason, slope_deg)."""
    import land as L

    lp = lp or L.LandParams()
    ang = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
    rs = np.arange(0.5, lp.r_clear + 1e-9, 0.5)
    R = np.concatenate([[0.0], np.repeat(rs, len(ang))])
    A = np.concatenate([[0.0], np.tile(ang, len(rs))])
    X, Y = x + R * np.cos(A), y + R * np.sin(A)
    Z = np.maximum(t.height_at(X, Y), _rock_top(t, X, Y))
    foot = R <= lp.r_foot
    M = np.column_stack([X - x, Y - y, np.ones_like(X)])
    coef, *_ = np.linalg.lstsq(M[foot], Z[foot], rcond=None)
    dev = Z - M @ coef
    slope = math.degrees(math.atan(math.hypot(coef[0], coef[1])))
    if slope > lp.slope_max_deg:
        return False, "slope", slope
    if (
        float(dev[foot].std()) > lp.rms_max
        or float(np.abs(dev[foot]).max()) > lp.step_max
    ):
        return False, "rough", slope
    if float(dev[(R > lp.r_foot) & (R <= lp.r_rise)].max()) > lp.step_max:
        return False, "obstacle", slope
    if float(dev[R > lp.r_foot].min()) < -lp.drop_max:
        return False, "drop-off", slope
    return True, "", slope


def _hazard_rocks(t, pad, rng):
    """Three boulders in the pad (0.4-0.6 m high above the surface): -> [{x, y, z, rx, ry, rz, yaw}]."""
    out = []
    for _ in range(3):
        a, d = float(rng.uniform(0, 2 * math.pi)), float(rng.uniform(0.7, 2.0))
        rz = float(rng.uniform(0.3, 0.45))
        x, y = pad.x + d * math.cos(a), pad.y + d * math.sin(a)
        out.append(
            dict(
                x=x,
                y=y,
                z=pad.z + 0.2 * rz,
                rx=float(rng.uniform(0.5, 0.8)),
                ry=float(rng.uniform(0.5, 0.8)),
                rz=rz,
                yaw=float(rng.uniform(0, math.pi)),
            )
        )
    return out


def terrain_world(family, seed, hazard=None, haz_p=0.35):
    """summit / ledge / mesa_brink worlds. Zones: designated, alternate, home (= the take-off pad, datum z = 0).
    summit      ridge, take-off pad on the crest ~100 m from the summit pad (designated); the alternate is a saddle pad 45 m along the crest. `hazard`: boulders on the summit pad.
    ledge       the designated pad is a ledge cut into the lee face below the crest, the alternate another ledge 30 m along the contour; `hazard` as above.
    mesa_brink  the designated pad lies near the cliff edge: with probability 0.6 (or hazard=True) it is within the drop-off's reach (truth unsafe), else a safe margin from
                it; the alternate is inland. The cliff line's lateral position, wobble and the boulders are seeded."""
    import napl.terrain as T

    spec = TERRAIN_FAMILIES[family]
    rng = np.random.default_rng(seed + 7000)
    base_args = dict(kind=spec["kind"], size=list(spec["size"]), n=spec["n"], relief=spec["relief"], roughness=spec["roughness"], hurst=spec["hurst"], seed=seed,
                     shape=dict(spec["shape"]), rocks=spec["rocks"])  # fmt: skip
    bad = (
        (rng.uniform() < (0.6 if family == "mesa_brink" else haz_p))
        if hazard is None
        else bool(hazard)
    )
    t0 = T.generate(
        **{
            **{k: v for k, v in base_args.items() if k != "shape"},
            **base_args["shape"],
            "rocks": 0,
        }
    )
    an = t0.anchors
    if family == "summit":
        sx, sy = an["summit"][:2]
        locs = dict(
            designated=(sx, sy), alternate=(sx + 3.0, sy + 45.0), home=(0.0, sy - 100.0)
        )
        blend = dict(designated=14.0, alternate=12.0, home=10.0)
    elif family == "ledge":
        lx, ly = an["ledge"][:2]
        locs = dict(
            designated=(lx, ly), alternate=(lx, ly + 32.0), home=(0.0, ly - 60.0)
        )
        blend = dict(designated=8.0, alternate=8.0, home=10.0)
    else:  # mesa_brink: scan toward the cliff along a seeded line for the first truth-unsafe spot
        y0 = float(rng.uniform(-40.0, 40.0))
        xe = t0.size[0] * 0.12  # the shape's default edge line
        x_fail = None
        for x in np.arange(xe - 70.0, xe + 5.0, 1.0):
            ok, _, _ = pad_truth(t0, float(x), y0)
            if not ok:
                x_fail = float(x)
                break
        if x_fail is None:
            raise RuntimeError(
                f"mesa_brink: no unsafe spot toward the cliff, seed {seed}"
            )
        # a pad is a cut: flat out to 5.5 m, then blended into the surface. Scan pad positions toward the cliff for the first whose PADDED truth fails.
        o_f = 14.0
        for o in np.arange(0.0, 34.0, 2.0):
            tp = build_terrain(
                dict(
                    base_args,
                    rocks=0,
                    pads=[
                        dict(
                            name="d",
                            at=[x_fail + float(o), y0],
                            radius=PAD_R_T,
                            blend=7.0,
                        )
                    ],
                )
            )
            if not pad_truth(tp, tp.pad("d").x, tp.pad("d").y)[0]:
                o_f = float(o)
                break
        xd = x_fail + o_f if bad else x_fail + o_f - 12.0
        locs = dict(
            designated=(xd, y0),
            alternate=(x_fail - 40.0, y0 + float(rng.uniform(-10.0, 10.0))),
            home=(x_fail - 100.0, y0),
        )
        blend = dict(designated=7.0, alternate=8.0, home=10.0)
    pads = [dict(name="home", at=list(locs["home"]), radius=PAD_R_T, blend=blend["home"]),
            dict(name="designated", at=list(locs["designated"]), radius=PAD_R_T, blend=blend["designated"]),
            dict(name="alternate", at=list(locs["alternate"]), radius=PAD_R_T, blend=blend["alternate"])]  # fmt: skip
    args = dict(base_args, pads=pads, zero_at="home")
    t = build_terrain(args)
    extra = (
        _hazard_rocks(t, t.pad("designated"), rng)
        if (bad and family != "mesa_brink")
        else []
    )
    t.rocks.extend(T.Rock(**r) for r in extra)
    zones = []
    why_haz = {
        "summit": "three boulders on the summit pad",
        "ledge": "three boulders on the ledge",
        "mesa_brink": "the pad is within reach of the drop-off",
    }[family]
    for name in ("designated", "alternate", "home"):
        p = t.pad(name)
        ok, why, slope = pad_truth(t, p.x, p.y)
        zones.append(Zone(name, p.x, p.y, p.z, p.radius, ok, why if not ok else ""))
    f = F.Forest(seed, "goal", start=(0.0, 0.0), goal=(zones[0].x, zones[0].y))
    h = t.pad("home")
    notes = dict(terrain_args=args, extra_rocks=extra, hazard=why_haz if (bad and not zones[0].safe) else "", alternate=[zones[1].x, zones[1].y],
                 designated_safe=zones[0].safe)  # fmt: skip
    return LandingWorld(family, seed, f, zones, (h.x, h.y, h.z), t, notes)
