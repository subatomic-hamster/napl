"""Generators for marine hardware: the twin-hull catamaran (BlueBoat style: two deep demi-hulls with flat inner walls, a
hatch lid on each and aluminium cross-members between them). Frame: centred in x, y; keel at z = 0, deck at z = hull_depth;
bow towards +x."""

import math

from . import gen
from .mesh import loft, merge, rbox, revolve, rot, tube


def _hull(L, W, H, n=36, stations=28):
    """One demi-hull centred on the origin (x stern -> bow): superelliptic sections, elliptical bow plan, keel rising to a raked
    bow, a slightly narrower transom."""
    rings = []
    for i in range(stations):
        s = (
            1 - (1 - i / (stations - 1)) ** 1.5
        )  # stations bunch up towards the bow, where the plan curves
        u = max(0.0, (s - 0.55) / 0.45)
        w = (0.08 + 0.92 * math.sqrt(max(1 - u * u, 0.0))) * (
            0.9 + 0.1 * min(1.0, s / 0.12)
        )
        zb = 0.45 * H * ((max(0.0, s - 0.45) / 0.55) ** 2)
        h = H - zb
        ring = []
        for k in range(n):
            t = (
                -2 * math.pi * k / n
            )  # clockwise in (y, z) seen from +x = counter-clockwise seen along +x
            cs, sn = math.cos(t), math.sin(t)
            ring.append(
                (
                    -L / 2 + s * L,
                    w * W / 2 * math.copysign(abs(cs) ** 0.5, cs),
                    zb + h / 2 + h / 2 * math.copysign(abs(sn) ** 0.45, sn),
                )
            )
        rings.append(ring)
    return loft(rings)


@gen(
    "catamaran_hull",
    hull="hull-orange",
    lid="plastic-dark",
    beam="alu-anodised-grey",
    skeg="plastic-dark",
)
def catamaran_hull(c, **_):
    L = c.p("hull_length", "length", default=1.2)
    W = c.p("hull_width", default=0.17)
    H = c.p("hull_depth", default=0.20)
    B = c.p("beam", default=0.93)
    off = (B - W) / 2
    hull = _hull(L, W, H)
    hulls = merge([hull.at((0, off, 0)), hull.at((0, -off, 0))])
    lids = merge(
        rbox((0.30, 0.6 * W, 0.008), 0.003).at((0.05, s * off, H)) for s in (1, -1)
    )
    beams = merge(
        rbox((0.04, 2 * off, 0.035), 0.004).at((x, 0, 0.55 * H)) for x in (-0.15, 0.30)
    )
    out = {"hull": hulls, "lid": lids, "beam": beams}
    chord, depth = c.p("skeg_chord"), c.p("skeg_depth")
    if (
        chord and depth
    ):  # fixed skeg under the stern of each hull (params skeg_chord / skeg_depth / skeg_x)
        x = c.p("skeg_x", default=-0.45)
        out["skeg"] = merge(
            rbox((chord, 0.008, depth + 0.012), 0.002).at(
                (x, s * off, -depth / 2 + 0.006)
            )
            for s in (1, -1)
        )
    return out


@gen("ducted_thruster", can="alu-anodised-grey", duct="plastic-dark")
def ducted_thruster(c, **_):
    """Flooded thruster (T200 style): motor can with a rounded nose, short shroud ring held by four stator struts. Mount face at
    z = 0, prop side +z (the part's `diameter` over the shroud, `length_body` tall)."""
    D = c.p("diameter", default=0.10)
    Lb = c.p("length_body", default=0.113)
    rc = 0.24 * D
    can = revolve(
        [
            (0, 0),
            (rc, 0),
            (rc, 0.78 * Lb),
            (0.85 * rc, 0.9 * Lb),
            (0.5 * rc, 0.985 * Lb),
            (0, Lb),
        ],
        48,
    )
    ro = D / 2
    z0 = 0.4 * Lb
    shroud = tube(ro, ro - 0.005, Lb - z0, 64, z0=z0)
    struts = merge(
        rbox((ro - rc, 0.004, 0.03), 0.001)
        .at((rc + (ro - rc) / 2 - 0.002, 0, 0.55 * Lb))
        .at(R=rot([0, 0, 1], k * math.pi / 2))
        for k in range(4)
    )
    return {"can": can, "duct": merge([shroud, struts])}


@gen(
    "esc_sealed",
    body="alu-anodised-black",
    label="label-white",
    fins="alu-anodised-grey",
)
def esc_sealed(c, **_):
    """Waterproof marine ESC: sealed rounded case with a label on top and cooling ribs along both long sides. Underside at
    z = 0 (size_length x size_width x size_height)."""
    L = c.p("size_length", default=0.06)
    W = c.p("size_width", default=0.038)
    H = c.p("size_height", default=0.025)
    body = rbox((L, W, H), 0.003).at((0, 0, H / 2))
    label = rbox((0.6 * L, 0.6 * W, 0.0006), 0.0001).at((0, 0, H + 0.0001))
    xs = [(i - 3.5) * 0.09 * L for i in range(8)]
    fins = merge(
        rbox((0.0025, 0.003, 0.7 * H), 0.0005).at((x, s * (W / 2 + 0.0008), 0.5 * H))
        for x in xs
        for s in (1, -1)
    )
    return {"body": body, "label": label, "fins": fins}
