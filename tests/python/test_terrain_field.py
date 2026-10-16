"""Small-scale terrain fields (python/napl/terrain_field.py, docs/models.md "Small-scale terrain"): determinism across processes, no seams between
patches, the per-kind contents, per-patch material friction in the physics, ground truth against ray casts, and the contact validation of a
12 kg dummy dog (risk 2). Needs the built kernel for the physics tests."""

import subprocess
import sys
import xml.etree.ElementTree as ET

import napl
import numpy as np
import pytest
from napl.terrain import MATERIALS, add_field, field

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
KINDS = ("rubble", "slopes", "scree", "ruins")


@pytest.fixture(scope="module", params=KINDS)
def full(request):
    return field(request.param, seed=3)  # 100 x 100 m at 5 cm: 2001 x 2001


def small(kind, **kw):
    return field(kind, size=20.0, seed=kw.pop("seed", 5), **kw)


def test_full_size_field_shape_and_patch_tiling(full):
    assert (
        full.heights.shape == (2001, 2001) and full.size == 100.0 and full.res == 0.05
    )
    assert len(full.patches) == 100 and {p.material for p in full.patches} <= set(
        MATERIALS
    )
    assert (
        np.all(np.isfinite(full.heights))
        and full.heights.max() - full.heights.min() < 20.0
    )


def test_same_seed_same_digest_across_processes():
    here = small("rubble").digest()
    code = "from napl.terrain import field; print(field('rubble', size=20.0, seed=5).digest())"
    out = subprocess.run(
        [sys.executable, "-I", "-c", code], capture_output=True, text=True, check=True
    ).stdout.strip()
    assert out == here
    assert small("rubble", seed=6).digest() != here
    assert small("ruins").digest() == small("ruins").digest()


@pytest.mark.parametrize("kind", KINDS)
def test_no_seams_between_patches(kind):
    """The hfield of neighbouring patches share their border row / column in world z (as emitted into the MJCF), and the global grid has no jump at
    the borders beyond what it has inside patches."""
    f = small(kind)
    root = ET.fromstring("<mujoco><worldbody/></mujoco>")
    add_field(root, f)
    hf = {h.get("name"): h for h in root.iter("hfield")}
    geom = {g.get("name"): g for g in root.iter("geom") if g.get("type") == "hfield"}

    def world(
        a, b
    ):  # (rows, cols) of world z for patch (a = x index, b = y index), data row 0 is the +y edge
        g = geom[f"fld_p{b}_{a}"]
        h = hf[g.get("hfield")]
        n = int(h.get("nrow"))
        elev = float(h.get("size").split()[2])
        z0 = float(g.get("pos").split()[2])
        return z0 + elev * np.array(h.get("elevation").split(), float).reshape(n, n)

    for b in range(2):
        for a in range(2):
            w = world(a, b)
            if a == 0:  # east edge of (0, b) == west edge of (1, b)
                assert np.allclose(w[:, -1], world(1, b)[:, 0], atol=1e-5)
            if (
                b == 0
            ):  # north edge (data row 0) of (a, 0) == south edge (last data row) of (a, 1)
                assert np.allclose(w[0, :], world(a, 1)[-1, :], atol=1e-5)
    # the stored patch heights are the global heights (up to the float text format)
    w = world(1, 0)  # data row k = global row (pc - k)
    pc = 200
    assert np.allclose(w[::-1], f.heights[0 : pc + 1, pc : 2 * pc + 1], atol=1e-5)
    d = np.abs(np.diff(f.heights, axis=1))
    border = d[:, pc - 1]  # step across the patch border column
    inner = d[:, pc // 2]
    assert (
        border.max() < 3 * max(inner.max(), 0.2)
        and np.abs(border - inner).mean() < 0.05
    )


def test_rubble_rocks_are_5_to_20_cm_and_boulders_exist():
    f = small("rubble")
    assert len(f.rocks) > 100 and all(0.05 <= h <= 0.20 for _, _, _, h in f.rocks)
    bump = f.heights - f.base
    assert bump.min() >= -1e-12 and 0.05 <= bump.max() <= 0.20 + 1e-9
    assert len(f.boxes) >= 10 and all(b.material == "rock" for b in f.boxes)


def test_slopes_have_15_to_30_degree_ramps_and_10_to_18_cm_steps():
    f = small("slopes", seed=2)
    prof = f.heights.mean(
        axis=0
    )  # the 1-D terrace profile (cross-y roughness averages out)
    dh = np.diff(prof)
    ang = np.degrees(np.arctan(np.abs(dh) / f.res))
    ramp, step = ang < 45, ang >= 45
    assert step.sum() >= 3
    assert (
        0.10 - 0.015 <= np.abs(dh[step]).min()
        and np.abs(dh[step]).max() <= 0.18 + 0.015
    )
    r = ang[ramp & (np.abs(dh) > 1e-4)]
    assert np.mean((r > 14) & (r < 31)) > 0.9


def test_scree_has_low_friction_patches():
    f = small("scree", seed=1)
    fr = [MATERIALS[p.material].friction[0] for p in f.patches]
    assert min(fr) < 0.4 and max(fr) >= 0.5
    g = field("scree", seed=1)
    assert sum(MATERIALS[p.material].friction[0] < 0.4 for p in g.patches) >= 10


def test_ruins_are_a_wall_maze_with_gaps_of_at_least_0_6_m():
    f = field("ruins", seed=4)
    assert len(f.boxes) > 400
    lines = {}
    for b in f.boxes:
        along_y = b.size[1] > b.size[0]
        key = ("x" if along_y else "y", round(b.pos[0] if along_y else b.pos[1], 6))
        lo, hi = (
            (b.pos[1] - b.size[1], b.pos[1] + b.size[1])
            if along_y
            else (b.pos[0] - b.size[0], b.pos[0] + b.size[0])
        )
        lines.setdefault(key, []).append((lo, hi))
    gaps = []
    for spans in lines.values():
        spans.sort()
        gaps += [b[0] - a[1] for a, b in zip(spans, spans[1:]) if b[0] - a[1] > 1e-6]
    assert len(gaps) > 100 and min(gaps) >= 0.6 and max(gaps) <= 1.6 + 1e-6


def test_ground_truth_includes_boxes_and_materials():
    f = small("ruins")
    b = f.boxes[0]
    top = b.pos[2] + b.size[2]
    assert f.height(b.pos[0], b.pos[1]) == pytest.approx(top)
    assert f.material_at(b.pos[0], b.pos[1]) == "rock"
    g = small("scree")
    p = g.patches[0]
    assert g.material_at((p.x0 + p.x1) / 2, (p.y0 + p.y1) / 2) == p.material


def sim_of(f, extra="", dt=0.002):
    xml = f'<mujoco><option timestep="{dt}"/><worldbody><light pos="0 0 10"/>{extra}</worldbody></mujoco>'
    text, names = add_field(xml, f)
    return text, names


def load_text(tmp_path, text, name="m.xml"):
    p = tmp_path / name
    p.write_text(text)
    return napl.Sim.load(p)


@needs_kernel
def test_ground_truth_height_equals_a_ray_cast_within_1_mm(tmp_path):
    f = small("rubble", seed=9)
    text, _ = sim_of(f)
    s = load_text(tmp_path, text)
    rng = np.random.default_rng(0)
    pts = rng.uniform(-9.5, 9.5, (1000, 2))
    got = np.array([12.0 - s.raycast([x, y, 12.0], [[0, 0, -1]])[0] for x, y in pts])
    err = np.abs(got - f.height(pts[:, 0], pts[:, 1]))
    assert err.max() < 1e-3, (err.max(), np.argmax(err))


@needs_kernel
def test_contact_friction_is_the_patch_material(tmp_path):
    """A box with the default friction 1 slid at 2 m/s over a flat patch decelerates at mu g: the patch's friction (priority 1) is what acts."""
    f = small("rubble", seed=1)
    f.heights[:] = 0.0
    f.boxes = []
    f.grid[:] = np.array([["mud", "rock"], ["gravel", "wet_rock"]], dtype=object)
    for p in f.patches:  # the patches carry the material into the MJCF
        p.material = f.grid[int(p.y0 >= 0), int(p.x0 >= 0)]
    body = '<body name="b" pos="{x} {y} 0.0505"><freejoint name="fj"/><geom type="box" size="0.1 0.1 0.05" mass="1"/></body>'
    for mat, (x, y) in {
        "mud": (-8.0, -5.0),
        "rock": (2.0, -5.0),
        "gravel": (-8.0, 5.0),
        "wet_rock": (2.0, 5.0),
    }.items():
        text, _ = sim_of(f, body.format(x=x, y=y), dt=0.001)
        s = load_text(tmp_path, text, f"{mat}.xml")
        s.step(300)  # settle
        s.set("/world/b/fj", "qvel", [2.0, 0, 0, 0, 0, 0])
        s.step(
            100
        )  # let the landing / launch transient pass, then measure the deceleration
        v1 = s.query("/world/b/fj", "qvel")[0][0]
        s.step(100)
        v2 = s.query("/world/b/fj", "qvel")[0][0]
        mu = MATERIALS[mat].friction[0]
        assert (v1 - v2) / 0.1 == pytest.approx(mu * 9.81, rel=0.1), (mat, v1, v2)


DOG = """<body name="dog" pos="{x} {y} {z}"><freejoint name="fj"/>
<geom type="box" size="0.25 0.12 0.05" mass="10" contype="0" conaffinity="0"/>
<geom name="f0" type="sphere" size="0.03" pos=" 0.2  0.14 -0.15" mass="0.5"/><geom name="f1" type="sphere" size="0.03" pos=" 0.2 -0.14 -0.15" mass="0.5"/>
<geom name="f2" type="sphere" size="0.03" pos="-0.2  0.14 -0.15" mass="0.5"/><geom name="f3" type="sphere" size="0.03" pos="-0.2 -0.14 -0.15" mass="0.5"/></body>"""


def flattest(f, w=1.0):
    """Centre of the flattest w x w m window on a 1 m grid (smallest peak-to-peak height), clear of boxes."""
    k = int(round(w / f.res))
    best = (9e9, 0, 0)
    for x in np.arange(-f.size / 2 + 2, f.size / 2 - 2, 1.0):
        for y in np.arange(-f.size / 2 + 2, f.size / 2 - 2, 1.0):
            i, j = (
                int(round((x + f.size / 2) / f.res)),
                int(round((y + f.size / 2) / f.res)),
            )
            win = f.heights[j - k : j + k + 1, i - k : i + k + 1]
            if any(
                abs(b.pos[0] - x) < 1.2 and abs(b.pos[1] - y) < 1.2 for b in f.boxes
            ):
                continue
            best = min(best, (float(np.ptp(win)), x, y))
    return best[1], best[2]


@needs_kernel
@pytest.mark.parametrize("kind", KINDS)
@pytest.mark.parametrize("dt", [0.001, 0.002])
def test_dummy_dog_drop_rest_and_drag_without_tunnelling_or_jitter(tmp_path, kind, dt):
    """12 kg box on four 3 cm spheres (friction from the patch): dropped 20 cm onto the flattest spot, it must rest within 1 mm of still, then be
    dragged (1 m/s impulse) without ever sinking through the surface."""
    f = small(kind, seed=11)
    x, y = flattest(f)
    z = float(f.height(x, y)) + 0.15 + 0.03 + 0.2
    text, _ = sim_of(f, DOG.format(x=x, y=y, z=z), dt=dt)
    s = load_text(tmp_path, text)
    n = lambda sec: int(round(sec / dt))
    s.step(n(2.5))
    zs = []
    for _ in range(n(1.0) // 10):
        s.step(10)
        zs.append(s.query("/world/dog", "pos")[0][2])
    assert np.ptp(zs) < 1e-3, (kind, dt, np.ptp(zs))
    s.set("/world/dog/fj", "qvel", [1.0, 0, 0, 0, 0, 0])
    for _ in range(n(1.5) // 10):
        s.step(10)
        px, py, pz = s.query("/world/dog", "pos")[0]
        assert np.isfinite(pz) and pz > f.height(px, py) - 0.05, (kind, dt, px, py, pz)
