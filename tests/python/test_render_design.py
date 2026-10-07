"""R1: built designs look like hardware (per-part colours from material parts / builder table), orbit camera presets,
`napl render` showcase frames and the animated PNG writer. `gpu`-marked tests need a Vulkan device."""

import re
import struct
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design, showcase

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.fixture(scope="module")
def models(tmp_path_factory):
    out = {}
    for n in ("x500v2", "so101"):
        out[n] = design.build(
            REPO / "assemblies" / f"{n}.toml", PARTS, tmp_path_factory.mktemp(n)
        )["xml"]
    return out


def _rgba(xml):
    return {
        g.get("name"): tuple(float(x) for x in g.get("rgba").split())
        for g in ET.parse(xml).getroot().iter("geom")
        if g.get("group") == "4"
    }


def test_builder_emits_a_colour_per_vehicle_geom(models):
    lib = design.Library(PARTS)
    carbon = tuple(lib.get("carbon-fibre-epoxy-ud").raw["appearance"]["rgba"])
    pla = tuple(lib.get("pla").raw["appearance"]["rgba"])
    x500, so = _rgba(models["x500v2"]), _rgba(models["so101"])
    assert all(len(c) == 4 and c[3] == 1.0 for c in (*x500.values(), *so.values()))
    assert any(c == carbon for c in x500.values())  # arm tubes
    assert any(c == pla for c in so.values())  # printed SO-101 parts
    assert len(set(x500.values())) >= 5 and len(set(so.values())) >= 3
    assert not any(
        c == (0.5, 0.5, 0.5, 1.0) for c in x500.values()
    )  # none left on MuJoCo's default grey


def test_appearance_is_cosmetic_and_builds_stay_deterministic(models, tmp_path):
    again = design.build(REPO / "assemblies" / "so101.toml", PARTS, tmp_path)["xml"]
    assert again.read_bytes() == models["so101"].read_bytes()
    stripped = lambda p: re.sub(r' rgba="[^"]*"', "", p.read_text())
    assert "rgba" not in stripped(
        again
    )  # colours are the only appearance-related attribute emitted


def test_apng_writer_roundtrips_frame_count_and_pixels():
    fr = [np.full((4, 6, 4), 40 * k, np.uint8) for k in range(3)]
    data = showcase.apng(fr, fps=10)
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, kinds, idat, fdat = 8, [], b"", []
    while pos < len(data):
        n, kind = struct.unpack(">I4s", data[pos : pos + 8])
        body = data[pos + 8 : pos + 8 + n]
        (crc,) = struct.unpack(">I", data[pos + 8 + n : pos + 12 + n])
        assert crc == zlib.crc32(kind + body)
        kinds.append(kind)
        if kind == b"acTL":
            assert struct.unpack(">II", body) == (3, 0)
        idat += body if kind == b"IDAT" else b""
        fdat += [body[4:]] if kind == b"fdAT" else []
        pos += 12 + n
    assert kinds[0] == b"IHDR" and kinds[-1] == b"IEND" and kinds.count(b"fcTL") == 3
    rows = zlib.decompress(idat)
    assert rows[1:25] == fr[0][0].tobytes()  # filter byte 0, then row 0
    assert zlib.decompress(fdat[1])[1:25] == fr[2][0].tobytes()


def _subject(rgba):
    """Mask of pixels that are not the floor (its flat colour is the image's most common one) or the sky."""
    flat = rgba.reshape(-1, 4)
    vals, counts = np.unique(flat, axis=0, return_counts=True)
    floor = vals[counts.argmax()]
    return (rgba != floor).any(-1)


@needs_kernel
@pytest.mark.gpu
@pytest.mark.parametrize("name", ["x500v2", "so101"])
def test_built_assembly_renders_centred_with_sane_depth(models, name):
    s = napl.Sim.load(models[name])
    pos, tgt = s.orbit_camera("iso")
    dist = float(np.linalg.norm(pos - tgt))
    rgba, depth = s.render(camera="iso", width=320, height=240)
    m = _subject(rgba)
    ys, xs = np.nonzero(m)
    assert 0.02 < m.mean() < 0.7, f"subject covers {m.mean():.3f} of the frame"
    assert abs(xs.mean() - 160) < 60 and abs(ys.mean() - 120) < 50  # centred
    assert xs.min() > 0 and xs.max() < 319 and ys.min() > 0  # fully in frame
    d = depth[m]
    assert np.isfinite(d).all() and d.min() > 0.2 * dist and d.max() < 2 * dist
    assert (
        len({tuple(p) for p in rgba[m][::7]}) > 8
    )  # coloured parts, not one flat blob


@needs_kernel
@pytest.mark.gpu
def test_orbit_camera_args_and_joint_motion_change_the_image(models):
    s = napl.Sim.load(models["so101"])
    a, _ = s.render(camera="orbit", az=30, el=20, width=160, height=120)
    b, _ = s.render(camera="orbit", az=120, el=20, width=160, height=120)
    assert not np.array_equal(a, b)
    body = next(p for p in s.body_positions() if p.endswith("/moving_jaw"))
    c, _ = s.render(camera="iso", target=body, dist=0.3, width=160, height=120)
    assert _subject(c).mean() > 0.1  # a close-up on one body target still sees the arm
    with pytest.raises(ValueError):
        s.render(camera="sideways")
    with pytest.raises(ValueError):
        s.render()  # neither pos/lookat nor camera=


@needs_kernel
@pytest.mark.gpu
def test_napl_render_cli_writes_views_and_an_animation(models, tmp_path):
    from napl.cli import main

    out = tmp_path / "frames"
    rc = main(
        [
            "render", str(models["so101"]), "--out", str(out), "--views", "iso,top",
            "--size", "160x120", "--frames", "4", "--sweep", "shoulder_pan:-0.6:0.6",
        ]
    )  # fmt: skip
    assert rc == 0
    assert (out / "so101-iso.png").read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert (out / "so101-top.png").stat().st_size > 500
    anim = (out / "so101-anim.apng").read_bytes()
    assert anim.count(b"fcTL") == 4
