"""FootContact (SP2, docs/models.md "FootContact") through the Python API. The design-side binding (type = "foot_contact") is covered in
test_qdd.py with the actuator it belongs to. Needs the built kernel."""

import napl
import pytest

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
FOOT_JSON = """{"foot_contacts": [{"name": "f1", "geom": "foot", "mode": 0, "f_max": 500, "noise_std": 0,
 "hysteresis_on": 2, "hysteresis_off": 1, "debounce_s": 0.005, "rate_hz": 1000}]}"""


def scene(tmp_path, mass=2.0, z=0.0505):
    p = tmp_path / "foot.xml"
    p.write_text(
        '<mujoco><option timestep="0.001"/><worldbody><geom name="floor" type="plane" size="5 5 0.1"/>'
        f'<body name="ball" pos="0 0 {z}"><freejoint name="fj"/><geom name="foot" type="sphere" size="0.05" mass="{mass}"/></body></worldbody>'
        f"<custom><text name=\"napl.devices\" data='{FOOT_JSON}'/></custom></mujoco>"
    )
    return napl.Sim.load(p)


def val(s, attr):
    return float(s.query("/devices/f1", attr)[0][0])


@needs_kernel
def test_force_is_the_weight_and_contact_is_on(tmp_path):
    s = scene(tmp_path)
    s.step(1500)
    assert val(s, "force") == pytest.approx(2 * 9.81, rel=0.02)
    assert val(s, "contact") == 1.0
    assert s.query("/devices/f1", "force")[1] == "N"


@needs_kernel
def test_airborne_foot_reads_zero_and_readings_are_read_only(tmp_path):
    s = scene(tmp_path, z=1.0)
    s.step(50)  # still falling
    assert val(s, "force") == 0.0 and val(s, "contact") == 0.0
    with pytest.raises(Exception):
        s.set("/devices/f1", "force", 1.0)
