import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import numpy as np
import pytest
from autonomy import config, iface


def test_shipped_config_loads_with_provenance_on_every_leaf():
    c = config.load()
    assert c["map"]["res"] == 0.05 and c["map"]["size"] == 100.0
    assert c["lidar"]["rays_mission"] == 5000 and c["lidar"]["rays_quality"] == 20000
    assert c["trav"]["slope_max"] == pytest.approx(np.radians(30))
    assert c["plan"]["t_budget"] == 1200.0
    assert set(c) == {
        "lidar",
        "gnss",
        "odom",
        "carrier",
        "dog",
        "map",
        "trav",
        "plan",
        "icp",
        "loc",
    }


@pytest.mark.parametrize(
    "body, msg",
    [
        ("[map]\nres = 0.05\n", "map.res: bare number"),
        (
            '[map]\nres = { value = 0.05, unit = "m", fidelity = "design" }\n',
            "map.res: missing rationale",
        ),
        (
            '[map]\nres = { value = 0.05, unit = "m", fidelity = "guess", rationale = "x" }\n',
            "map.res: fidelity",
        ),
    ],
)
def test_numbers_rule_refuses_bare_or_unsourced_leaves(tmp_path, body, msg):
    p = tmp_path / "a.toml"
    p.write_text(body)
    with pytest.raises(ValueError, match=msg):
        config.load(p)


def test_override_copies_and_refuses_unknown_keys():
    c = config.load()
    d = config.override(c, **{"lidar.rays_mission": 2500})
    assert d["lidar"]["rays_mission"] == 2500 and c["lidar"]["rays_mission"] == 5000
    with pytest.raises(KeyError):
        config.override(c, **{"lidar.nope": 1})


def test_locomotion_protocol_is_structural():
    class Fake:
        sim = None
        dt = 0.01
        lidar, gnss, base = "lidar", "gnss", "/world/carrier"

        def advance(self, seconds): ...
        def command(self, vx, vy, wz): ...
        def odometry(self):
            return iface.Odom(
                0.0,
                np.zeros(3),
                np.array([1.0, 0, 0, 0]),
                np.zeros(3),
                np.zeros(3),
                False,
            )

        def fix(self, p_xy, yaw, t_pose, sig_p=None, sig_yaw=None): ...
        def limits(self):
            return {"v_max": 1.5, "wz_max": 1.0}

        def truth(self):
            return np.zeros(3), np.array([1.0, 0, 0, 0])

    assert isinstance(Fake(), iface.Locomotion)
    assert not isinstance(object(), iface.Locomotion)
