"""Kernel-side I/O of the cuas2 launcher model: a one-launcher scene (barrel level along +x, target plate downrange) and the rig that writes a
sampled Shot into the kernel launcher's per-shot attrs (muzzle_velocity, spin_rps, dir_offset_x/y, docs/cuas2/spec.md 7.4).

    rig = LauncherRig(tmp_path, calib.Calib(), plate_x=10.0)
    rig.fire(shot)              # shot = LauncherModel.sample_shot(): sets the attrs, then `fire`
    rig.step(700)               # 1 ms ticks
    rig.hit()                   # (x, y, z) of the newest hit on the plate / ground, or None

The scene is a fixture for the model (like tests/data/devices/launcher.xml); the turret scene with the cuas2 assembly is the INTEG worker's. The
site frame is x = up, y = right, z = barrel (+x world), the convention of Shot.attrs().
"""

import json
from pathlib import Path

import numpy as np

from .calib import Calib

_MJCF = """<mujoco model="cuas2_launcher">
  <option timestep="0.001"/>
  <worldbody>
    <geom name="floor" type="plane" size="500 500 1" contype="0" conaffinity="0"/>
    <body name="barrel" pos="0 0 {z}">
      <geom type="box" size="0.05 0.01 0.01" contype="0" conaffinity="0"/>
      <site name="muzzle" pos="0.05 0 0" xyaxes="0 0 1 0 -1 0"/>
    </body>
    <body name="target" pos="{px} 0 {z}">
      <geom name="plate" type="box" size="0.001 {half} {half}" contype="0" conaffinity="0"/>
    </body>
    <body name="pellet0" mocap="true" pos="0 0 -1000">
      <geom type="sphere" size="0.011" group="5" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <custom>
    <text name="napl.devices" data='{devs}'/>
  </custom>
</mujoco>
"""


def scene_xml(calib, plate_x=10.0, muzzle_z=10.0, half=5.0, **launcher):
    """MJCF text. `launcher` overrides device keys (cl_k=0, spin_tau=0 -> a launcher without the per-shot block, i.e. the cuas-lite device)."""
    b, lo = calib.ball, calib.launcher
    dev = dict(name="gun", site="muzzle", muzzle_velocity=30.0, pellet_mass=b.m_g / 1000, diameter=b.d_mm / 1000, cd=b.cd, max_time=3.0, ray_groups=31,
               visual_bodies=["pellet0"], cl_k=lo.cl_k, spin_tau=lo.spin_tau_s)  # fmt: skip
    dev.update(launcher)
    devs = json.dumps(
        {"environments": [{"name": "air"}], "launchers": [dev]}, separators=(",", ":")
    )
    return _MJCF.format(z=muzzle_z, px=plate_x, half=half, devs=devs)


class LauncherRig:
    def __init__(self, out_dir, calib=None, **scene):
        from .. import Sim  # the kernel extension: only imported when a rig is built

        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        (out / "scene.xml").write_text(scene_xml(calib or Calib(), **scene))
        self.sim = Sim.load(out / "scene.xml")
        self.path = "/devices/gun"
        self.shots = 0

    def step(self, n=1):
        return self.sim.step(n)

    def get(self, attr):
        return self.sim.query(self.path, attr)[0]

    def fire(self, shot):
        for k, v in shot.attrs().items():
            self.sim.set(self.path, k, v)
        self.sim.set(self.path, "fire", 1.0)
        self.shots += 1

    @property
    def rounds(self):
        return int(self.get("rounds")[0])

    def hit(self):
        n = int(self.get("hit_n")[0])
        if n == 0:
            return None
        k = (n - 1) % 16
        return np.array(self.get("hit_pos")[3 * k : 3 * k + 3])

    def hit_time(self):
        n = int(self.get("hit_n")[0])
        return float(self.get("hit_t")[(n - 1) % 16]) if n else None
