"""The turret camera as the program sees it: numpy frames from the `camera` device's pose, at the real camera's field of view.

    cam = Camera(sim, "/devices/turret_cam", fov_v=0.7156, width=1280, height=720)
    f = cam.grab()          # Frame(rgb uint8 (h, w, 3), depth float32 (h, w) | None, pos, fwd, up, t)
    cam.ray(u, v, f)        # unit world direction of pixel (u, v) (u right, v down, pixel centres at +0.5)
    cam.project(x, f)       # pixel (u, v, depth) of a world point, or None behind the camera

The kernel's camera device is pose-only: it publishes the site's position / forward / up each tick (queries `pos`, `fwd`, `up` on the device
prim). The image comes from the renderer (`Sim.render`, Vulkan, a pinhole at `fovy` with +z up), looking along `fwd`. A pan / tilt turret never
rolls, so +z up equals the camera's up. `depth` (linear metres along the optical axis) is GROUND TRUTH from the z-buffer: the turret program never
reads it (examples/cuas/turret.py estimates range from apparent size); it is returned for the tests and plots only.

Determinism: the pose, and everything the physics sees, is bit-exact (and in the replay hash). The PIXELS are deterministic per GPU / driver
only (Sim.render says so): another GPU, another driver version or a software rasteriser can differ in the last bit of a colour, hence in
the odd edge pixel of a detection. The tracker works on thresholds with margin, so tracks and IDs come out the same; if a pixel flips, the
program's decisions (and so the SetAttr stream and the hash) CAN differ across machines. On one machine and one driver, runs repeat exactly.
"""

import math
from dataclasses import dataclass

import numpy as np


def write_png(path, rgb):
    """Minimal PNG writer (uint8 (h, w, 3)): zlib + struct, no imaging dependency."""
    import struct
    import zlib

    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
    chunk = lambda t, d: (
        struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d))
    )
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 1)) + chunk(b"IEND", b""))  # fmt: skip


@dataclass
class Frame:
    rgb: np.ndarray
    depth: "np.ndarray | None"
    pos: np.ndarray
    fwd: np.ndarray
    up: np.ndarray
    t: float
    index: int = 0


class Camera:
    def __init__(self, sim, path, fov_v, width=1280, height=720):
        self.sim, self.path, self.fov_v, self.w, self.h = (
            sim,
            path,
            float(fov_v),
            int(width),
            int(height),
        )
        self.f_px = (
            0.5 * self.h / math.tan(self.fov_v / 2)
        )  # focal length in pixels (square pixels)
        self.fov_h = 2 * math.atan(0.5 * self.w / self.f_px)
        self.n = 0

    def pose(self):
        q = self.sim.query
        return q(self.path, "pos")[0], q(self.path, "fwd")[0], q(self.path, "up")[0]

    def grab(self, t=0.0, depth=False) -> Frame:
        pos, fwd, up = self.pose()
        rgba, d = self.sim.render(
            pos, pos + fwd, math.degrees(self.fov_v), self.w, self.h
        )
        self.n += 1
        return Frame(
            np.ascontiguousarray(rgba[..., :3]),
            d if depth else None,
            pos,
            fwd,
            up,
            t,
            self.n,
        )

    def _basis(self, fr):
        right = np.cross(fr.fwd, fr.up)
        right /= np.linalg.norm(right)
        return right, np.cross(right, fr.fwd)

    def ray(self, u, v, fr):
        right, up = self._basis(fr)
        d = (
            fr.fwd
            + right * (u - self.w / 2) / self.f_px
            - up * (v - self.h / 2) / self.f_px
        )
        return d / np.linalg.norm(d)

    def project(self, x, fr):
        right, up = self._basis(fr)
        r = np.asarray(x, float) - fr.pos
        z = float(r @ fr.fwd)
        if z <= 1e-6:
            return None
        return (
            self.w / 2 + self.f_px * float(r @ right) / z,
            self.h / 2 - self.f_px * float(r @ up) / z,
            z,
        )
