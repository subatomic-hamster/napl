"""Subject following: the walking subject itself (scripted, ground truth) and two small helpers for the map. The person DETECTOR and the tracker are
in detector.py and see only the depth image and the pose estimate; nothing in this module's `Subject` is ever read by the autonomy (fly.py reads
`Subject.pos()` for evaluation only: the follow distance and the detection statistics)."""

import numpy as np

SPEED = 2.4  # m/s: a jog
LOOKAHEAD = 1.4  # m: pure-pursuit distance
HEIGHT = 1.75  # m
CENTRE_Z = 1.0  # m: height of the box centre (torso)


class Subject:
    """Pure-pursuit walker along a polyline, driven through the velocity actuators `subject_vx / vy / wz` of skydio_scene.py."""

    def __init__(self, sim, path, speed=SPEED):
        self.sim, self.path, self.speed = sim, np.array(path, float), speed
        self.s = 0.0  # arc length done
        seg = np.diff(self.path, axis=0)
        self.cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(seg, axis=1))])
        self.length = float(self.cum[-1])
        self.go = False

    def pos(self):
        return self.sim.query("/world/subject", "pos")[0]

    def vel(self):
        return np.array(
            [
                self.sim.query("/world/subject/subject_x", "qvel")[0][0],
                self.sim.query("/world/subject/subject_y", "qvel")[0][0],
            ]
        )

    def _at(self, s):
        s = min(max(s, 0.0), self.length)
        i = min(int(np.searchsorted(self.cum, s, side="right")) - 1, len(self.path) - 2)
        f = (s - self.cum[i]) / max(self.cum[i + 1] - self.cum[i], 1e-9)
        return self.path[i] + f * (self.path[i + 1] - self.path[i])

    def update(self):
        p = self.pos()[:2]
        if self.go:  # progress = projection of the walker on the path, never backwards
            near = np.linspace(self.s, min(self.s + 3.0, self.length), 31)
            pts = np.array([self._at(s) for s in near])
            self.s = float(near[np.argmin(np.linalg.norm(pts - p, axis=1))])
        tgt = self._at(self.s + LOOKAHEAD)
        d = tgt - p
        n = np.linalg.norm(d)
        arrived = self.s >= self.length - 0.3
        v = np.zeros(2) if (not self.go or arrived or n < 1e-6) else self.speed * d / n
        yaw = float(self.sim.query("/world/subject/subject_yaw", "qpos")[0][0])
        want = np.arctan2(d[1], d[0]) if n > 1e-6 and self.go and not arrived else yaw
        w = float(
            np.clip(4.0 * np.arctan2(np.sin(want - yaw), np.cos(want - yaw)), -4.0, 4.0)
        )
        for name, val in (
            ("subject_vx", v[0]),
            ("subject_vy", v[1]),
            ("subject_wz", w),
        ):
            self.sim.set(f"/actuators/{name}", "ctrl", float(val))


def subject_mask(cam, det, margin=1.2):
    """Bool (n_v, n_h) of the pixels inside the detection box (grown by `margin` px): left out of the map so the subject leaves no trail."""
    m = np.zeros((cam.n_v, cam.n_h), bool)
    r0, r1 = (
        int(np.floor(det["row"] - det["half_h"] - margin)),
        int(np.ceil(det["row"] + det["half_h"] + margin)) + 1,
    )
    c0, c1 = (
        int(np.floor(det["col"] - det["half_w"] - margin)),
        int(np.ceil(det["col"] + det["half_w"] + margin)) + 1,
    )
    m[max(r0, 0) : max(r1, 0), max(c0, 0) : max(c1, 0)] = True
    return m


def track_box(cam, xy, cam_pos, cam_R):
    """Pixel box (same keys as a detection) where the TRACKED position (estimate frame) should appear, or None outside the field of view: used to
    keep the subject out of the map in frames where the detector missed it."""
    pc = (np.asarray([xy[0], xy[1], CENTRE_Z]) - cam_pos) @ cam_R
    i, j, z = cam.project(pc[None])
    i, j, z = float(i[0]), float(j[0]), float(z[0])
    if not np.isfinite(i) or z < cam.min_range:
        return None
    f = 0.5 * cam.n_v / np.tan(cam.fov_v / 2)
    return dict(row=i, col=j, half_h=0.5 * HEIGHT * f / z, half_w=0.3 * f / z)
