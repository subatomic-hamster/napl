"""Subject following: the walking subject itself (scripted, ground truth), the detector that stands in for a person-detection network, and the tracker.

The autonomy may use ONLY what leaves this module's `SubjectDetector.detect`: the pixel box of the subject in the depth camera's image (centre and
extent, row / column), and the depth image itself. The 3D position is rebuilt from that pixel and the depth at it (`SubjectTracker`), with the vehicle's
own pose estimate; the ground-truth position is used inside `detect` only to decide WHERE a network would have drawn the box and whether the subject is
visible (in the field of view and not hidden behind something nearer in the depth image)."""

import numpy as np

SPEED = 2.4  # m/s: a jog
LOOKAHEAD = 1.4  # m: pure-pursuit distance
HEIGHT = 1.75  # m
CENTRE_Z = 1.0  # m: height of the box centre (torso)


class Subject:
    """Pure-pursuit walker along a polyline, driven through the velocity actuators `subject_vx / vy / wz` of scene.py."""

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


class SubjectDetector:
    """Stand-in for a person-detection network running on the depth camera's image. Returns the pixel box (row / col of the centre, half-height /
    half-width in pixels) of the subject, with 0.8 px of centre jitter and a 3 % chance of missing a frame, or None when the subject is outside the
    field of view, hidden behind something closer (its depth at the box centre is not within 0.8 m of the subject's range), nearer than the camera's
    minimum range or smaller than 4 pixels tall."""

    def __init__(self, cam, truth, rng, jitter_px=0.8, miss=0.03):
        self.cam, self.truth, self.rng, self.jitter, self.miss = (
            cam,
            truth,
            rng,
            jitter_px,
            miss,
        )

    def detect(self, depth, cam_pos, cam_R):
        c = self.cam
        p = self.truth()
        pc = (
            np.asarray([p[0], p[1], CENTRE_Z]) - cam_pos
        ) @ cam_R  # R^T (p - o): sensor frame
        i, j, z = c.project(pc[None])
        i, j, z = float(i[0]), float(j[0]), float(z[0])
        if not np.isfinite(i) or z < c.min_range + 0.2:
            return None
        f = 0.5 * c.n_v / np.tan(c.fov_v / 2)
        hh, hw = 0.5 * HEIGHT * f / z, 0.3 * f / z
        if hh < 2.0 or self.rng.random() < self.miss:
            return None
        r, q = int(round(i)), int(round(j))
        patch = depth[max(r - 1, 0) : r + 2, max(q - 1, 0) : q + 2]
        d = np.nanmedian(patch) if np.isfinite(patch).any() else np.nan
        if not np.isfinite(d) or abs(d - z) > 0.8:
            return None  # hidden
        jit = self.rng.normal(0.0, self.jitter, 2)
        return dict(row=i + jit[0], col=j + jit[1], half_h=hh, half_w=hw)


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


class SubjectTracker:
    """3D position from the box and the depth at it, alpha-beta velocity in the ground plane, coasting when the box is lost."""

    def __init__(self, alpha=0.5, beta=0.15, coast_s=6.0):
        self.alpha, self.beta, self.coast = alpha, beta, coast_s
        self.p = self.v = None
        self.t = self.t_seen = None

    def update(self, t, cam, det, depth, cam_pos, cam_R):
        """-> (position xy, velocity xy, seen bool) or None while nothing has been seen / the track expired."""
        if det is not None:
            r, q = int(round(det["row"])), int(round(det["col"]))
            patch = depth[max(r - 1, 0) : r + 2, max(q - 1, 0) : q + 2]
            if np.isfinite(patch).any():
                d = float(np.nanmedian(patch))
                ray = cam.tangents()[
                    min(max(r, 0), cam.n_v - 1), min(max(q, 0), cam.n_h - 1)
                ]
                m = (cam_pos + cam_R @ (ray * d))[:2]
                if self.p is None:
                    self.p, self.v = m, np.zeros(2)
                else:
                    dt = max(t - self.t, 1e-3)
                    pred = self.p + self.v * dt
                    res = m - pred
                    self.p = pred + self.alpha * res
                    self.v = self.v + self.beta * res / dt
                self.t = self.t_seen = t
                return self.p.copy(), self.v.copy(), True
        if self.p is None or t - self.t_seen > self.coast:
            return None
        dt = t - self.t
        self.p = self.p + self.v * dt
        self.v = self.v * 0.99  # 25 Hz: velocity half-life ~2.8 s while unseen
        self.t = t
        return self.p.copy(), self.v.copy(), False
