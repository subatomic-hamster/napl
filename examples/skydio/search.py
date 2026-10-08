"""Where is the subject while the detector does not see it? A particle belief over the subject's ground position and velocity, and the visibility of
that belief from a candidate viewpoint. numpy only.

The belief starts from the tracker's Kalman state at the moment of the loss and is propagated like a walker: heading and speed diffuse (people turn
and change pace), and a particle never walks into an occupied cell of the map (the trunks, poles and boulders the drone has already seen). Each depth
frame then removes the mass the camera could have seen but did not: a particle inside the field of view, inside the detectable range band and with an
unoccluded line of sight in the map is down-weighted by (1 - P_DETECT). What is left is hidden behind something or outside the field of view, which
is where a real follower looks next. `visible` is the same geometry used the other way round: the fraction of the belief a candidate position would
see, which the planner adds to the follow objective (visibility-aware viewpoint selection).

Detectable range band, from the sensor spec: the detector needs the legs down to 0.9 m (detector.MIN_SPAN / z_low), the D435i's vertical field of
view is 58 deg (+-29 deg), so a person closer than (z_cam - 0.9) / tan(29 deg - margin) is cut off at the knees: ~2.4 m at a 1.9 m flying height.
The far limit is the 8 m past which the person is under 2 px wide-high (follow.py / _score)."""

import numpy as np

R_MIN, R_MAX = (
    2.4,
    7.0,
)  # m: horizontal range band where the detector reliably finds a person (see above; it misses more beyond 7 m, measured in the flights)
HALF_FOV = np.radians(
    28.0
)  # the camera's +-43.5 deg, less the outer band where the person is cut by the image border or foreshortened (the detector rejects blobs touching the border; a subject seen at 35-41 deg was missed for seconds in flight)
P_DETECT = 0.35  # how much a miss counts against a visible hypothesis. The detector's recall is 0.88 on average but its misses come in runs of seconds (pose, range, partial occlusion by low obstacles), so one miss is weak evidence: the mass must not be wiped out and jump to a far hidden corner
CHEST_Z = 1.2  # m: the point of the person that must be unoccluded
TURN_NOISE, SPEED_NOISE, POS_NOISE = (
    0.8,
    0.25,
    0.3,
)  # rad/s, m/s and m per sqrt(s): a walker keeps its pace but its route bends (the forest's paths turn 60-90 deg at their corners)
MODE_R, EMA = (
    1.5,
    0.35,
)  # m: kernel radius of the cluster search; smoothing of the estimate per depth frame
N_LOS = 40  # samples along a line of sight (0.2 m voxels, <= 7.5 m)


def visible(win, view, tgt, face, r_min=R_MIN, r_max=R_MAX):
    """-> bool (V, N): target n (ground xy, (N, 2)) seen from viewpoint v ((V, 3), camera position) looking along the unit xy vector face[v]: inside the
    range band and field of view, and the line of sight to its chest crosses no occupied cell of the window (unknown cells are free: optimistic)."""
    d = tgt[None, :, :] - view[:, None, :2]
    r = np.linalg.norm(d, axis=2)
    ok = (
        (r >= r_min)
        & (r <= r_max)
        & ((d * face[:, None, :]).sum(2) >= r * np.cos(HALF_FOV))
    )
    vi, ni = np.nonzero(ok)
    if len(vi):
        f = np.linspace(0.08, 0.97, N_LOS)[None, :, None]
        a = view[vi][:, None, :]
        b = np.concatenate([tgt[ni], np.full((len(ni), 1), CHEST_Z)], 1)[:, None, :]
        pts = (a + f * (b - a)).reshape(-1, 3)
        i, inside = win.index(pts)
        hit = (
            (win.occ[i[:, 0], i[:, 1], i[:, 2]] & inside).reshape(len(vi), N_LOS).any(1)
        )
        ok[vi[hit], ni[hit]] = False
    return ok


class Belief:
    def __init__(self, rng, n=120):
        self.rng, self.n = rng, n
        self.xy = None

    @property
    def active(self):
        return self.xy is not None

    def start(self, x, P):
        """Particles from the tracker's state x = (px, py, vx, vy) and covariance P."""
        s = self.rng.multivariate_normal(x, P + 1e-6 * np.eye(4), self.n)
        self.xy, self.v = s[:, :2], s[:, 2:]
        self.w = np.full(self.n, 1.0 / self.n)
        self.m_prev = self.v_prev = None
        pace = float(np.hypot(x[2], x[3]))
        self.pace = (0.6 * pace, 1.4 * pace) if pace > 0.8 else (0.0, 3.0)

    def stop(self):
        self.xy = None

    def predict(self, dt, win):
        n, rng = self.n, self.rng
        sp = np.linalg.norm(self.v, axis=1)
        th = np.arctan2(self.v[:, 1], self.v[:, 0]) + rng.normal(
            0, TURN_NOISE * np.sqrt(dt), n
        )
        sp = sp + rng.normal(0, SPEED_NOISE * np.sqrt(dt), n)
        sp = np.clip(
            sp, *self.pace
        )  # a walker that was walking keeps walking (it stops only where its route ends): no hidden "stood still" mode
        v = sp[:, None] * np.stack([np.cos(th), np.sin(th)], 1)
        new = self.xy + v * dt + rng.normal(0, POS_NOISE * np.sqrt(dt), (n, 2))
        pts = np.concatenate([new, np.full((n, 1), CHEST_Z)], 1)
        i, inside = win.index(pts)
        i0, inside0 = win.index(np.concatenate([self.xy, np.full((n, 1), CHEST_Z)], 1))
        # a walker does not walk through a trunk: it stays and turns away. A particle already inside an occupied cell (map noise, a ghost of the person
        # itself) is let out instead of being pinned there for ever.
        blocked = (win.occ[i[:, 0], i[:, 1], i[:, 2]] & inside) & ~(
            win.occ[i0[:, 0], i0[:, 1], i0[:, 2]] & inside0
        )
        th2 = th + rng.choice([-1.0, 1.0], n) * rng.uniform(0.6, 1.6, n)
        v_b = sp[:, None] * np.stack([np.cos(th2), np.sin(th2)], 1)
        self.xy = np.where(blocked[:, None], self.xy, new)
        self.v = np.where(blocked[:, None], v_b, v)

    def observe_unseen(self, win, cam_pos, cam_R):
        """No detection this frame: the mass in the camera's detectable volume is mostly not the subject."""
        face = cam_R[:2, 2] / max(np.linalg.norm(cam_R[:2, 2]), 1e-6)
        vis = visible(win, cam_pos[None], self.xy, face[None])[0]
        self.w = self.w * np.where(vis, 1.0 - P_DETECT, 1.0)
        self.w /= self.w.sum()
        if (
            1.0 / (self.w**2).sum() < 0.5 * self.n
        ):  # systematic resampling with a little jitter
            u = (self.rng.random() + np.arange(self.n)) / self.n
            k = np.minimum(np.searchsorted(np.cumsum(self.w), u), self.n - 1)
            self.xy = self.xy[k] + self.rng.normal(0, 0.08, (self.n, 2))
            self.v = self.v[k] + self.rng.normal(0, 0.05, (self.n, 2))
            self.w = np.full(self.n, 1.0 / self.n)

    def estimate(self):
        """-> (mean xy, mean v, sigma m): the weighted mean, velocity and the rms spread about the mean."""
        w = self.w[:, None]
        m0 = (w * self.xy).sum(0)
        sg = float(np.sqrt((w * (self.xy - m0) ** 2).sum()))
        d2 = ((self.xy[:, None, :] - self.xy[None, :, :]) ** 2).sum(2)
        peak = int(
            np.argmax((np.exp(-d2 / (2 * MODE_R**2)) * self.w[None, :]).sum(1))
        )  # the densest cluster, not the (meaningless) mean of a split belief
        near = (d2[peak] < (2 * MODE_R) ** 2)[:, None] * w
        m, v = (near * self.xy).sum(0) / near.sum(), (near * self.v).sum(0) / near.sum()
        pace = (near[:, 0] * np.linalg.norm(self.v, axis=1)).sum() / near.sum()
        v = (
            v * pace / max(float(np.linalg.norm(v)), 1e-6)
        )  # headings diffuse, so the mean velocity shrinks toward 0; a walker's pace does not: keep the pace, take the mean direction
        if self.m_prev is not None:
            m = (
                self.m_prev + EMA * (m - self.m_prev)
            )  # the target must not jump between frames: the planner would reverse with it
            v = self.v_prev + EMA * (v - self.v_prev)
        self.m_prev, self.v_prev = m, v
        return m, v, sg

    def sample(self, m=60):
        """m particles drawn by weight (for the viewpoint scoring)."""
        k = self.rng.choice(self.n, m, p=self.w)
        return self.xy[k], self.v[k]
