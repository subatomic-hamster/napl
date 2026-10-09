"""Person detection on the onboard depth image, and the track it feeds. numpy only.

The detector sees NOTHING but the depth frame, the camera intrinsics and the vehicle's pose ESTIMATE; the subject's true position never enters.
Classical pipeline (no network): back-project the frame to world points with the estimate, drop the ground (z < GROUND_Z), label connected
components of the remaining pixels (4-neighbours, depth step below 5 % of range + 0.12 m), and keep the components whose metric size fits a standing
person: top between 1.25 and 2.1 m, reaching down to within 0.9 m of the ground (2nd-percentile height) with at least MIN_SPAN = 0.9 m of visible
height, 0.25-1.1 m wide, shallow in depth, not touching the top of the image
(a trunk runs off it), with background (far or no return) above its top. Trunks, poles, the arch and walls fail the height / width test; boulders
fail the height test. It is a size-prior detector: anything else person-sized (a stump, a post) would pass, and a person occluded below the
waist is missed. `Detection` carries the pixel box (for the HUD and the map mask), the range, and the body-centre position in the world frame.

    det = PersonDetector(cam)
    dets = det.detect(depth, cam_pos, cam_R)     # [dict(row, col, half_h, half_w, range, score, xy, z_top, width)]
    trk = Tracker(); est = trk.update(t, dets)   # (xy, vxy, seen, sigma_xy) | None
"""

import numpy as np

GROUND_Z = 0.35  # m: depth hits below this height (estimated frame) are ground
TOP = (1.25, 2.1)  # m: admissible height of the component's top
WIDTH = (0.25, 1.1)  # m
BODY_R = 0.2  # m: torso radius: the body centre is this far behind the surface the camera sees
MIN_PIX = 8
MIN_SPAN = 0.9  # m: visible vertical extent (the camera's bottom edge cuts off the legs of a person closer than ~6 m)


def label(adj_h, adj_v):
    """Connected-component labels of a pixel grid from edge masks (adj_h[i, j]: pixel (i, j) joined to (i, j+1); adj_v[i, j]: to (i+1, j)). Min-label
    propagation with pointer jumping."""
    n_v, n_h = adj_v.shape[0] + 1, adj_h.shape[1] + 1
    lab = np.arange(n_v * n_h).reshape(n_v, n_h)
    for _ in range(64):
        new = lab.copy()
        new[:, :-1] = np.where(adj_h, np.minimum(new[:, :-1], lab[:, 1:]), new[:, :-1])
        new[:, 1:] = np.where(adj_h, np.minimum(new[:, 1:], lab[:, :-1]), new[:, 1:])
        new[:-1, :] = np.where(adj_v, np.minimum(new[:-1, :], lab[1:, :]), new[:-1, :])
        new[1:, :] = np.where(adj_v, np.minimum(new[1:, :], lab[:-1, :]), new[1:, :])
        new = new.ravel()[new.ravel()]  # pointer jumping
        new = new.reshape(n_v, n_h)
        if np.array_equal(new, lab):
            break
        lab = new
    return lab


class PersonDetector:
    def __init__(self, cam):
        self.cam = cam
        self.tang = cam.tangents()
        self.ray_len = np.linalg.norm(
            self.tang, axis=-1
        )  # ray length per metre of depth (the device's range limit is on the ray)
        self.px_w = (
            2 * np.tan(cam.fov_h / 2) / cam.n_h
        )  # metres per pixel per metre of range

    def detect(self, depth, cam_pos, cam_R):
        c = self.cam
        d = np.asarray(depth, np.float32)
        ok = np.isfinite(d) & (d >= c.min_range)
        pts = cam_pos + (self.tang * np.where(ok, d, 0.0)[..., None]) @ cam_R.T
        ok &= pts[..., 2] > GROUND_Z
        dd = np.where(ok, d, 0.0)
        step = lambda a, b: np.abs(a - b) < 0.05 * np.minimum(a, b) + 0.12
        adj_h = ok[:, :-1] & ok[:, 1:] & step(dd[:, :-1], dd[:, 1:])
        adj_v = ok[:-1, :] & ok[1:, :] & step(dd[:-1, :], dd[1:, :])
        lab = np.where(ok, label(adj_h, adj_v), -1)
        ids, counts = np.unique(lab[ok], return_counts=True)
        out = []
        for i, n in zip(ids, counts):
            if n < MIN_PIX:
                continue
            rr, cc = np.nonzero(lab == i)
            r0, r1, c0, c1 = rr.min(), rr.max(), cc.min(), cc.max()
            if r0 == 0 or c0 == 0 or c1 == c.n_h - 1:
                continue  # runs off the top of the image (a trunk, pole or wall) or off its side (only part of it is seen)
            rng = float(np.median(dd[rr, cc]))
            z = pts[rr, cc, 2]
            width = (c1 - c0 + 1) * rng * self.px_w
            z_top, z_low = float(np.percentile(z, 98)), float(np.percentile(z, 2))
            if not (
                TOP[0] <= z_top <= TOP[1]
                and z_low <= 0.9
                and z_top - z_low >= MIN_SPAN
                and WIDTH[0] <= width <= WIDTH[1]
                and np.ptp(dd[rr, cc]) < 1.0
            ):
                continue
            if rng * self.ray_len[r0 - 1, c0 : c1 + 1].max() > 0.92 * c.max_range:
                continue  # the rays just above would be out of range anyway: "no return" there is not a silhouette (a distant trunk looks like a person)
            above = dd[max(r0 - 3, 0) : r0, c0 : c1 + 1]
            if (
                np.abs(above - rng) < 0.5
            ).mean() > 0.3:  # something at the same range continues above: not a head
                continue
            ray = cam_pos[:2] - np.median(pts[rr, cc, :2], axis=0)
            xy = np.median(pts[rr, cc, :2], axis=0) - BODY_R * ray / max(
                np.linalg.norm(ray), 1e-6
            )
            fill = n / ((r1 - r0 + 1) * (c1 - c0 + 1))
            score = float(fill - abs(z_top - 1.75) - abs(width - 0.6) / 2)
            out.append(dict(row=float(rr.mean()), col=float(cc.mean()), half_h=(r1 - r0 + 1) / 2, half_w=(c1 - c0 + 1) / 2, range=rng, score=score,
                            xy=xy, z_top=z_top, width=float(width)))  # fmt: skip
        return out


class Tracker:
    """Constant-velocity Kalman filter on the ground-plane position (state x y vx vy) fed by the detections; picks the detection that best fits the
    prediction (Mahalanobis gate) or, with no track, the best-scoring one; coasts on its velocity for `coast_s` seconds without one."""

    def __init__(self, accel_sigma=1.2, coast_s=10.0, gate=3.5, keep_s=2.0):
        self.keep_s = keep_s
        self.q, self.coast, self.gate = accel_sigma, coast_s, gate
        self.x = self.P = None
        self.t = self.t_seen = self.last_xy = (
            None  # last_xy: where a detection last put the subject
        )

    def _predict(self, t):
        dt = max(t - self.t, 0.0)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        Q = np.zeros((4, 4))
        for k in (0, 1):
            Q[k, k], Q[k, k + 2], Q[k + 2, k], Q[k + 2, k + 2] = (
                dt**4 / 4,
                dt**3 / 2,
                dt**3 / 2,
                dt**2,
            )
        self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + self.q**2 * Q, t

    def update(self, t, dets):
        """-> (xy, vxy, seen, sigma_xy, chosen detection or None) or None while there is no track."""
        chosen = None
        if self.x is not None:
            self._predict(t)
            if t - self.t_seen > self.coast:
                self.x = None
        best = np.inf
        for d in dets:
            sig = (
                0.15 + 0.03 * d["range"]
            )  # blob-centre ambiguity + range noise (2 % of range) + pose error
            Rm = np.eye(2) * sig**2
            if self.x is None:
                m = -d["score"]
            else:
                S = self.P[:2, :2] + Rm
                r = d["xy"] - self.x[:2]
                m = float(r @ np.linalg.solve(S, r))
                if m > self.gate**2:
                    continue
            if m < best:
                best, chosen, chosen_R = m, d, Rm
        if chosen is not None:
            z = chosen["xy"]
            if self.x is None:
                self.x, self.P, self.t = (
                    np.array([z[0], z[1], 0.0, 0.0]),
                    np.diag([chosen_R[0, 0], chosen_R[0, 0], 4.0, 4.0]),
                    t,
                )
            else:
                H = np.zeros((2, 4))
                H[0, 0] = H[1, 1] = 1.0
                S = H @ self.P @ H.T + chosen_R
                K = self.P @ H.T @ np.linalg.inv(S)
                self.x = self.x + K @ (z - H @ self.x)
                self.P = (np.eye(4) - K @ H) @ self.P
            self.t_seen, self.last_xy = t, chosen["xy"].copy()
        if self.x is None:
            return None
        if chosen is None:
            if (
                t - self.t_seen > self.keep_s
            ):  # a walker keeps walking: the velocity is held for keep_s, then fades (half-life ~1.4 s at 25 Hz)
                self.x[2:] *= 0.98
        return (
            self.x[:2].copy(),
            self.x[2:].copy(),
            chosen is not None,
            float(np.sqrt(self.P[0, 0])),
            chosen,
        )
