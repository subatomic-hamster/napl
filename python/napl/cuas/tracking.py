"""Vision for the turret: sky detection, blob extraction, multi-target tracking with stable IDs, range from apparent size.

Pure numpy, no scene knowledge: the input is a camera frame (uint8 rgb) and its pose; the output is tracks with a pixel box, a range
estimated from the box size, a world position estimated from (pixel ray x range) and a filtered velocity.

    det = detect(rgb, horizon_row)                    # [Blob(bbox, centroid, area)]: things that are not sky, above the horizon
    tr = Tracker(cam)                                 # IDs "A", "B", ... never reused
    tracks = tr.update(det, frame)                    # confirmed + coasting tracks, nearest-neighbour association in pixel space

Detection. The sky is not flat (gradient, warm horizon band, haze, clouds), so its colour is estimated per pixel as the local mean over a
SKY_WINDOW_PX window (an integral image); smooth sky structure cancels, a small dark drone does not. Foreground = pixels darker than
that local sky by more than `thr` (sum of RGB) AND below DARK_RATIO of it: drones are dark silhouettes against the sky, while the blue next
to a bright cloud is only slightly darker than its cloud-lifted local mean, above the horizon row (computed from the camera pitch, minus a margin) and outside the static mask of the turret's own barrel,
which the camera sees at the bottom of the image. Foreground pixels are dilated by 2 px so that thin props / arms join their body, then
labelled (4-connected, a small flood fill: foreground is a few hundred pixels), boxes taken from the undilated pixels.

Association. The camera turns (pan / tilt), so a target moves across the image even when it is still: each track's world position (+ velocity)
is projected into the CURRENT frame with the camera pose, and detections are matched to those predicted pixels (nearest first, gated).

Range. A drone's apparent size s px (largest box side) at range r: s = f_px * D / r with f_px the camera focal length in pixels and D an
ASSUMED characteristic size (`ASSUMED_SIZE_M`, a program constant: the turret does not know the drone model; D is tuned for ~0.5 m
class quads). Range error is therefore tens of percent for a drone of another size and a few percent from pixel quantisation; it is reported
as is. Bearing (the pixel ray) is far better than range, so the world-position filter is loose along the line of sight and tight across it.
"""

from dataclasses import dataclass, field

import numpy as np

ASSUMED_SIZE_M = 0.55  # m: characteristic size (the largest box side) of a small quad, for range from apparent size
SKY_THR = 18  # sum of RGB below the local sky's
DARK_RATIO = 0.75  # and darker than this fraction of it: a drone is a dark silhouette, a cloud edge only a little darker than its mean
SKY_WINDOW_PX = 61  # local sky = mean over this window: follows the gradient, haze and soft clouds, not a drone (a few to tens of px)
HORIZON_MARGIN_PX = 6
BARREL_MASK = (
    0.43,
    0.66,
    0.57,
)  # the muzzle in the image (camera-fixed): columns x0..x1 and rows from y0, as fractions of w / h


@dataclass
class Blob:
    bbox: tuple  # (x0, y0, x1, y1) inclusive pixel box
    centroid: tuple
    area: int

    @property
    def size(self):
        return max(self.bbox[2] - self.bbox[0] + 1, self.bbox[3] - self.bbox[1] + 1)


def horizon_row(cam, fr):
    """Image row of the horizon (level direction) for the camera pose; may lie outside the image."""
    el = np.arcsin(np.clip(fr.fwd[2], -1, 1))
    return cam.h / 2 + cam.f_px * np.tan(el)


def _local_mean(img, k):
    """Per-pixel mean over a k x k window (edge-padded), all channels: an integral image, O(pixels)."""
    r = k // 2
    p = np.pad(img, ((r + 1, r), (r + 1, r), (0, 0)), mode="edge").cumsum(0).cumsum(1)
    h, w = img.shape[:2]
    return (p[k:k + h, k:k + w] - p[:h, k:k + w] - p[k:k + h, :w] + p[:h, :w]) / (k * k)  # fmt: skip


def detect(rgb, horizon, thr=SKY_THR, min_area=8, mask=BARREL_MASK):
    h, w, _ = rgb.shape
    top = int(min(h, max(0, horizon - HORIZON_MARGIN_PX)))
    if top < 8:
        return []
    lum = rgb[:top].astype(np.float32).sum(axis=2)
    sky = _local_mean(lum[..., None], SKY_WINDOW_PX)[..., 0]
    fg = (sky - lum > thr) & (lum < DARK_RATIO * sky)
    if mask:
        fg[int(h * mask[1]) :, int(w * mask[0]) : int(w * mask[2])] = False
    if (
        not fg.any() or fg.sum() > 60000
    ):  # nothing, or a whole-frame change (a wall / the ground in view): nothing to track
        return []
    grow = np.zeros_like(fg)
    for dy in (-2, -1, 0, 1, 2):
        for dx in (-2, -1, 0, 1, 2):
            grow[max(dy, 0) : top + min(dy, 0), max(dx, 0) : w + min(dx, 0)] |= fg[
                max(-dy, 0) : top + min(-dy, 0), max(-dx, 0) : w + min(-dx, 0)
            ]
    seen = np.zeros(fg.shape, bool)
    out = []
    for y0, x0 in zip(*np.nonzero(grow)):
        if seen[y0, x0]:
            continue
        stack, comp = [(y0, x0)], []
        seen[y0, x0] = True
        while stack:
            y, x = stack.pop()
            comp.append((y, x))
            for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                if 0 <= ny < top and 0 <= nx < w and grow[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    stack.append((ny, nx))
        c = np.array(comp)
        c = c[fg[c[:, 0], c[:, 1]]]  # undilated pixels only
        if len(c) < min_area:
            continue
        out.append(Blob((int(c[:, 1].min()), int(c[:, 0].min()), int(c[:, 1].max()), int(c[:, 0].max())),
                        (float(c[:, 1].mean()), float(c[:, 0].mean())), len(c)))  # fmt: skip
    return out


@dataclass
class Track:
    id: str
    bbox: tuple
    uv: np.ndarray  # pixel centroid of the last detection
    hits: int = 1
    misses: int = 0
    range_m: float = 0.0
    pos: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # world position, estimated
    vel: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # world velocity, estimated (alpha-beta)
    t: float = 0.0
    confirmed: bool = False
    sizes: list = field(default_factory=list)


class Tracker:
    """Nearest-neighbour multi-target tracker. IDs are letters A..Z then AA.. in order of appearance, never reused. World state per track:
    position from (pixel ray x range from size), velocity by an alpha-beta filter whose gains are (lateral, along the line of sight)."""

    def __init__(
        self,
        cam,
        gate_px=45.0,
        confirm=3,
        max_miss=10,
        gains=(0.6, 0.12, 0.15, 0.01),
        size_m=ASSUMED_SIZE_M,
    ):
        self.cam, self.gate, self.confirm, self.max_miss = (
            cam,
            gate_px,
            confirm,
            max_miss,
        )
        self.gains, self.size_m = gains, size_m
        self.tracks: list = []
        self.n = 0
        self.t = None

    def _new_id(self):
        n, s = self.n, ""
        self.n += 1
        while True:
            s = chr(65 + n % 26) + s
            n = n // 26 - 1
            if n < 0:
                return s

    def _predict(self, tr, dt, fr):
        tr.pos = tr.pos + tr.vel * dt
        p = self.cam.project(tr.pos, fr)
        return np.array(p[:2]) if p else tr.uv

    def update(self, blobs, fr):
        dt = 0.0 if self.t is None else fr.t - self.t
        self.t = fr.t
        pred = [self._predict(tr, dt, fr) for tr in self.tracks]
        pairs = sorted((d, i, j) for i, b in enumerate(blobs) for j in range(len(self.tracks))
                       if (d := float(np.hypot(*(np.array(b.centroid) - pred[j])))) < self.gate)  # fmt: skip
        used_b, used_t = set(), set()
        for _, i, j in pairs:
            if i in used_b or j in used_t:
                continue
            used_b.add(i)
            used_t.add(j)
            self._correct(self.tracks[j], blobs[i], dt, fr)
        for j, tr in enumerate(self.tracks):
            if j not in used_t:
                tr.misses += 1
                tr.uv = pred[j]
        for i, b in enumerate(blobs):
            if i not in used_b:
                tr = Track(self._new_id(), b.bbox, np.array(b.centroid), t=fr.t)
                tr.pos = self._measure(tr, b, fr)[0]
                self.tracks.append(tr)
        self.tracks = [t for t in self.tracks if t.misses <= self.max_miss]
        for t in self.tracks:
            t.confirmed = t.hits >= self.confirm
        return list(self.tracks)

    def _measure(self, tr, b, fr):
        """-> (measured world position, unit line of sight); also updates the box / range."""
        tr.sizes = (tr.sizes + [b.size])[-5:]
        tr.range_m = self.cam.f_px * self.size_m / max(float(np.median(tr.sizes)), 1.0)
        tr.uv, tr.bbox, tr.t = np.array(b.centroid), b.bbox, fr.t
        los = self.cam.ray(b.centroid[0], b.centroid[1], fr)
        return fr.pos + los * tr.range_m, los

    def _correct(self, tr, b, dt, fr):
        z, los = self._measure(tr, b, fr)
        tr.hits += 1
        tr.misses = 0
        if (
            tr.hits == 2 and not tr.vel.any()
        ):  # first re-detection: position only (a velocity from 2 noisy points is worse than none)
            tr.pos = z
            return
        res = z - tr.pos  # tr.pos is already predicted to this frame
        r_los = (res @ los) * los
        r_lat = res - r_los
        al, bl, ar, br = self.gains
        tr.pos = tr.pos + al * r_lat + ar * r_los
        if dt > 0:
            tr.vel = tr.vel + (bl * r_lat + br * r_los) / dt
