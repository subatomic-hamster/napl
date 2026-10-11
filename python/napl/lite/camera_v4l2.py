"""V4L2Camera: the Logitech Brio through OpenCV's V4L2 backend (Apache-2.0 on the Jetson; imported lazily, napl itself needs only numpy).

MJPG 1280x720 @ 60, manual exposure (~1/500 s), autofocus off (spec 5.2). A capture thread keeps only the newest frame; t_cap = monotonic time when the
buffer was dequeued minus a fixed cam_latency (0.030 s [est]). Intrinsics come from ~/.config/cuas-lite/calib.toml (chessboard calibration) or, without
it, the FOV prior (the app then refuses to ARM: AppCfg.calib_ok). NOT exercised on hardware in this repo (no camera on the build host)."""

import threading
import time
import tomllib
from pathlib import Path

from .io import Frame, Intrinsics

CALIB_PATH = Path("~/.config/cuas-lite/calib.toml").expanduser()
CAM_LATENCY = 0.030


def load_calib(w, h, path=CALIB_PATH):
    """-> (Intrinsics, ok): ok when calib.toml exists for this mode."""
    try:
        d = tomllib.loads(Path(path).read_text())
        if (d["w"], d["h"]) != (w, h):
            raise KeyError
        return Intrinsics(
            w,
            h,
            d["fx"],
            d["fy"],
            d["cx"],
            d["cy"],
            tuple(d.get("dist", (0, 0, 0, 0, 0))),
        ), True
    except (FileNotFoundError, KeyError, tomllib.TOMLDecodeError):
        return Intrinsics.from_fov(w, h, 0.911), False


class V4L2Camera:
    def __init__(self, dev="/dev/video0", w=1280, h=720, fps=60):
        import cv2

        self.cv2 = cv2
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        for prop, v in ((cv2.CAP_PROP_FRAME_WIDTH, w), (cv2.CAP_PROP_FRAME_HEIGHT, h), (cv2.CAP_PROP_FPS, fps), (cv2.CAP_PROP_AUTOFOCUS, 0),
                        (cv2.CAP_PROP_AUTO_EXPOSURE, 1), (cv2.CAP_PROP_EXPOSURE, 20), (cv2.CAP_PROP_BUFFERSIZE, 1)):  # fmt: skip
            self.cap.set(prop, v)  # exposure 20 = 2 ms in 100 us units (~1/500 s)
        self.intrinsics, self.calib_ok = load_calib(w, h)
        self._lock, self._frame, self._seq, self._returned, self._stop = (
            threading.Lock(),
            None,
            0,
            0,
            False,
        )
        self._jpeg = None
        threading.Thread(target=self._grab, daemon=True).start()

    def _grab(self):
        cv2 = self.cv2
        while not self._stop:
            ok, bgr = self.cap.read()
            t = time.monotonic() - CAM_LATENCY
            if not ok:
                time.sleep(0.01)
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            with self._lock:
                self._seq += 1
                self._frame = Frame(self._seq, t, rgb.shape[1], rgb.shape[0], rgb)
                self._bgr = bgr

    def read(self, timeout=0.1):
        end = time.monotonic() + timeout
        while True:
            with self._lock:
                if self._frame is not None and self._frame.seq != self._returned:
                    self._returned = self._frame.seq
                    return self._frame
            if time.monotonic() >= end:
                return None
            time.sleep(0.002)

    def jpeg(self, quality=70):
        """Newest frame as JPEG bytes for the web UI's MJPEG stream (None before the first frame)."""
        with self._lock:
            bgr = getattr(self, "_bgr", None)
        if bgr is None:
            return None
        ok, buf = self.cv2.imencode(
            ".jpg", bgr, [self.cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        return buf.tobytes() if ok else None

    def close(self):
        self._stop = True
        self.cap.release()
