"""Shared video helper for the record_* tools: the frame store (capture stage) and the encoder (encode stage).

Frame store. The capture stages used to write one PNG per grabbed frame (CPU deflate + a PNG decode in the encode stage). They
now save the viewer's raw RGBA frame (`_napl.viewer_frame`) as `f00000.rgz` = 8-byte header (w, h, uint32 LE) + zlib level 1 of
the pixels, and the encode stages read it back with load_frame(). PNG files stay readable (frames the sim itself wrote, older
captures) and are also written next to the .rgz when NAPL_KEEP_FRAMES=1 (the tools' --keep-frames).

Encoder. Frames are streamed (raw RGBA on a pipe) to the first backend that passes a short probe encode:
  default   GStreamer VA (LGPL): gst-launch-1.0 fdsrc ! rawvideoparse ! vapostproc ! vaav1enc ! av1parse ! webmmux   (.webm)
                                                                            ... vah264enc ! h264parse ! mp4mux     (.mp4)
  --gpl     ffmpeg VAAPI (av1_vaapi .webm / h264_vaapi .mp4) via rawvideo on stdin, -vaapi_device /dev/dri/renderD128
            (NAPL_VAAPI_DEVICE overrides; on the dev box renderD128 is the discrete RX 9070 XT, renderD129 the iGPU)
  fallback  PyAV software encode (libvpx-vp9 .webm; libopenh264 .mp4, libx264 with --gpl): what the tools did before.
NAPL_VIDEO_BACKEND=gst|ffmpeg|pyav forces one (no probe fallback); the chosen backend is printed.
"""

import os
import shutil
import struct
import subprocess
import tempfile
import zlib
from pathlib import Path

import numpy as np

KEEP = lambda: os.environ.get("NAPL_KEEP_FRAMES") == "1"
VAAPI_DEVICE = lambda: os.environ.get("NAPL_VAAPI_DEVICE", "/dev/dri/renderD128")


def keep_from(args):
    """--keep-frames -> NAPL_KEEP_FRAMES=1 (inherited by the xvfb child the capture runs in)."""
    if getattr(args, "keep_frames", False):
        os.environ["NAPL_KEEP_FRAMES"] = "1"


# ---- frame store --------------------------------------------------------------------------------------------------------------------
def save_frame(d, stem, rgba):
    """Saves one (h, w, 4) uint8 RGBA frame as D/STEM.rgz (and D/STEM.png with NAPL_KEEP_FRAMES=1). STEM e.g. "f00012"."""
    a = np.asarray(rgba, np.uint8)
    if a.shape[2] == 3:
        a = np.dstack([a, np.full(a.shape[:2], 255, np.uint8)])
    a = np.ascontiguousarray(a)
    h, w = a.shape[:2]
    (Path(d) / f"{stem}.rgz").write_bytes(
        struct.pack("<II", w, h) + zlib.compress(a.tobytes(), 1)
    )
    if KEEP():
        from napl import _napl

        (Path(d) / f"{stem}.png").write_bytes(_napl.encode_png(a))


def load_array(d, stem):
    """(h, w, 4) uint8 RGBA of D/STEM.rgz, else of D/STEM.png (needs Pillow for the latter)."""
    p = Path(d) / f"{stem}.rgz"
    if p.exists():
        b = p.read_bytes()
        w, h = struct.unpack("<II", b[:8])
        return np.frombuffer(zlib.decompress(b[8:]), np.uint8).reshape(h, w, 4)
    from PIL import Image

    return np.asarray(Image.open(Path(d) / f"{stem}.png").convert("RGBA"))


def load_frame(d, stem):
    """Like load_array, as an RGB PIL image (what the encode stages composite captions onto)."""
    from PIL import Image

    p = Path(d) / f"{stem}.rgz"
    if p.exists():
        return Image.fromarray(load_array(d, stem)).convert("RGB")
    return Image.open(Path(d) / f"{stem}.png").convert("RGB")


def frame_stems(d, pattern="f*"):
    """Sorted stems of the frames in D (.rgz or .png; a frame saved both ways counts once)."""
    return sorted(
        {p.stem for p in Path(d).glob(pattern + ".rgz")}
        | {p.stem for p in Path(d).glob(pattern + ".png")}
    )


# ---- encoder ------------------------------------------------------------------------------------------------------------------------
def _gst_ok():
    return bool(shutil.which("gst-launch-1.0") and shutil.which("gst-inspect-1.0"))


def _gst_has(element):
    return (
        subprocess.run(["gst-inspect-1.0", element], capture_output=True).returncode
        == 0
    )


def _gst_cmd(out, w, h, fps, av1):
    enc = "vaav1enc ! av1parse ! webmmux" if av1 else "vah264enc ! h264parse ! mp4mux"
    desc = (
        f"fdsrc fd=0 ! rawvideoparse format=rgba width={w} height={h} framerate={int(fps)}/1 ! "
        f"videoconvert ! vapostproc ! {enc} ! filesink location={out}"
    )
    return ["gst-launch-1.0", "-q"] + desc.split()


def _ffmpeg_cmd(out, w, h, fps, av1):
    codec = "av1_vaapi" if av1 else "h264_vaapi"
    return [
        "ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
        "-vaapi_device", VAAPI_DEVICE(), "-vf", "format=nv12,hwupload", "-c:v", codec, "-b:v", "6M", str(out),
    ]  # fmt: skip


def _candidates(suffix, gpl):
    av1 = suffix == ".webm"
    gst = (
        ("gst", _gst_cmd, av1)
        if _gst_ok() and _gst_has("vaav1enc" if av1 else "vah264enc")
        else None
    )
    ff = ("ffmpeg", _ffmpeg_cmd, av1) if shutil.which("ffmpeg") else None
    order = (
        [ff, gst] if gpl else [gst]
    )  # ffmpeg (GPL build possible) only when asked for
    return [c for c in order if c]


def _probe(maker, av1, suffix):
    """A 5-frame 320x240 encode through the backend: it must exit 0 and leave a non-empty file."""
    out = Path(tempfile.mkdtemp(prefix="napl-enc-")) / f"probe{suffix}"
    try:
        p = subprocess.Popen(
            maker(out, 320, 240, 10, av1), stdin=subprocess.PIPE, stderr=subprocess.PIPE
        )
        frame = (
            (np.arange(320 * 240 * 4, dtype=np.uint32) % 251).astype(np.uint8).tobytes()
        )
        for _ in range(5):
            p.stdin.write(frame)
        p.stdin.close()
        err = p.stderr.read().decode(errors="replace")
        ok = p.wait(timeout=60) == 0 and out.exists() and out.stat().st_size > 0
        return ok, err.strip()[-300:]
    except Exception as e:  # noqa: BLE001 (a missing / hung tool is just "not usable")
        return False, repr(e)
    finally:
        shutil.rmtree(out.parent, ignore_errors=True)


class Encoder:
    """Encoder(out, w, h, fps, gpl=False): .write(img) per frame (PIL image, or (h, w, 3|4) uint8 array), .close(). .backend names
    what is encoding; the container follows the suffix of `out` (.webm AV1 / VP9, .mp4 H.264)."""

    def __init__(self, out, w, h, fps, gpl=False):
        self.out, self.w, self.h, self.fps, self.n = Path(out), w, h, fps, 0
        self.proc = self.cont = self.st = None
        want = os.environ.get("NAPL_VIDEO_BACKEND", "")
        suffix = self.out.suffix
        cands = [c for c in _candidates(suffix, gpl) if not want or c[0] == want]
        self.backend = None
        for name, maker, av1 in cands:
            ok, err = (True, "") if want else _probe(maker, av1, suffix)
            if ok:
                self.backend = name + ("-va" if name == "gst" else "-vaapi")
                self.proc = subprocess.Popen(
                    maker(self.out, w, h, fps, av1), stdin=subprocess.PIPE
                )
                break
            print(f"videnc: {name} VA encode unusable here ({err or 'probe failed'})")
        if self.proc is None:
            if want and want != "pyav":
                raise SystemExit(f"NAPL_VIDEO_BACKEND={want}: not available")
            self._open_pyav(gpl)
        print(f"videnc: encoding {self.out.name} {w}x{h}@{fps} with {self.backend}")

    def _open_pyav(self, gpl):
        import av

        names = (
            ["libvpx-vp9"] if self.out.suffix == ".webm" else ["libx264", "libopenh264"]
        )
        codec = next(
            (c for c in names if c in av.codecs_available and (c != "libx264" or gpl)),
            None,
        )
        if codec is None:
            raise SystemExit(
                f"no software encoder for {self.out.suffix} in this PyAV (libx264 is GPL: --gpl)"
            )
        self.av = av
        self.cont = av.open(str(self.out), "w")
        self.st = self.cont.add_stream(codec, rate=self.fps)
        self.st.width, self.st.height, self.st.pix_fmt = self.w, self.h, "yuv420p"
        self.st.options = (
            {"crf": "32", "b": "0", "cpu-used": "2", "row-mt": "1"}
            if codec == "libvpx-vp9"
            else {}
        )
        self.backend = f"pyav-{codec}"

    def write(self, img):
        self.n += 1
        if self.proc is not None:
            a = img if isinstance(img, np.ndarray) else np.asarray(img.convert("RGBA"))
            if a.shape[2] == 3:
                a = np.dstack([a, np.full(a.shape[:2], 255, np.uint8)])
            assert a.shape[:2] == (self.h, self.w), (
                f"frame {a.shape[:2]} != video {(self.h, self.w)}"
            )
            self.proc.stdin.write(np.ascontiguousarray(a).tobytes())
            return
        if isinstance(img, np.ndarray):
            frame = self.av.VideoFrame.from_ndarray(
                np.ascontiguousarray(img[..., :3]), format="rgb24"
            )
        else:
            frame = self.av.VideoFrame.from_image(img.convert("RGB"))
        for pk in self.st.encode(frame):
            self.cont.mux(pk)

    def close(self):
        if self.proc is not None:
            self.proc.stdin.close()
            rc = self.proc.wait()
            if rc != 0:
                raise SystemExit(f"{self.backend} encoder exited {rc}")
            return
        for pk in self.st.encode():
            self.cont.mux(pk)
        self.cont.close()
