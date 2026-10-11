"""hopsample.py VIDEO OUT_DIR N: decode VIDEO and save N evenly spaced frames as PNG (to read them back); prints duration, size, resolution."""

import sys
from pathlib import Path

import av

video, out, n = Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3])
out.mkdir(parents=True, exist_ok=True)
with av.open(str(video)) as c:
    s = c.streams.video[0]
    total = s.frames or int(
        c.duration / 1e6 * float(s.average_rate)
    )  # webm: the stream carries neither
    want = {int((k + 0.5) * total / n) for k in range(n)}
    for i, fr in enumerate(c.decode(video=0)):
        if i in want:
            fr.to_image().save(
                out / f"s{i:05d}_{i / float(s.average_rate):7.1f}s.png".replace(" ", "")
            )
    print(
        f"{video.name}: {total} frames, {total / float(s.average_rate):.1f} s, {s.width}x{s.height}, {s.codec_context.name}, {video.stat().st_size / 1e6:.1f} MB"
    )
