import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_bench_prints_one_row_per_kind_and_patch():
    out = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "bench_autonomy.py"), "--kinds", "ruins", "--rays", "2500", "--patch", "10,5", "--sim", "0.5"],
        capture_output=True, text=True, check=True, cwd=ROOT,
    ).stdout  # fmt: skip
    rows = [l for l in out.splitlines() if l.startswith("| ruins |")]
    assert len(rows) == 2
    for r in rows:
        assert float(re.split(r"\s*\|\s*", r.strip("| "))[-1]) > 0
