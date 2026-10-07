"""Evidence writer: runs/<id>/{result.json, trace.csv, fail_snapshot.bin (failures only), plot.png (if matplotlib)}.

fail_frame.png needs the M9 renderer and is not written yet.
"""

import csv
import hashlib
import json
import re
from pathlib import Path


def run_id(spec: dict, model) -> str:
    """Deterministic: spec name + hash of (spec, model), so the same test on the same model reuses its directory."""
    h = hashlib.sha256(
        json.dumps([spec, str(model)], sort_keys=True).encode()
    ).hexdigest()[:8]
    return f"{re.sub(r'[^A-Za-z0-9_.-]', '_', spec.get('name') or 'test')}-{h}"


def write_report(result: dict, spec: dict, model, runs_dir="runs") -> Path:
    """`result` is Sim.run_test()'s dict. Returns the run directory."""
    d = Path(runs_dir) / run_id(spec, model)
    d.mkdir(parents=True, exist_ok=True)
    cols, flat = result["trace_cols"], result["trace"]
    rows = [flat[i : i + len(cols)] for i in range(0, len(flat), len(cols))]
    with open(d / "trace.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)
    snap = d / "fail_snapshot.bin"
    if result["fail_snapshot"]:
        snap.write_bytes(result["fail_snapshot"])
    else:
        snap.unlink(missing_ok=True)  # stale file from an earlier run of this id
    out = {
        k: v
        for k, v in result.items()
        if k not in ("trace", "trace_cols", "fail_snapshot")
    }
    out.update(run_id=d.name, model=str(model), spec=spec, trace_rows=len(rows))
    (d / "result.json").write_text(json.dumps(out, indent=2))
    _plot(d, cols, rows)
    return d


def _plot(d: Path, cols, rows) -> None:
    try:  # optional dependency: never required
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots()
    for i, c in enumerate(cols[2:], start=2):
        ax.plot([r[1] for r in rows], [r[i] for r in rows], label=c)
    ax.set_xlabel("t [s]")
    ax.legend()
    fig.savefig(d / "plot.png")
    plt.close(fig)
