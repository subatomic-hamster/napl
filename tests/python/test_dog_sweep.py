"""Speed sweep harness (tools/dog_sweep.py): variant files load for every default point; the report labels every column and claims no
gait speed it did not measure (Review Focus 5); two runs give the same numbers."""

import json
import subprocess
import sys
from pathlib import Path

import napl
import pytest
from napl.design import assembly as A

REPO = Path(__file__).resolve().parents[2]
TOOL = REPO / "tools" / "dog_sweep.py"
sys.path.insert(0, str(REPO / "tools"))
import dog_sweep as S

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.mark.parametrize("point", S.DEFAULT_POINTS)
def test_variant_files_load(tmp_path, point):
    act, ratio, L = point
    asm = A.load(S.write_variant(tmp_path, act, ratio, L))
    by = {i["id"]: i for i in asm.doc["instances"]}
    assert by["fl:knee_motor"]["params"]["drive_ratio"]["value"] == pytest.approx(
        8 * ratio
    )
    assert by["rr:thigh"]["part"] == "dog-thigh-bracket-l" + S.LSFX[L]
    probe = [d for d in asm.doc["devices"] if d["type"] == "qdd" and "map" not in d]
    assert bool(probe) == (
        act == "gim6010"
    )  # the probe carries the GIM6010-8 electrics as provenance-tagged `set`


@needs_kernel
def test_report_labels_every_column_and_is_deterministic(tmp_path):  # Review Focus 5
    outs = []
    for k in range(2):
        out = tmp_path / f"s{k}"
        r = subprocess.run([sys.executable, str(TOOL), "--quick", "--points", "gim8108:1.5:0.22", "--out", str(out), "--jobs", "1"],
                           capture_output=True, text=True, timeout=3000)  # fmt: skip
        assert r.returncode == 0, r.stderr[-3000:]
        outs.append(json.loads((out / "speed-sweep.json").read_text()))
    data = outs[0]
    assert set(data["columns"]) == set(S.COLUMNS) and set(S.COLUMNS.values()) <= {
        "measured",
        "derived",
        "bom",
        "sp4",
    }
    assert S.COLUMNS["v_top_trot"] == "sp4" and all(
        p["v_top_trot"] is None for p in data["points"]
    )
    assert all(
        S.COLUMNS[c] != "measured" for c in ("v_swing", "v_stance", "v_trot_bound")
    )
    md = (tmp_path / "s0" / "speed-sweep.md").read_text()
    assert (
        "not measured (SP4)" in md
        and "derived (study formulas, sim-measured inputs)" in md
    )
    assert outs[0]["points"] == outs[1]["points"]
