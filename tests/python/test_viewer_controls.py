"""SP1-ui tasks 1-2: the controls icon + popover in the top bar (the BUILD overlay and the WIRING CONTROLS frame are gone), and the
status-bar DRC message that must follow the DRC chip. The window reports what it shows with the backend op `ui.report`
(`Backend.log` keeps every report in order); these tests read those reports. GPU lane: `xvfb-run -a pytest -m gpu`."""

import json
import os
from pathlib import Path

import napl
import pytest
from napl.design import Design
from napl.design.viewer import Backend

REPO = Path(__file__).resolve().parents[2]
X500 = REPO / "assemblies" / "x500v2.toml"
needs_display = pytest.mark.skipif(
    not os.environ.get("DISPLAY"), reason="needs a display: run under xvfb-run"
)
# the first row of each page's popover (viewer.cpp key_rows): a popover that shows another page's first row is stale
FIRST_ROW = {
    "build": "drag | orbit",
    "library": "drag card | drop in the 3D view",
    "wiring": "drag pin | wire to a pin",
    "simulate": "drag | orbit",
    "program": "Ctrl+P | open a file",
}


def ask(b, tool, args=None):
    return json.loads(b(json.dumps({"tool": tool, "args": args or {}})))


def reports(b):
    return [q["args"]["layout"] for q, _ in b.log if q["tool"] == "ui.report"]


def run(tmp_path, script, frames, width=1280, height=720, api=None):
    d = Design.open(X500)
    b = Backend(lambda: d, tmp_path)
    sim = napl.Sim.load(b.state(d)["xml"])
    sim.show(frames=frames, script=script, width=width, height=height, design=api or b)
    return b, d


def test_ui_keys_op_toggles_and_queues_an_event(tmp_path):
    d = Design.open(X500)
    b = Backend(lambda: d, tmp_path)
    ask(b, "ui_poll")  # a window is attached
    assert ask(b, "ui.keys")["result"] == {"open": True}
    assert ask(b, "ui.keys", {"open": False})["result"] == {"open": False}
    assert ask(b, "ui.state")["result"]["keys"] is False
    ev = ask(b, "ui_poll")["result"]["events"]
    assert [(e["cmd"], e["open"]) for e in ev] == [("keys", True), ("keys", False)]
    assert ask(b, "ui.report", {"layout": {"page": "build"}})["ok"]
    assert ask(b, "ui.state")["result"]["layout"] == {"page": "build"}


@needs_display
@pytest.mark.gpu
def test_popover_replaces_the_build_overlay_and_follows_the_page(tmp_path):
    script = "10 mode build; 20 uikeys off; 30 uikeys on; 80 mode wiring; 140 mode program; 200 mode library; 260 mode simulate"
    b, _ = run(tmp_path, script, 420)
    reps = reports(b)
    assert reps, "the window reported nothing (ui.report)"
    # BUILD frame before the popover opens: the icon is in the top bar, no CONTROLS window (##vpkeys), no popover
    closed = next(r for r in reps if r["page"] == "build" and not r["keys"])
    assert closed["icon"][2] > closed["icon"][0], "no ##qkeys icon in the top bar"
    assert "##vpkeys" not in closed["windows"] and "##keyspop" not in closed["windows"]
    assert all("##vpkeys" not in r["windows"] for r in reps)
    # open: the rows of the page that is up -- never another page's (Review Focus 1)
    seen = {}
    for r in reps:
        if r["keys"] and r["rows"]:
            assert r["rows"][0] == FIRST_ROW[r["page"]], (r["page"], r["rows"][0])
            assert "##keyspop" in r["windows"]
            seen[r["page"]] = r["rows"]
    assert set(seen) == set(FIRST_ROW), (
        f"popover never shown on {set(FIRST_ROW) - set(seen)}"
    )
    # PROGRAM rows are the chords the IDE binds (program_ui.cpp), WIRING the canvas bindings
    assert "Ctrl+Shift+F | find in files" in seen["program"]
    assert "Ctrl+` | terminal" in seen["program"]
    assert "middle drag | pan" in seen["wiring"]


@needs_display
@pytest.mark.gpu
def test_narrow_window_icon_clears_the_tabs_and_the_wiring_zoom(tmp_path):
    b, _ = run(tmp_path, "10 mode wiring; 20 uikeys on", 300, width=900, height=600)
    r = [x for x in reports(b) if x["page"] == "wiring"][-1]
    assert r["size"][0] == 900

    def disjoint(a, c):
        return a[2] <= c[0] or c[2] <= a[0] or a[3] <= c[1] or c[3] <= a[1]

    assert r["icon"][2] > r["icon"][0]
    assert r["zoom"][2] > r["zoom"][0], "the Wiring zoom label was not shown"
    assert disjoint(r["icon"], r["zoom"]) and disjoint(r["icon"], r["tabs"])
    assert disjoint(r["zoom"], r["tabs"])
    # the popover hangs under the icon inside the window
    assert r["keys"] and r["popover"][0] >= 0 and r["popover"][2] <= 900


@needs_display
@pytest.mark.gpu
def test_status_drc_message_follows_the_chip_after_a_foreign_edit(tmp_path):
    """Task 2. The window removes the PDB (status: "DRC E errors, ... (before ...)"); then another client undoes that edit and the
    window re-reads the design (dsync). The chip changes; the message must not keep the old counts."""
    d = Design.open(X500)
    b = Backend(lambda: d, tmp_path)
    state = {"edited": False, "foreign": False}

    def api(text):
        tool = json.loads(text)["tool"]
        if tool == "state" and state["edited"] and not state["foreign"]:
            state["foreign"] = True
            d.undo()  # the other client's edit, between the window's requests
        r = b(text)
        if tool == "design_edit":
            state["edited"] = True
        return r

    sim = napl.Sim.load(b.state(d)["xml"])
    sim.show(
        frames=420,
        script="5 dremove pdb; 300 dsync",
        width=1280,
        height=720,
        design=api,
    )
    assert state["foreign"], "the scenario did not run"
    reps = reports(b)
    chips = [(r["chip"]["errors"], r["chip"]["warnings"]) for r in reps]
    shown = [r for r in reps if r["status"].startswith("DRC ")]
    assert shown, "the edit's DRC message was never shown"
    first_msg = shown[0]
    assert (first_msg["chip"]["errors"], first_msg["chip"]["warnings"]) != chips[
        0
    ] or len(set(chips)) > 1, (
        "the edit did not move the DRC count: pick another edit for this test"
    )
    for r in reps:  # the message and the chip never disagree
        if r["status"].startswith("DRC "):
            want = f"DRC {r['chip']['errors']} errors, {r['chip']['warnings']} warnings"
            assert r["status"].startswith(want), (r["status"], r["chip"])
    last = reps[-1]
    assert not last["status"].startswith("DRC "), last[
        "status"
    ]  # the stale summary is gone
