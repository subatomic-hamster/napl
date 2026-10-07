"""V2: the viewer's Design window edits through the Design API. The Backend (JSON in / JSON out, what the window calls) must
give exactly what the same operations on a `Design` give, and a scripted window session (needs a display and a Vulkan device:
run under `xvfb-run -a`) must end in the design and DRC state the API reaches."""

import json
import os
from pathlib import Path

import napl
import pytest
from napl.design import Design
from napl.design.viewer import Backend

REPO = Path(__file__).resolve().parents[2]
X500 = REPO / "assemblies" / "x500v2.toml"
GPS = "holybro-m8n-gps"
needs_display = pytest.mark.skipif(
    not os.environ.get("DISPLAY"), reason="needs a display: run under xvfb-run"
)


def ask(b, tool, args=None, **kw):
    return json.loads(b(json.dumps({"tool": tool, "args": args or {}, **kw})))


def fkeys(findings):
    return sorted((f["code"], tuple(f["where"]), f["severity"]) for f in findings)


def api_steps(d):
    """The window's session on the Python API: remove the stock GPS, snap another, cycle its placement, autowire, undo."""
    yield d.remove("gps")
    yield d.add(GPS, id="gps2", to="mast.tray")
    d.undo()  # "dcycle 0": undo + the same add pinned to the chosen port pair and rotation
    pin = {
        "op": "add",
        "part": GPS,
        "id": "gps2",
        "to": "mast.tray",
        "port": "base",
        "k": 0,
    }
    yield d.edit([pin])
    yield d.autowire("gps2.gps_out")
    yield d.undo()
    yield d.remove("pm02")  # drops the power feed: the DRC changes
    yield d.undo()


def test_backend_replies_equal_the_design_api(tmp_path):
    d, ref = Design.open(X500), Design.open(X500)
    b = Backend(lambda: d, tmp_path)
    st0 = ask(b, "state")["state"]
    assert st0["rev"] == Backend.rev_of(ref) and st0["xml"] and st0["undo"] == 0
    tray = next(p for p in st0["ports"]["mast"] if p["name"] == "tray")
    assert tray["used"] and tray["kind"] == "mech"  # gps sits on it
    steps = [
        ("design_edit", {"ops": [{"op": "remove", "id": "gps"}]}, {}),
        (
            "design_edit",
            {"ops": [{"op": "add", "part": GPS, "id": "gps2", "to": "mast.tray"}]},
            {},
        ),
        (
            "design_edit",
            {
                "ops": [
                    {
                        "op": "add",
                        "part": GPS,
                        "id": "gps2",
                        "to": "mast.tray",
                        "port": "base",
                        "k": 0,
                    }
                ]
            },
            {"replace_last": True},
        ),
        ("design_edit", {"ops": [{"op": "autowire", "a": "gps2.gps_out"}]}, {}),
        ("design_undo", {}, {}),
    ]
    for (tool, args, kw), r in zip(steps, api_steps(ref)):
        reply = ask(b, tool, args, **kw)
        assert reply["ok"], reply
        assert reply["result"]["drc"] == json.loads(
            json.dumps(r.as_dict()["drc"])
        )  # the DRC delta of the very same edit
        assert reply["state"]["rev"] == Backend.rev_of(ref)  # the same document
        assert fkeys(reply["state"]["report"]["findings"]) == fkeys(
            [f.as_dict() for f in ref.findings()]
        )
    assert d.doc == ref.doc and reply["state"]["undo"] == len(ref._undo)
    # a failing edit changes nothing and says why (the tray is taken)
    taken = ask(
        b,
        "design_edit",
        {"ops": [{"op": "add", "part": GPS, "id": "gps3", "to": "mast.tray"}]},
    )
    assert (
        not taken["ok"] and taken["error"] and Backend.rev_of(d) == Backend.rev_of(ref)
    )
    # replace_last restores the step it undid when the replacement fails (history intact)
    before, hist = Backend.rev_of(d), (len(d._undo), len(d._redo))
    bad = ask(
        b,
        "design_edit",
        {"ops": [{"op": "add", "part": "no-such-part", "to": "mast.tray"}]},
        replace_last=True,
    )
    assert (
        not bad["ok"] and "no-such-part" in bad["error"] and Backend.rev_of(d) == before
    )
    assert (len(d._undo), len(d._redo)) == hist
    for args in (
        {"target": "mast", "limit": 3},
        {"target": "mast.tray", "limit": 5, "category": "sensor"},
    ):
        got = ask(b, "design_suggest", args)["result"]["suggestions"]
        want = d.suggest(args["target"], args["limit"], args.get("category"))
        assert got == json.loads(json.dumps(want))


def test_backend_involved_instances_of_a_finding(tmp_path):
    d = Design.open(X500)
    st = ask(Backend(lambda: d, tmp_path), "state")["state"]
    by = {f["code"] + "|" + ",".join(f["where"]): f for f in st["report"]["findings"]}
    f = by["CURRENT_PEAK_OVER_RATING|bat-to-pm02,bat.power_out"]
    assert f["instances"] == [
        "bat",
        "pm02",
    ]  # a wire name resolves to the instances at its two ends, a port to its instance


def test_backend_without_a_design_and_unknown_tools(tmp_path):
    b = Backend(lambda: None, tmp_path)
    assert ask(b, "rev")["result"] == {"rev": None}
    assert not ask(b, "state")["ok"]
    d = Design.open(X500)
    assert "unknown tool" in ask(Backend(lambda: d, tmp_path), "bogus")["error"]


@needs_display
@pytest.mark.gpu
def test_scripted_window_session_equals_the_python_api(tmp_path, capsys):
    d, ref = Design.open(X500), Design.open(X500)
    b = Backend(lambda: d, tmp_path / "b")
    st = b.state(d)
    sim = napl.Sim.load(st["xml"])
    script = (
        "5 dremove gps; 8 dsuggest mast.tray; 10 dsnap "
        + GPS
        + " mast.tray gps2; 12 dcycle 0; 14 dautowire gps2.gps_out;"
        " 16 dfinding 0; 18 dundo; 19 dremove pm02; 20 dundo; 22 dsim hover; 26 dsave "
        + str(tmp_path / "edited.toml")
    )
    sim.show(frames=60, script=script, width=1280, height=720, design=b)

    reqs = [(q, r) for q, r in b.log if q["tool"] not in ("rev", "state")]
    names = [
        q["tool"]
        + (":" + q["args"]["ops"][0]["op"] if q["tool"] == "design_edit" else "")
        for q, _ in reqs
    ]
    assert names == [
        "design_edit:remove",
        "design_suggest",
        "design_edit:add",
        "design_edit:add",
        "design_edit:autowire",
        "design_undo",
        "design_edit:remove",
        "design_undo",
        "design_simulate",
        "design_save",
    ], names
    assert all(r["ok"] for _, r in reqs), [r for _, r in reqs if not r["ok"]]
    edits = [r for q, r in reqs if q["tool"] in ("design_edit", "design_undo")]
    for r, a in zip(edits, api_steps(ref)):
        assert r["result"]["drc"] == json.loads(json.dumps(a.as_dict()["drc"]))
    assert any(
        r["result"]["drc"]["new"] for r in edits
    )  # an edit that really moves the DRC (not all steps are no-ops for it)
    assert d.doc == ref.doc and Backend.rev_of(d) == Backend.rev_of(ref)
    assert fkeys(edits[-1]["state"]["report"]["findings"]) == fkeys(
        [f.as_dict() for f in ref.findings()]
    )
    assert [q["args"] for q, _ in reqs if q["tool"] == "design_edit"][2]["ops"][0][
        "k"
    ] == 0  # the cycle pins port pair and rotation
    sug = next(r for q, r in reqs if q["tool"] == "design_suggest")["result"]
    ref0 = Design.open(X500)
    ref0.remove("gps")  # the design the library browser was asked about
    assert sug["suggestions"] == json.loads(json.dumps(ref0.suggest("mast.tray", 12)))

    # the quick check the Simulate button runs = Design.simulate on the same design
    hover = next(r for q, r in reqs if q["tool"] == "design_simulate")["result"]
    want = ref.simulate("hover", out=tmp_path / "ref")
    assert hover["pass"] == want["pass"] and hover["thrust_to_weight"] == pytest.approx(
        want["thrust_to_weight"]
    )
    assert hover["hover_throttle"] == pytest.approx(want["hover_throttle"], rel=1e-6)
    assert Design.open(tmp_path / "edited.toml").doc["instances"] == d.doc["instances"]

    # the World the window left behind is the last build (gps2 in, gps out), reloaded through the bus
    prims = sim.call({"cmd_type": "Query", "cmd": {"path": ""}})["prims"]
    assert any(p.endswith("/gps2") for p in prims) and not any(
        p.endswith("/gps") for p in prims
    )
    with capsys.disabled():
        print(f"\n[ viewer-design ] ops via window: {names}")
        print(
            f"[ viewer-design ] design == API design: {d.doc == ref.doc}; rev {Backend.rev_of(d)}"
        )
        print(
            f"[ viewer-design ] DRC after each edit (E/W/I): {[tuple(r['result']['drc']['after'].values()) for r in edits]}"
        )
        print(
            f"[ viewer-design ] World prims {len(prims)}; hover pass={hover['pass']} T/W={hover['thrust_to_weight']:.3f}"
        )


def test_cycling_placement_alternatives_keeps_one_undo_step(tmp_path):
    d = Design.open(X500)
    b = Backend(lambda: d, tmp_path)
    base = len(d._undo)
    ask(b, "design_edit", {"ops": [{"op": "remove", "id": "pdb"}]})
    r = ask(
        b,
        "design_edit",
        {
            "ops": [
                {
                    "op": "add",
                    "part": "holybro-pdb-60a",
                    "id": "pdb",
                    "to": "bottom.pdb",
                }
            ]
        },
    )
    snap = r["result"]["changes"][0]["snap"]
    assert [a["k"] for a in snap["alternatives"]] == [1, 2, 3] and snap["k"] == 0
    rot = lambda: next(m for m in d.doc["mates"] if m["b"].startswith("pdb."))[
        "rotation"
    ]
    for a in snap[
        "alternatives"
    ]:  # what "next >" sends: undo + the add pinned to that alternative
        pin = {
            "op": "add",
            "part": "holybro-pdb-60a",
            "id": "pdb",
            "to": a["to"],
            "port": a["port"],
            "k": a["k"],
        }
        assert ask(b, "design_edit", {"ops": [pin]}, replace_last=True)["ok"]
        assert rot() == a["k"] and len(d._undo) == base + 2


@needs_display
@pytest.mark.gpu
def test_mcp_view_shows_the_agents_design_edits_in_the_window(tmp_path):
    """`napl mcp --view`: the agent edits the design over MCP; the window (its Design panel shares that Design) notices
    within a second, rebuilds and reloads the model into the World the agent also sees."""
    import asyncio
    import shutil
    import sys
    import time

    from mcp import Client
    from mcp.client.stdio import StdioServerParameters

    shutil.copy(X500, tmp_path / "x500v2.toml")
    params = StdioServerParameters(
        command=sys.executable,
        args=[
            "-m",
            "napl.cli",
            "mcp",
            "--view",
            "--root",
            str(tmp_path),
            "--allow-destructive",
        ],
        cwd=tmp_path,
        env=dict(os.environ),
    )

    async def prims(c):
        return json.loads((await c.call_tool("list_prims", {})).content[0].text)[
            "prims"
        ]

    async def main():
        async with Client(params) as c:
            assert not (
                await c.call_tool("design_open", {"path": "x500v2.toml"})
            ).is_error
            assert not (await c.call_tool("design_build", {})).is_error
            assert not (
                await c.call_tool("load_model", {"path": "build/design/x500v2.xml"})
            ).is_error
            await asyncio.sleep(1.5)  # the window is up
            assert any(p.endswith("/gps") for p in await prims(c))
            r = await c.call_tool(
                "design_edit", {"ops": [{"op": "remove", "id": "gps"}]}
            )
            assert not r.is_error
            t0 = time.time()
            while time.time() - t0 < 15 and any(
                p.endswith("/gps") for p in await prims(c)
            ):
                await asyncio.sleep(0.25)
            return time.time() - t0, await prims(c)

    waited, after = asyncio.run(main())
    assert not any(p.endswith("/gps") for p in after), (
        "the window did not pick up the agent's edit"
    )
    assert any(p.endswith("/mast") for p in after)  # and the rest of the model is there
