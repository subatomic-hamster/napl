"""The PROGRAM page's IDE ops (prog.* files / search / check / define / rename / hover, term.* pty shells), at the Backend's
JSON level, the way the window calls them. No display needed."""

import json
import os
import time
from pathlib import Path

import pytest
from napl.design import Design
from napl.design.viewer import Backend

REPO = Path(__file__).resolve().parents[2]
jedi = pytest.importorskip("jedi")


def ask(b, tool, **args):
    return json.loads(b(json.dumps({"tool": tool, "args": args})))


def ok(b, tool, **args):
    r = ask(b, tool, **args)
    assert r["ok"], r
    return r["result"]


@pytest.fixture
def be(tmp_path):
    root = tmp_path / "prog"
    root.mkdir()
    (root / "a.py").write_text(
        "def planner(x):\n    return x + 1\n\n\nprint(planner(2))\n"
    )
    (root / "b.py").write_text("from a import planner\n\nvalue = planner(5)\n")
    (root / "sub").mkdir()
    (root / "sub" / "n.toml").write_text("k = 1\n")
    b = Backend(lambda: None, tmp_path / "out", program_root=root)
    yield b
    b.prog.close()


def test_files_read_write_roundtrip(be):
    r = ok(be, "prog.files")
    assert r["files"] == ["a.py", "b.py", "sub/n.toml"]
    assert ok(be, "prog.read", path="sub/n.toml")["text"] == "k = 1\n"
    ok(be, "prog.write", path="sub/new/c.txt", text="héllo\n")
    assert ok(be, "prog.read", path="sub/new/c.txt")["text"] == "héllo\n"


def test_paths_are_confined(be, tmp_path):
    (tmp_path / "secret.txt").write_text("nope")
    (be.prog.root() / "link").symlink_to(tmp_path)
    for bad in (
        "../secret.txt",
        "/etc/passwd",
        "sub/../../secret.txt",
        "link/secret.txt",
        "",
    ):
        for tool in ("prog.read", "prog.write"):
            r = ask(be, tool, path=bad, text="x")
            assert not r["ok"] and "outside" in r["error"] or "no such" in r["error"], (
                tool,
                bad,
                r,
            )
    assert (tmp_path / "secret.txt").read_text() == "nope"
    assert "link" not in [
        f.split("/")[0] for f in ok(be, "prog.files")["files"]
    ]  # symlinked dirs are not walked


def test_search(be):
    m = ok(be, "prog.search", query="planner")["matches"]
    assert {(x["path"], x["line"]) for x in m} == {
        ("a.py", 0),
        ("a.py", 4),
        ("b.py", 0),
        ("b.py", 2),
    }  # first match per line
    assert not ok(be, "prog.search", query="PLANNER", case=True)["matches"]
    assert (
        len(ok(be, "prog.search", query=r"plan\w+\(\d\)", regex=True)["matches"]) == 2
    )
    assert not ok(be, "prog.search", query="plan", word=True)["matches"]
    assert not ask(be, "prog.search", query="(", regex=True)["ok"]


def test_check(be):
    bad = ok(be, "prog.check", path="a.py", text="def f(:\n  pass\n")["problems"]
    assert bad and bad[0]["line"] == 0
    assert not ok(be, "prog.check", path="a.py", text="x = 1\n")["problems"]
    t = ok(be, "prog.check", path="x.toml", text="a = 1\nb = \n")["problems"]
    assert t and t[0]["line"] == 1
    assert ok(be, "prog.check", path="x.json", text="{")["problems"]


def test_define_and_rename(be):
    root = be.prog.root()
    d = ok(be, "prog.define", path="b.py", line=2, col=10)  # `planner` in `planner(5)`
    assert d == {"path": "a.py", "line": 0, "col": 4}
    r = ok(be, "prog.rename", path="a.py", line=0, col=5, new="route")
    assert {e["path"] for e in r["edits"]} == {"a.py", "b.py"}
    assert (
        "def route(x)" in (root / "a.py").read_text()
        and "print(route(2))" in (root / "a.py").read_text()
    )
    assert (root / "b.py").read_text() == "from a import route\n\nvalue = route(5)\n"
    assert not ask(be, "prog.rename", path="a.py", line=0, col=5, new="not valid")["ok"]
    assert not ask(be, "prog.rename", path="a.py", line=0, col=5, new="class")["ok"]


def test_rename_refuses_library_code(be):
    (be.prog.root() / "c.py").write_text("import os\nos.getcwd()\n")
    r = ask(be, "prog.rename", path="c.py", line=1, col=3, new="x")
    assert not r["ok"] and "outside" in r["error"]


SKYDIO = REPO / "assemblies" / "skydio.toml"


@pytest.fixture(scope="module")
def sky():
    d = Design.open(SKYDIO)
    b = Backend(lambda: d, "/tmp/napl-progops-sky")
    yield d, b
    b.prog.close()


def test_root_is_the_programs_directory(sky):
    d, b = sky
    assert b.prog.root() == REPO / "examples" / "skydio"
    assert "planner.py" in ok(b, "prog.files")["files"]


def test_hover_shows_pins_and_wiring(sky, tmp_path):
    d, b = sky
    w = d.doc["wires"][0]
    (a, pa), (c, pc) = (w["from"].split(".")[:2], w["to"].split(".")[:2])
    f = b.prog.root() / "zz_hover_test.py"
    f.write_text(f'x = "{a}.{pa}"\ny = "{a}"\n')
    try:
        h = ok(b, "prog.hover", path=f.name, line=0, col=8)
        assert h["entity"]["kind"] == "port" and h["entity"]["instance"] == a
        peers = {
            (q["component"], q["port"]) for row in h["pins"] for q in row["connects"]
        }
        assert (c, pc) in peers
        assert all(
            {"component", "port", "pin", "connects"} <= set(r) for r in h["pins"]
        )
        i = ok(b, "prog.hover", path=f.name, line=1, col=6)
        assert i["entity"] == {"kind": "instance", "id": a, "part": i["entity"]["part"]}
        assert any(r["connects"] for r in i["pins"])
        dev = d.doc["devices"][0]
        f.write_text(f'q = "/devices/{d.asm.name}_{dev["name"]}"\n')  # the runtime path
        dv = ok(b, "prog.hover", path=f.name, line=0, col=14)
        assert dv["entity"]["kind"] == "device" and dv["entity"]["id"] == dev["name"]
        assert not ok(b, "prog.hover", path=f.name, line=0, col=0).get(
            "pins"
        )  # `q`: nothing wired
    finally:
        f.unlink()


# ---- terminals ----------------------------------------------------------------------------------------------------------------
def read_until(b, tid, needle, timeout=8.0):
    out, end = "", time.time() + timeout
    while time.time() < end:
        out += ok(b, "term.read", id=tid)["data"]
        if needle in out:
            return out
        time.sleep(0.05)
    raise AssertionError(f"{needle!r} not in {out!r}")


def test_terminal_echo_cwd_and_exit(be):
    tid = ok(be, "term.open")["id"]
    ok(be, "term.write", id=tid, data="echo hi$((20+1)); pwd\n")
    out = read_until(be, tid, "hi21")
    read_until(be, tid, "prog") if "prog" not in out else None
    ok(be, "term.write", id=tid, data="exit\n")
    end = time.time() + 5
    while time.time() < end and ok(be, "term.read", id=tid)["alive"]:
        time.sleep(0.05)
    assert not ok(be, "term.read", id=tid)["alive"]


def test_terminal_ctrl_c_interrupts_the_foreground_job_not_the_shell(be):
    tid = ok(be, "term.open")["id"]
    ok(be, "term.write", id=tid, data="sleep 60\n")
    time.sleep(0.5)
    t0 = time.time()
    ok(be, "term.write", id=tid, data="\x03")
    ok(be, "term.write", id=tid, data="echo after$((1+1))\n")
    read_until(be, tid, "after2", timeout=5)
    assert time.time() - t0 < 5
    assert ok(be, "term.read", id=tid)["alive"]


def test_terminal_kill_and_close_reap_the_shells(be):
    a, b2 = ok(be, "term.open")["id"], ok(be, "term.open")["id"]
    pa, pb = be.prog.terms[a].proc.pid, be.prog.terms[b2].proc.pid
    ok(be, "term.kill", id=a)
    assert not ask(be, "term.read", id=a)["ok"]
    be.prog.close()
    for pid in (pa, pb):
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)


def test_terminal_cwd_is_confined(be):
    assert not ask(be, "term.open", cwd="../..")["ok"]
    assert ok(be, "term.open", cwd="sub")["cwd"].endswith("sub")


def test_ops_are_not_mcp_tools():
    src = (REPO / "python" / "napl" / "mcp_server.py").read_text()
    assert "prog." not in src and "term." not in src
