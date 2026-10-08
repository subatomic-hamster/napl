"""The viewer's Design window talks to a Design through this module: JSON request in, JSON reply out. No design logic lives
here (edit.py is the single source of truth, the same `Design` the MCP `design_*` tools drive): each request maps onto one
Design method, the way the matching MCP tool does, and every reply carries the resulting `state` so the window can show
the new build, the DRC findings and the instance / port lists.

Requests: {"tool": "design_edit" | "design_suggest" | "design_undo" | "design_check" | "design_simulate" | "design_save" |
"state" | "rev", "args": {...}, "replace_last": bool}. `replace_last` (design_edit only) = undo the last edit, then apply
`ops` (cycling through the placement alternatives of a snap keeps one undo step). Replies: {"ok": true, "result": ...,
"state": ...} or {"ok": false, "error": "..."} (a failed edit changes nothing).
"""

import hashlib
import json
import tempfile
from pathlib import Path

from .library import DesignError
from .wire2d import Flatten


def _involved(doc, where) -> list:
    """Instance ids a finding's `where` entries (instance, 'inst.port', mate / wire name) refer to."""
    ids = {i["id"] for i in doc["instances"]}
    mates = {m.get("name"): (m["a"], m["b"]) for m in doc.get("mates", [])}
    wires = {w.get("name"): (w["from"], w["to"]) for w in doc.get("wires", [])}
    out = []
    for w in where:
        ends = mates.get(w) or wires.get(w) or (w,)
        for e in ends:
            i = e.partition(".")[0]
            if i in ids and i not in out:
                out.append(i)
    return out


class Backend:
    """`Backend(lambda: design, out_dir)(request_json) -> reply_json`. `lock`: shared with whoever else touches the Design
    (the MCP server); `save_to(path)`: how a save is written (default `Design.save`); `max_wall_s`: hover wall-clock cap."""

    def __init__(self, get_design, out_dir, lock=None, save_to=None, max_wall_s=600.0):
        import threading

        self.get_design, self.out = get_design, Path(out_dir)
        self.lock = lock or threading.RLock()
        self.save_to, self.max_wall_s = save_to, max_wall_s
        self.log = []  # [(request dict, reply dict)] of every request, for tests and replay of a UI session
        self._cache = None  # (design, rev, state)
        self.flat = Flatten()  # the 2D wiring mode (wire2d.py): the flatten.* tools

    # ---- entry point -------------------------------------------------------------------------------------------------
    def __call__(self, text: str) -> str:
        req = {}
        try:
            req = json.loads(text)
            with self.lock:
                reply = self._dispatch(req)
        except DesignError as e:
            reply = {"ok": False, "error": str(e)}
        except Exception as e:  # never let an exception cross into the viewer thread
            reply = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        self.log.append((req, reply))
        return json.dumps(reply, default=str)

    def _design(self):
        d = self.get_design()
        if d is None:
            raise DesignError("no design open")
        return d

    @staticmethod
    def rev_of(d) -> str:
        return hashlib.sha1(
            json.dumps(d.doc, sort_keys=True, default=str).encode()
        ).hexdigest()[:12]

    def _dispatch(self, req) -> dict:
        tool, a = req.get("tool"), req.get("args") or {}
        if tool == "rev":
            d = self.get_design()
            return {
                "ok": True,
                "result": {
                    "rev": self.rev_of(d) if d is not None else None,
                    "flat_rev": self.flat.rev,
                },
            }
        d = self._design()
        if (tool or "").startswith("flatten."):
            return self.flat.handle(self, d, tool, a)
        if tool == "state":
            return {"ok": True, "result": {}, "state": self.state(d)}
        if tool == "design_edit":
            if req.get("replace_last"):
                d.undo()
                try:
                    r = d.edit(a["ops"])
                except DesignError:
                    d.redo()
                    raise
            else:
                r = d.edit(a["ops"])
            return {
                "ok": True,
                "result": {**r.as_dict(), "undo_depth": len(d._undo)},
                "state": self.state(d),
            }
        if tool == "design_undo":
            r = d.redo() if a.get("redo") else d.undo()
            return {"ok": True, "result": r.as_dict(), "state": self.state(d)}
        if tool == "design_suggest":
            rows = d.suggest(a["target"], a.get("limit", 10), a.get("category") or None)
            return {"ok": True, "result": {"target": a["target"], "suggestions": rows}}
        if tool == "design_check":
            return {
                "ok": True,
                "result": {"findings": [f.as_dict() for f in d.check()]},
                "state": self.state(d),
            }
        if tool == "design_simulate":
            kw = {"wall_s": self.max_wall_s} if a["kind"] == "hover" else {}
            return {"ok": True, "result": d.simulate(a["kind"], out=self.out, **kw)}
        if tool == "design_save":
            path = a.get("path") or str(d.path or self.out / f"{d.asm.name}.toml")
            saved = self.save_to(path) if self.save_to else d.save(path)
            # save_to (the MCP tool's) returns {"saved": relative path}; Design.save a Path
            return {
                "ok": True,
                "result": {
                    "saved": saved["saved"] if isinstance(saved, dict) else str(saved)
                },
            }
        raise DesignError(f"unknown tool {tool!r}")

    # ---- the state the Design window shows ------------------------------------------------------------------------------
    def state(self, d) -> dict:
        """Builds the design (cached per document revision) and returns what the window needs. A design that does not
        build (a lost mate leaves a part floating) still has DRC findings and an instance list: `xml` is then null and
        `build_error` says why."""
        rev = self.rev_of(d)
        if self._cache and self._cache[0] is d and self._cache[1] == rev:
            st = self._cache[2]
        else:
            try:
                r = d.build(self.out)
                rep, xml, err = dict(r["report"]), str(r["xml"]), ""
            except DesignError as e:
                rep, xml, err = None, None, str(e)
            fs = d.check()  # the findings the edit results' DRC delta is computed on (not the build report's copy)
            if rep is None:
                rep = {
                    "assembly": d.asm.name,
                    "mass": {"total": 0, "by_group": {}},
                    "instances": [
                        {
                            "id": i["id"],
                            "part": i["part"],
                            "group": i.get("group", ""),
                            "body": i["id"],
                            "mass": 0,
                        }
                        for i in d.doc["instances"]
                    ],
                    "targets": [],
                }
            rep["findings"] = [
                {**f.as_dict(), "instances": _involved(d.doc, f.where)} for f in fs
            ]
            used = d._ports_used(d.doc)
            ports = {
                i["id"]: [
                    {
                        "name": p["name"],
                        "kind": p.get("kind", ""),
                        "type": p.get("type", ""),
                        "used": (i["id"], p["name"]) in used,
                    }
                    for p in d._part(i["part"]).ports
                ]
                for i in d.doc["instances"]
            }
            st = {
                "rev": rev,
                "xml": xml,
                "build_error": err,
                "report": rep,
                "ports": ports,
                "path": str(d.path or ""),
            }
            self._cache = (d, rev, st)
        return {**st, "undo": len(d._undo), "redo": len(d._redo)}


def view(design, out=None, **show_kw):
    """Open `design` (a napl.design.Design) in the viewer for interactive editing; blocks until the window closes.
    Builds into `out` (default: a temp dir), loads that build, shows it. Returns (Sim, Backend): the Sim holds the model as
    last built in the window, the Backend its request log. `show_kw` = Sim.show's frames / script / width / height."""
    from .. import Sim

    out = Path(out) if out else Path(tempfile.mkdtemp(prefix="napl-view-"))
    backend = Backend(lambda: design, out)
    st = backend.state(design)
    if st["xml"] is None:
        raise DesignError(f"the design does not build: {st['build_error']}")
    sim = Sim.load(st["xml"])
    sim.show(design=backend, **show_kw)
    return sim, backend
