"""Editing API: `Design` = a mutable working copy of an assembly document + its Library.

Every edit (add / remove / swap / mate / unmate / wire / set, or a batch through `edit`) is atomic: the document is
validated afterwards (numbers rule included); on any error nothing changes. Each returns an `EditResult` with what changed
and the DRC delta (findings new / resolved / changed against the previous state). `undo` / `redo` walk a snapshot stack.

Snapping and suggestions are heuristics over library data only (port type / gender / symmetry, category in this assembly,
ratings on power ports); the score weights below are choices, not physical data.
"""

import copy
import difflib
import re
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

from . import assembly as A
from .build import build as _build
from .drc import _default_parts
from .drc import check as _check
from .library import DesignError, Library, Part, genders_ok, types_ok
from .snap import find_compatible_ports, incompatible, n_mate, symmetry, thread_of

# suggest / snap score weights (design choices)
W_EXACT, W_FAMILY = (
    100,
    50,
)  # identical port type / paired family (shaft+bore, tube+tube-clamp)
W_GENDER, W_NEUTRAL = 10, 5  # male+female / a neutral side
W_SYMMETRY = 5  # x min(sym) / max(sym): equal symmetry scores the most
W_CATEGORY, W_FAMILY_CAT = (
    30,
    15,
)  # candidate category seen on ports of this type in the assembly / same top-level family
W_STOCK = 20  # the part that currently sits on the port
W_RATING_BAD = -40  # per failed rating (voltage / current) on a power-port candidate
MECH, POWER, SIGNAL = "mech", "power", "signal"


def _split(ref: str):
    i, _, p = ref.partition(".")
    return i, (p or None)


def _num(e):
    v = e.get("value") if isinstance(e, dict) else None
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _fkey(f):
    return (f.code, tuple(f.where))


def finding_delta(before, after):
    """(new, resolved, changed) between two finding lists, matched on (code, where); changed = same key, other severity / text."""

    def index(fs):
        seen, out = {}, {}
        for f in fs:
            k = _fkey(f)
            n = seen[k] = seen.get(k, 0) + 1
            out[(k, n)] = f
        return out

    b, a = index(before), index(after)
    new = [f.as_dict() for k, f in a.items() if k not in b]
    gone = [f.as_dict() for k, f in b.items() if k not in a]
    chg = [
        {"before": b[k].as_dict(), "after": f.as_dict()}
        for k, f in a.items()
        if k in b and (b[k].severity, b[k].message) != (f.severity, f.message)
    ]
    return new, gone, chg


def _counts(fs):
    return {s: sum(f.severity == s for f in fs) for s in ("error", "warning", "info")}


@dataclass
class EditResult:
    changes: list  # one dict per applied op: what changed (swap: kept / lost mates + wires, dangling references, ...)
    new: list = field(
        default_factory=list
    )  # DRC findings that did not exist before the edit
    resolved: list = field(default_factory=list)  # findings that no longer exist
    changed: list = field(
        default_factory=list
    )  # {"before", "after"} for the same finding with another severity / text
    before: dict = field(default_factory=dict)  # {"error": n, "warning": n, "info": n}
    after: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "changes": self.changes,
            "drc": {
                "before": self.before,
                "after": self.after,
                "new": self.new,
                "resolved": self.resolved,
                "changed": self.changed,
            },
        }


class Design:
    def __init__(self, asm: A.Assembly, lib: Library, path=None):
        self.asm, self.lib, self.path = asm, lib, path
        self._undo: list = []
        self._redo: list = []
        self._findings = None

    # ---- open / save -------------------------------------------------------------------------------------------------
    @classmethod
    def open(cls, path, parts=None) -> "Design":
        lib = Library(parts if parts is not None else _default_parts())
        return cls(A.load(path), lib, Path(path))

    @property
    def doc(self) -> dict:
        return self.asm.doc

    def save(self, path=None) -> Path:
        """Write the assembly TOML (deterministic; comments of a hand-written file are not kept). Checked by reading it back."""
        p = Path(path or self.path or "")
        if not str(p):
            raise DesignError(
                "save: no path given and the design was not opened from a file"
            )
        A.validate(self.doc)
        text = A.dumps(self.asm)
        if A.loads(text) != self.asm:
            raise DesignError(
                "save: the TOML written does not read back equal (writer bug); nothing written"
            )
        p.write_text(text, encoding="utf-8")
        self.path = p
        return p

    # ---- checks / build / simulate -----------------------------------------------------------------------------------
    def check(self) -> list:
        self._findings = _check(self.asm, self.lib)
        return self._findings

    def findings(self) -> list:
        return self._findings if self._findings is not None else self.check()

    def build(self, out=None) -> dict:
        """-> {"xml": Path, "report_path": Path, "report": dict} (the DRC findings are in report["findings"])."""
        out = Path(out) if out else Path(tempfile.mkdtemp(prefix="napl-design-"))
        r = _build(self.asm, self.lib.dir, out, lib=self.lib)
        self._findings = None
        return r

    def simulate(self, kind: str = "hover", out=None, **kw) -> dict:
        """kind = "hover" (multirotor: built-in altitude + attitude controller on the ESC throttles) or "hold" (arm: servos
        torque-on at a pose). Builds first; needs the C++ kernel. Returns the numbers listed in simulate.py."""
        from . import simulate as S

        if kind not in S.KINDS:
            raise DesignError(f"simulate: kind {kind!r} not in {sorted(S.KINDS)}")
        r = self.build(out)
        return S.KINDS[kind](r["xml"], r["report"], **kw)

    # ---- lookups -----------------------------------------------------------------------------------------------------
    def _inst(self, doc, iid: str) -> dict:
        for d in doc["instances"]:
            if d["id"] == iid:
                return d
        raise DesignError(
            f"unknown instance '{iid}'"
            + _didyoumean(iid, [d["id"] for d in doc["instances"]])
        )

    def _part(self, pid: str) -> Part:
        try:
            return self.lib.get(pid)
        except DesignError as e:
            raise DesignError(
                f"{e}" + _didyoumean(pid, list(self.lib.parts), "part")
            ) from None

    def _ports_used(self, doc) -> set:
        """{(instance, port)} on mech ports that some mate uses (a bare 'inst' end counts when its pair is unique)."""
        used = set()
        by_id = {d["id"]: d for d in doc["instances"]}
        for m in doc.get("mates", []):
            (ia, pa), (ib, pb) = _split(m["a"]), _split(m["b"])
            if ia not in by_id or ib not in by_id:
                continue
            if not (pa and pb):
                try:
                    c = find_compatible_ports(
                        self._part(by_id[ia]["part"]),
                        self._part(by_id[ib]["part"]),
                        pa,
                        pb,
                    )
                except DesignError:
                    c = []
                if len(c) != 1:
                    continue
                pa, pb = c[0]
            used |= {(ia, pa), (ib, pb)}
        return used

    def _connected(self, doc) -> set:
        root = next(d["id"] for d in doc["instances"] if d.get("root"))
        adj = {}
        for m in doc.get("mates", []):
            a, b = _split(m["a"])[0], _split(m["b"])[0]
            adj.setdefault(a, set()).add(b)
            adj.setdefault(b, set()).add(a)
        seen, todo = {root}, [root]
        while todo:
            for n in adj.get(todo.pop(), ()) - seen:
                seen.add(n)
                todo.append(n)
        return seen

    # ---- snapping ----------------------------------------------------------------------------------------------------
    @staticmethod
    def _pairs(ports_a, ports_b) -> list:
        """Compatible mech port pairs, best first: [{"a": port a, "b": port b, "score": s, "why": [...]}]."""
        out = []
        for i, pa in enumerate(ports_a):
            for j, pb in enumerate(ports_b):
                if (
                    pa.get("kind") != MECH
                    or pb.get("kind") != MECH
                    or incompatible(pa, pb)
                ):
                    continue
                exact = pa.get("type") == pb.get("type")
                s = W_EXACT if exact else W_FAMILY
                why = ["same port type" if exact else "paired family"]
                ga, gb = pa.get("gender"), pb.get("gender")
                if {ga, gb} == {"male", "female"}:
                    s += W_GENDER
                    why.append("male+female")
                else:
                    s += W_NEUTRAL
                sa, sb = symmetry(pa), symmetry(pb)
                s += W_SYMMETRY * min(sa, sb) / max(sa, sb)
                if sa != sb:
                    why.append(f"symmetry {sa} vs {sb}")
                # tie-break, not a score term: +1 when mating leaves the child right side up (its port faces -z where the
                # parent port faces +z, e.g. a board's `base`), -1 when it would be flipped (a `stack_top` onto a plate)
                up = pa.get("axis", [0, 0, 0])[2] * -pb.get("axis", [0, 0, 0])[2]
                out.append(
                    {"a": pa, "b": pb, "score": s, "why": why, "up": up, "_o": (i, j)}
                )
        out.sort(key=lambda r: (-r["score"], -r["up"], r["_o"]))
        return out

    def _pick(self, doc, parent_id, parent_port, child_part, child_port, what) -> tuple:
        """Best open compatible pair parent.port <-> child.port -> (parent port name, child port name, info dict)."""
        pinst = self._inst(doc, parent_id)
        ppart = self._part(pinst["part"])
        used = self._ports_used(doc)
        pports = [
            p
            for p in ppart.ports
            if (parent_port is None or p.get("name") == parent_port)
            and p.get("kind") == MECH
            and (parent_id, p["name"]) not in used
        ]
        if parent_port is not None and not pports:
            ppart.port(parent_port)  # raises with the port list when it does not exist
            raise DesignError(
                f"{parent_id}.{parent_port} is not an open mech port (already mated, or not mechanical)"
            )
        cports = [
            p
            for p in child_part.ports
            if child_port is None or p.get("name") == child_port
        ]
        if child_port is not None and not cports:
            child_part.port(child_port)
        pairs = self._pairs(pports, cports)
        if not pairs:
            free = [
                p["name"]
                for p in ppart.ports
                if p.get("kind") == MECH and (parent_id, p["name"]) not in used
            ]
            raise DesignError(
                f"{what}: no compatible open mech port pair between {parent_id} ({ppart.id}) and {child_part.id}; "
                f"open ports of {parent_id}: {free or 'none'}. design suggest {parent_id}.<port> lists parts that fit"
            )
        top = pairs[0]
        tied = [
            p
            for p in pairs
            if abs(p["score"] - top["score"]) < 1e-9 and p["up"] == top["up"]
        ]
        alts = [
            {
                "to": f"{parent_id}.{p['a']['name']}",
                "port": p["b"]["name"],
                "score": round(p["score"], 2),
            }
            for p in pairs[1:6]
        ]
        info = {
            "score": round(top["score"], 2),
            "why": top["why"],
            "ambiguous": len(tied) > 1,
            "alternatives": alts,
        }
        return top["a"]["name"], top["b"]["name"], info

    def _mate_dict(
        self,
        a,
        b,
        k,
        pa,
        pb,
        fasteners="auto",
        joint=None,
        name=None,
        nut=None,
        closure=False,
    ):
        m = {"a": a, "b": b}
        if name:
            m["name"] = name
        n = n_mate(pa, pb)
        k = 0 if k is None else k
        if isinstance(k, bool) or not isinstance(k, int) or not 0 <= k < n:
            raise DesignError(
                f"rotation k={k!r} out of range: ports {a} / {b} allow 0..{n - 1}"
            )
        m["rotation"] = k
        if fasteners == "auto":
            if thread_of(pa) or thread_of(pb):
                m["fasteners"] = "auto"
        elif fasteners:
            m["fasteners"] = fasteners
        if nut:
            m["nut"] = nut
        if joint:
            m["joint"] = joint
        if closure:
            m["closure"] = True
        return m

    # ---- the ops (each mutates `doc` and returns a change dict) ------------------------------------------------------
    def _op_add(self, doc, s) -> dict:
        part = self._part(s["part"])
        ids = {d["id"] for d in doc["instances"]}
        iid = s.get("id")
        if not iid:
            iid, n = part.id, 1
            while iid in ids:
                n += 1
                iid = f"{part.id}-{n}"
        if iid in ids:
            raise DesignError(f"instance id '{iid}' already exists")
        inst = {"id": iid, "part": part.id}
        for k in ("group", "offboard"):
            if s.get(k) is not None:
                inst[k] = s[k]
        ch = {"op": "add", "id": iid, "part": part.id}
        if s.get("to"):
            tid, tport = _split(s["to"])
            ppn, cpn, info = self._pick(
                doc, tid, tport, part, s.get("port"), f"add {part.id} to {s['to']}"
            )
            if s.get("strict") and info["ambiguous"]:
                raise DesignError(
                    f"snap is ambiguous (strict): {[info['why']] + info['alternatives']}"
                )
            pa, pb = self._part(self._inst(doc, tid)["part"]).port(ppn), part.port(cpn)
            m = self._mate_dict(
                f"{tid}.{ppn}",
                f"{iid}.{cpn}",
                s.get("k"),
                pa,
                pb,
                fasteners=s.get("fasteners", "auto"),
                joint=s.get("joint"),
                nut=s.get("nut"),
                name=s.get(
                    "mate"
                ),  # mate name (device bindings refer to spin mates by it)
            )
            doc["instances"].append(inst)
            doc.setdefault("mates", []).append(m)
            ch.update(mate=m, snap=info)
        else:
            if s.get("port") or s.get("k") is not None:
                raise DesignError("add: port / k only apply together with `to`")
            doc["instances"].append(inst)
            ch["note"] = (
                "not mated: it is floating until a mate connects it (design suggest <inst.port> lists parts / ports)"
            )
        return ch

    def _drop_refs(self, doc, iid, mate_names, ch):
        """Devices bound to a removed instance / mate go with it."""
        keep = []
        for d in doc.get("devices", []):
            hit = [
                k
                for k in ("instance", "body", "site", "motor")
                if iid and d.get(k) == iid
            ] or [k for k in ("joint", "backlash_joint") if d.get(k) in mate_names]
            if hit:
                ch["devices_removed"].append(
                    {"name": d["name"], "type": d["type"], "because": hit}
                )
            else:
                keep.append(d)
        if "devices" in doc:
            doc["devices"] = keep
        if "devices" in doc and not doc["devices"]:
            del doc["devices"]

    def _op_remove(self, doc, s) -> dict:
        iid = s["id"]
        inst = self._inst(doc, iid)
        if inst.get("root"):
            raise DesignError(f"cannot remove the root instance '{iid}'")
        before = self._connected(doc)
        ch = {
            "op": "remove",
            "id": iid,
            "part": inst["part"],
            "mates_removed": [],
            "wires_removed": [],
            "devices_removed": [],
        }
        doc["instances"].remove(inst)
        names = set()
        keep = []
        for m in doc.get("mates", []):
            if iid in (_split(m["a"])[0], _split(m["b"])[0]):
                nm = m.get("name", f"{m['a']}--{m['b']}")
                names.add(nm)
                ch["mates_removed"].append(nm)
            else:
                keep.append(m)
        if "mates" in doc:
            doc["mates"] = keep
        wk = []
        for w in doc.get("wires", []):
            if (
                iid in (_split(w["from"])[0], _split(w["to"])[0])
                or w.get("included_in") == iid
            ):
                ch["wires_removed"].append(w.get("name", f"{w['from']}--{w['to']}"))
            else:
                wk.append(w)
        if "wires" in doc:
            doc["wires"] = wk
        self._drop_refs(doc, iid, names, ch)
        after = self._connected(doc)
        orphans = sorted(before - after - {iid})
        if orphans:
            ch["orphaned"] = (
                orphans  # still in the design, no longer connected to the root (DRC FLOATING_INSTANCE)
            )
        return ch

    def _op_unmate(self, doc, s) -> dict:
        mates = doc.get("mates", [])
        if s.get("name"):
            hit = [
                m for m in mates if m.get("name", f"{m['a']}--{m['b']}") == s["name"]
            ]
        else:
            if not (s.get("a") and s.get("b")):
                raise DesignError("unmate: give the mate `name`, or both ends a and b")

            def end(
                given, actual
            ):  # 'inst' matches any port of inst, 'inst.port' only that port
                return given == actual or (
                    _split(given)[1] is None and given == _split(actual)[0]
                )

            hit = [
                m
                for m in mates
                if (end(s["a"], m["a"]) and end(s["b"], m["b"]))
                or (end(s["a"], m["b"]) and end(s["b"], m["a"]))
            ]
        if not hit:
            raise DesignError(
                "unmate: no such mate; mates: "
                + ", ".join(m.get("name", f"{m['a']}--{m['b']}") for m in mates[:40])
            )
        if len(hit) > 1:
            raise DesignError(
                f"unmate: {len(hit)} mates match, give the mate name: {[m.get('name', m['a'] + '--' + m['b']) for m in hit]}"
            )
        m = hit[0]
        before = self._connected(doc)
        nm = m.get("name", f"{m['a']}--{m['b']}")
        mates.remove(m)
        ch = {"op": "unmate", "mate": nm, "dangling": self._dev_joints(doc, {nm})}
        orphans = sorted(before - self._connected(doc))
        if orphans:
            ch["orphaned"] = orphans
        return ch

    def _op_mate(self, doc, s) -> dict:
        (ia, pa), (ib, pb) = _split(s["a"]), _split(s["b"])
        A_, B_ = self._inst(doc, ia), self._inst(doc, ib)
        partA, partB = self._part(A_["part"]), self._part(B_["part"])
        info = None
        if pa is None or pb is None:
            used = self._ports_used(doc)
            ports_a = [
                p
                for p in partA.ports
                if (pa is None or p.get("name") == pa) and (ia, p["name"]) not in used
            ]
            ports_b = [
                p
                for p in partB.ports
                if (pb is None or p.get("name") == pb) and (ib, p["name"]) not in used
            ]
            pairs = self._pairs(ports_a, ports_b)
            if not pairs:
                raise DesignError(
                    f"mate {s['a']} / {s['b']}: no compatible open mech port pair"
                )
            top = pairs[0]
            tied = [
                p
                for p in pairs
                if abs(p["score"] - top["score"]) < 1e-9 and p["up"] == top["up"]
            ]
            pa, pb = top["a"]["name"], top["b"]["name"]
            info = {
                "score": round(top["score"], 2),
                "ambiguous": len(tied) > 1,
                "alternatives": [
                    {"a": f"{ia}.{p['a']['name']}", "b": f"{ib}.{p['b']['name']}"}
                    for p in pairs[1:6]
                ],
            }
        porta, portb = partA.port(pa), partB.port(pb)
        if porta.get("kind") != MECH or portb.get("kind") != MECH:
            raise DesignError(
                f"mate {ia}.{pa} / {ib}.{pb}: both ports must be mech ports"
            )
        why = incompatible(porta, portb)
        if why:
            raise DesignError(f"mate {ia}.{pa} / {ib}.{pb}: {why}")
        k = s.get("k")
        conn = self._connected(doc)
        a, b = f"{ia}.{pa}", f"{ib}.{pb}"
        if (
            ia not in conn and ib in conn
        ):  # a = parent: the side already connected to the root
            a, b, partA, partB, porta, portb, ia, ib = (
                b,
                a,
                partB,
                partA,
                portb,
                porta,
                ib,
                ia,
            )
            k = (-(k or 0)) % n_mate(porta, portb)
        elif ia in conn and ib in conn and not s.get("closure"):
            raise DesignError(
                f"mate {a} / {b}: both are already connected to the root, so this closes a loop; pass closure=true if it is a loop closure"
            )
        names = {m.get("name", f"{m['a']}--{m['b']}") for m in doc.get("mates", [])}
        m = self._mate_dict(
            a,
            b,
            k,
            porta,
            portb,
            s.get("fasteners", "auto"),
            s.get("joint"),
            s.get("name"),
            s.get("nut"),
            s.get("closure"),
        )
        if m.get("name", f"{a}--{b}") in names:
            raise DesignError(
                f"a mate named '{m.get('name', f'{a}--{b}')}' already exists (pass a name)"
            )
        doc.setdefault("mates", []).append(m)
        ch = {"op": "mate", "mate": m}
        if info:
            ch["snap"] = info
        return ch

    def _op_wire(self, doc, s) -> dict:
        for end in ("from", "to"):
            iid, pn = _split(s[end])
            if pn is None:
                raise DesignError(
                    f"wire {end} '{s[end]}': name the port as 'inst.port'"
                )
            p = self._part(self._inst(doc, iid)["part"]).port(pn)
            if p.get("kind") not in (POWER, SIGNAL):
                raise DesignError(
                    f"wire {end} '{s[end]}': port kind {p.get('kind')!r}, wires join power or signal ports"
                )
        self._part(s["part"])
        w = {
            "name": s.get("name") or f"{s['from']}--{s['to']}",
            "from": s["from"],
            "to": s["to"],
            "part": s["part"],
            "length": s["length"],
        }
        if s.get("conductors") is not None:
            w["conductors"] = s["conductors"]
        if s.get("included_in"):
            w["included_in"] = s["included_in"]
        if s.get("connectors"):  # pin the end connector parts: {"from": id, "to": id}
            w["connectors"] = s["connectors"]
        if w["name"] in {x.get("name") for x in doc.get("wires", [])}:
            raise DesignError(
                f"a wire named '{w['name']}' already exists (pass a name)"
            )
        doc.setdefault("wires", []).append(w)
        return {"op": "wire", "wire": w}

    def _op_device_map(self, doc, s) -> dict:
        """device_map: name + map ({"motor.kv": "motor:kv", ...}): a [device_maps.<name>] table, replaced when it exists."""
        if not s.get("name") or not isinstance(s.get("map"), dict) or not s["map"]:
            raise DesignError("device_map: needs name and a non-empty map {field: ref}")
        doc.setdefault("device_maps", {})[s["name"]] = dict(s["map"])
        return {"op": "device_map", "name": s["name"], "map": dict(s["map"])}

    def _op_device(self, doc, s) -> dict:
        """device: device = {type, name, instance, port, map, set, ...}: one [[devices]] binding (see assemblies/README.md)."""
        dev = s.get("device")
        if not isinstance(dev, dict) or not dev.get("type") or not dev.get("name"):
            raise DesignError("device: needs device = {type, name, ...}")
        if dev["name"] in {x.get("name") for x in doc.get("devices", [])}:
            raise DesignError(f"a device named '{dev['name']}' already exists")
        doc.setdefault("devices", []).append(copy.deepcopy(dev))
        return {"op": "device", "device": dev}

    def _op_target(self, doc, s) -> dict:
        """target: name + value ('<inst>.<param>' or a provenance table) + groups=[...] | all=True: a mass target."""
        if not s.get("name") or s.get("value") is None:
            raise DesignError("target: needs name and value")
        t = {"name": s["name"]}
        if s.get("all"):
            t["all"] = True
        else:
            t["groups"] = list(s.get("groups") or [])
        t["value"] = s["value"]
        if s["name"] in {x.get("name") for x in doc.get("targets", [])}:
            raise DesignError(f"a target named '{s['name']}' already exists")
        doc.setdefault("targets", []).append(t)
        return {"op": "target", "target": t}

    def device_map(self, name, map) -> EditResult:
        return self.edit([{"op": "device_map", "name": name, "map": map}])

    def bind(self, **device) -> EditResult:
        """Add a device binding: bind(type="esc", name="esc1", instance="esc", port="power_in", motor="mot1", ...)."""
        return self.edit([{"op": "device", "device": device}])

    def target(self, name, value, groups=None, all=False) -> EditResult:
        return self.edit(
            [
                {
                    "op": "target",
                    "name": name,
                    "value": value,
                    "groups": groups,
                    "all": all,
                }
            ]
        )

    def _op_unwire(self, doc, s) -> dict:
        wires = doc.get("wires", [])
        hit = [
            w for w in wires if w.get("name", f"{w['from']}--{w['to']}") == s["name"]
        ]
        if not hit:
            raise DesignError(
                f"unwire: no wire named '{s['name']}'; wires: "
                + ", ".join(
                    w.get("name", f"{w['from']}--{w['to']}") for w in wires[:40]
                )
            )
        wires.remove(hit[0])
        if not wires:
            del doc["wires"]
        return {"op": "unwire", "wire": hit[0]}

    def _op_set(self, doc, s) -> dict:
        inst = self._inst(doc, s["id"])
        part = self._part(inst["part"])
        param = s["param"]
        old = inst.get("params", {}).get(param) or part.params.get(param)
        if s.get("unset"):
            if param not in inst.get("params", {}):
                raise DesignError(f"set: '{s['id']}' has no override for '{param}'")
            del inst["params"][param]
            if not inst["params"]:
                del inst["params"]
            return {"op": "set", "id": s["id"], "param": param, "unset": True}
        v = s.get("value")
        if isinstance(v, dict):  # a full provenance table
            entry = dict(v)
        else:
            entry = {"value": v, "unit": s.get("unit"), "fidelity": s.get("fidelity")}
            for k in ("src", "quote", "derivation", "rationale"):
                if s.get(k):
                    entry[k] = s[k]
            if entry["unit"] is None or entry["fidelity"] is None:
                raise DesignError(
                    f"set {s['id']}.{param}: numbers rule: give unit and fidelity (+ src & quote | derivation | rationale), "
                    "or a provenance table as value; a bare number is not accepted"
                )
        A.check_provenance(entry, f"set {s['id']}.{param}")
        inst.setdefault("params", {})[param] = entry
        ch = {
            "op": "set",
            "id": s["id"],
            "param": param,
            "value": entry["value"],
            "unit": entry["unit"],
            "fidelity": entry["fidelity"],
        }
        if isinstance(old, dict) and "value" in old:
            ch["was"] = {
                "value": old["value"],
                "unit": old.get("unit"),
                "fidelity": old.get("fidelity"),
            }
            if old.get("unit") != entry["unit"]:
                ch["warning"] = (
                    f"unit changed from {old.get('unit')!r} to {entry['unit']!r}"
                )
        else:
            ch["note"] = f"part {part.id} has no param '{param}': a new override"
        return ch

    # ---- swap --------------------------------------------------------------------------------------------------------
    def _new_port(
        self,
        new: Part,
        old_port: dict,
        other_port: dict | None,
        other_part: Part,
        kind: str,
    ):
        """Port of `new` that takes over `old_port`: same name, else same type, else any compatible with the other end.
        -> (port, how, alternatives) or (None, "", [])."""
        if kind == MECH:

            def ok(q):
                if q.get("kind") != MECH:
                    return False
                if other_port is not None:
                    return incompatible(other_port, q) is None
                return any(
                    incompatible(o, q) is None
                    for o in other_part.ports
                    if o.get("kind") == MECH
                )

            cands = [q for q in new.ports if ok(q)]
        else:
            cands = [
                q
                for q in new.ports
                if q.get("kind") == kind
                and types_ok(old_port.get("type", ""), q.get("type", ""))
                and genders_ok(old_port.get("gender", ""), q.get("gender", ""))
            ]
        by_name = [q for q in cands if q.get("name") == old_port.get("name")]
        same_type = [q for q in cands if q.get("type") == old_port.get("type")]
        for tier, how in (
            (by_name, "same port name"),
            (same_type, "same port type"),
            (cands, "compatible port"),
        ):
            if tier:
                alts = [q["name"] for q in tier[1:]] if how != "same port name" else []
                return tier[0], how, alts
        if (
            kind != MECH
        ):  # power / signal: same name and kind but another type is kept, DRC will judge the connector
            same = [
                q
                for q in new.ports
                if q.get("name") == old_port.get("name") and q.get("kind") == kind
            ]
            if same:
                return same[0], "same port name (type changed)", []
        return None, "", []

    def _op_swap(self, doc, s) -> dict:
        iid = s["id"]
        inst = self._inst(doc, iid)
        old, new = self._part(inst["part"]), self._part(s["part"])
        ch = {
            "op": "swap",
            "id": iid,
            "from": old.id,
            "to": new.id,
            "mates": {"kept": [], "lost": []},
            "wires": {"kept": [], "lost": []},
            "devices": [],
            "overrides": {"kept": [], "dropped": []},
            "dangling": [],
            "notes": [],
        }
        by_id = {d["id"]: d for d in doc["instances"]}
        # mates
        for m in list(doc.get("mates", [])):
            for end in ("a", "b"):
                if _split(m[end])[0] != iid:
                    continue
                oend = "b" if end == "a" else "a"
                oi, opn = _split(m[oend])
                opart = self._part(by_id[oi]["part"])
                pn = _split(m[end])[1]
                nm = m.get("name", f"{m['a']}--{m['b']}")
                oport = opart.port(opn) if opn else None
                oldp = old.port(pn) if pn else None
                if oldp is None:  # bare 'inst' end: resolve through the other end
                    c = find_compatible_ports(old, opart, None, opn)
                    if len(c) == 1:
                        oldp = old.port(c[0][0])
                        oport = oport or opart.port(c[0][1])
                if oldp is None:
                    ch["mates"]["lost"].append(
                        {"mate": nm, "old_port": pn, "why": "old port not resolvable"}
                    )
                    m["__lost"] = True
                    continue
                q, how, alts = self._new_port(new, oldp, oport, opart, MECH)
                if q is None:
                    ch["mates"]["lost"].append(
                        {
                            "mate": nm,
                            "old_port": oldp["name"],
                            "other_end": m[oend],
                            "why": f"no port of {new.id} is compatible with {m[oend]}"
                            + (
                                f" ({oport.get('type')}, {oport.get('gender')})"
                                if oport
                                else ""
                            ),
                        }
                    )
                    m["__lost"] = True
                    continue
                if "name" not in m:
                    m["name"] = (
                        nm  # the default name derives from the port names; keep it (device bindings use it)
                    )
                m[end] = f"{iid}.{q['name']}"
                rec = {
                    "mate": nm,
                    "old_port": oldp["name"],
                    "new_port": q["name"],
                    "how": how,
                }
                if alts:
                    rec["alternatives"] = alts
                if oport is not None:
                    n = n_mate(*(oport, q))
                    if m.get("rotation", 0) >= n:
                        m["rotation"] = 0
                        rec["rotation"] = "reset to 0 (new symmetry)"
                    f = m.get("fasteners")
                    ta, tb = thread_of(oport), thread_of(q)
                    th = ta or tb
                    if f is not None and ((ta and tb and ta != tb) or (th is None)):
                        rec["fasteners"] = (
                            f"was {f if f == 'auto' else f['part']}; reset to auto (thread {ta} vs {tb})"
                            if th
                            else "removed (no thread on the new ports)"
                        )
                        if th:
                            m["fasteners"] = "auto"
                        else:
                            m.pop("fasteners", None)
                            m.pop("nut", None)
                    elif (
                        isinstance(f, dict)
                        and th
                        and f.get("part")
                        and not self._part(f["part"]).has_port(f"thread.{th}", "male")
                    ):
                        m["fasteners"] = "auto"
                        rec["fasteners"] = (
                            f"was {f['part']}; reset to auto (not a {th} screw)"
                        )
                ch["mates"]["kept"].append(rec)
        if any(m.get("__lost") for m in doc.get("mates", [])):
            lost_names = [
                m.get("name", f"{m['a']}--{m['b']}")
                for m in doc["mates"]
                if m.get("__lost")
            ]
            doc["mates"] = [m for m in doc["mates"] if not m.get("__lost")]
            ch["dangling"] += self._dev_joints(doc, set(lost_names))
        # wires
        for w in list(doc.get("wires", [])):
            for end in ("from", "to"):
                if _split(w[end])[0] != iid:
                    continue
                pn = _split(w[end])[1]
                nm = w.get("name", f"{w['from']}--{w['to']}")
                oldp = old.port(pn)
                q, how, alts = self._new_port(new, oldp, None, old, oldp.get("kind"))
                if q is None:
                    ch["wires"]["lost"].append({"wire": nm, "old_port": pn})
                    w["__lost"] = True
                    continue
                if "name" not in w:
                    w["name"] = nm
                w[end] = f"{iid}.{q['name']}"
                rec = {"wire": nm, "old_port": pn, "new_port": q["name"], "how": how}
                if alts:
                    rec["alternatives"] = alts
                ch["wires"]["kept"].append(rec)
        if any(w.get("__lost") for w in doc.get("wires", [])):
            doc["wires"] = [w for w in doc["wires"] if not w.get("__lost")]
        # overrides: those that shadow a param of the old part were tuned for it; custom extras stay
        ov = inst.get("params", {})
        for k in list(ov):
            if k in old.params:
                ch["overrides"]["dropped"].append(k)
                del ov[k]
            else:
                ch["overrides"]["kept"].append(k)
        if not ov:
            inst.pop("params", None)
        inst["part"] = new.id
        # device bindings: port-valued fields on this instance
        for d in doc.get("devices", []):
            if d.get("instance") == iid:
                for key in ("port", "a", "b", "site_port"):
                    pn = d.get(key)
                    if (
                        pn is None
                        or pn == "ground"
                        or any(p.get("name") == pn for p in new.ports)
                    ):
                        continue
                    try:
                        oldp = old.port(pn)
                    except DesignError:
                        continue
                    q, how, _ = self._new_port(new, oldp, None, old, oldp.get("kind"))
                    if q:
                        d[key] = q["name"]
                        ch["devices"].append(
                            {"device": d["name"], key: f"{pn} -> {q['name']} ({how})"}
                        )
                    else:
                        ch["dangling"].append(
                            f"device '{d['name']}' {key} = '{pn}': {new.id} has no such port"
                        )
        ch["dangling"] += self._dangling(doc, iid, new)
        return ch

    @staticmethod
    def _dev_joints(doc, names) -> list:
        """Device bindings that name a joint (mate) that no longer exists: kept, so re-creating the mate restores them."""
        return [
            f"device '{d['name']}' {k} = '{d[k]}': that mate no longer exists (mate it again with name '{d[k]}')"
            for d in doc.get("devices", [])
            for k in ("joint", "backlash_joint")
            if d.get(k) in names
        ]

    def _dangling(self, doc, iid, new: Part) -> list:
        """References the swapped-in part cannot satisfy: '<iid>.<param>' strings, device-map params, @table.column."""
        out, params = [], {*new.params}
        inst = self._inst(doc, iid)
        have = params | set(inst.get("params", {}))
        tables = new.raw.get("tables", {})

        def ref(s, where):
            m = (
                re.fullmatch(rf"{re.escape(iid)}\.([A-Za-z0-9_]+)", s)
                if isinstance(s, str)
                else None
            )
            if m and m.group(1) not in have:
                out.append(
                    f"{where}: '{s}' no longer resolves ({new.id} has no param '{m.group(1)}')"
                )

        def walk(x, where):
            if isinstance(x, str):
                ref(x, where)
            elif isinstance(x, dict):
                for k, v in x.items():
                    walk(v, f"{where}.{k}")
            elif isinstance(x, list):
                for i, v in enumerate(x):
                    walk(v, f"{where}[{i}]")

        for sec, key in (("mates", "joint"), ("wires", "length"), ("targets", "value")):
            for i, it in enumerate(
                doc.get(sec, [])
            ):  # numeric fields only: mate / wire ends are port refs, not params
                walk(it.get(key), f"{sec}[{i}].{key}")
        for d in doc.get("devices", []):
            walk(d.get("set", {}), f"device '{d['name']}' set")
            maps = doc.get("device_maps", {}).get(d.get("map"), {})
            for fld, spec in maps.items():
                src, _, name = spec.rpartition(":") if ":" in spec else ("", "", spec)
                holder = d.get(src) if src else d.get("instance")
                if holder != iid:
                    continue
                if name.startswith("@"):
                    tb, _, col = name[1:].partition(".")
                    t = tables.get(tb)
                    if not t or col not in t.get("columns", []):
                        out.append(
                            f"device '{d['name']}' map '{d['map']}' {fld}: {new.id} has no table column {name}"
                        )
                elif name not in have:
                    out.append(
                        f"device '{d['name']}' map '{d['map']}' {fld}: {new.id} has no param '{name}' (set it on '{iid}' with provenance, or pick a part that has it)"
                    )
        return out

    # ---- transaction -------------------------------------------------------------------------------------------------
    def _expand(self, ops):
        out = []
        for o in ops:
            if o.get("op") == "swap" and isinstance(o.get("id"), list):
                out += [{**o, "id": i} for i in o["id"]]
            else:
                out.append(o)
        return out

    def edit(self, ops: list) -> EditResult:
        """Apply a list of op dicts ({"op": add|remove|swap|mate|unmate|wire|unwire|set, ...}) atomically: all or none."""
        table = {
            "add": self._op_add,
            "remove": self._op_remove,
            "swap": self._op_swap,
            "mate": self._op_mate,
            "unmate": self._op_unmate,
            "wire": self._op_wire,
            "unwire": self._op_unwire,
            "set": self._op_set,
            "device_map": self._op_device_map,
            "device": self._op_device,
            "target": self._op_target,
        }
        before = self.findings()
        snap = copy.deepcopy(self.doc)
        changes = []
        try:
            for o in self._expand(ops):
                fn = table.get(o.get("op"))
                if fn is None:
                    raise DesignError(
                        f"unknown op {o.get('op')!r} (allowed: {sorted(table)})"
                    )
                changes.append(fn(self.doc, {k: v for k, v in o.items() if k != "op"}))
            A.validate(self.doc)
            for d in self.doc[
                "instances"
            ]:  # unknown part ids surface here, not at build time
                self._part(d["part"])
        except DesignError:
            self.asm.doc = snap
            raise
        except (KeyError, TypeError, ValueError, StopIteration) as e:
            self.asm.doc = snap
            raise DesignError(
                f"bad edit ({type(e).__name__}: {e}); nothing changed"
            ) from None
        self._undo.append(snap)
        self._redo.clear()
        return self._result(changes, before)

    def _result(self, changes, before) -> EditResult:
        after = self.check()
        new, gone, chg = finding_delta(before, after)
        return EditResult(changes, new, gone, chg, _counts(before), _counts(after))

    # ---- public edit methods -----------------------------------------------------------------------------------------
    def add(self, part, id=None, to=None, port=None, k=None, **kw) -> EditResult:
        return self.edit(
            [
                {
                    "op": "add",
                    "part": part,
                    "id": id,
                    "to": to,
                    "port": port,
                    "k": k,
                    **kw,
                }
            ]
        )

    def snap(
        self, new_part, to, port=None, k=None, id=None, strict=False, **kw
    ) -> EditResult:
        """add(new_part) mated to the best open compatible port pair of `to` (instance or 'inst.port'); the change record
        has snap = {score, why, ambiguous, alternatives}; strict=True raises when the best pair is tied. Extra keywords
        (joint=..., fasteners=..., nut=..., group=...) go to the mate / instance like `mate` / `add` take them."""
        return self.add(new_part, id=id, to=to, port=port, k=k, strict=strict, **kw)

    def remove(self, id) -> EditResult:
        return self.edit([{"op": "remove", "id": id}])

    def swap(self, id, new_part) -> EditResult:
        return self.edit([{"op": "swap", "id": id, "part": new_part}])

    def mate(self, a, b, k=None, joint=None, fasteners="auto", **kw) -> EditResult:
        return self.edit(
            [
                {
                    "op": "mate",
                    "a": a,
                    "b": b,
                    "k": k,
                    "joint": joint,
                    "fasteners": fasteners,
                    **kw,
                }
            ]
        )

    def unmate(self, name=None, a=None, b=None) -> EditResult:
        return self.edit([{"op": "unmate", "name": name, "a": a, "b": b}])

    def unwire(self, name) -> EditResult:
        return self.edit([{"op": "unwire", "name": name}])

    def wire(self, from_, to, part, length, **kw) -> EditResult:
        return self.edit(
            [
                {
                    "op": "wire",
                    "from": from_,
                    "to": to,
                    "part": part,
                    "length": length,
                    **kw,
                }
            ]
        )

    def set(
        self,
        id,
        param,
        value=None,
        unit=None,
        fidelity=None,
        src=None,
        quote=None,
        derivation=None,
        rationale=None,
        unset=False,
    ) -> EditResult:
        return self.edit(
            [
                {
                    "op": "set",
                    "id": id,
                    "param": param,
                    "value": value,
                    "unit": unit,
                    "fidelity": fidelity,
                    "src": src,
                    "quote": quote,
                    "derivation": derivation,
                    "rationale": rationale,
                    "unset": unset,
                }
            ]
        )

    def undo(self) -> EditResult:
        return self._travel(self._undo, self._redo, "undo")

    def redo(self) -> EditResult:
        return self._travel(self._redo, self._undo, "redo")

    def _travel(self, src, dst, what) -> EditResult:
        if not src:
            raise DesignError(f"nothing to {what}")
        before = self.findings()
        dst.append(copy.deepcopy(self.doc))
        self.asm.doc = src.pop()
        return self._result([{"op": what}], before)

    # ---- suggestions -------------------------------------------------------------------------------------------------
    def suggest(self, target: str, limit: int = 10) -> list:
        """target = 'inst.port': library parts (with the port they would use) that mate / connect that port, best first.
        target = 'inst': replacement candidates for that instance, best fit first (what swap would keep / lose)."""
        iid, pn = _split(target)
        inst = self._inst(self.doc, iid)
        part = self._part(inst["part"])
        return (
            self._suggest_port(iid, part, part.port(pn), limit)
            if pn
            else self._suggest_swap(inst, part, limit)
        )

    def _suggest_port(self, iid, part: Part, port: dict, limit) -> list:
        by_id = {d["id"]: d for d in self.doc["instances"]}
        kind = port.get("kind")
        # categories / parts seen on ports of this type in the assembly (the occupant of this very port is the stock part)
        cats, stock = set(), set()
        for m in self.doc.get("mates", []):
            for e, o in (("a", "b"), ("b", "a")):
                ei, ep = _split(m[e])
                oi, op = _split(m[o])
                if ei not in by_id or oi not in by_id:
                    continue
                pp = self._part(by_id[ei]["part"])
                try:
                    q = pp.port(ep) if ep else None
                except DesignError:
                    q = None
                if (
                    q
                    and pp.category == part.category
                    and types_ok(q.get("type", ""), port.get("type", ""))
                ):
                    other = self._part(by_id[oi]["part"])
                    cats.add(other.category)
                    if (ei, ep) == (iid, port["name"]):
                        stock.add(other.id)
        top = {c.split(".")[0] for c in cats}
        rows = []
        for pid, cand in sorted(self.lib.parts.items()):
            for q in cand.ports:
                if q.get("kind") != kind:
                    continue
                row = self._score_port(port, q, kind)
                if row is None:
                    continue
                s, why = row
                if cand.category in cats:
                    s += W_CATEGORY
                    why.append(
                        f"category {cand.category} is used on {port.get('type')} ports here"
                    )
                elif cand.category.split(".")[0] in top:
                    s += W_FAMILY_CAT
                    why.append("same part family as the neighbours on this port type")
                if cand.category == part.category:
                    s -= W_CATEGORY  # a peer of the part that owns the port is rarely what plugs into it
                    why.append(f"same category as {part.id}")
                if pid in stock:
                    s += W_STOCK
                    why.append("stock part (currently on this port)")
                rows.append(
                    {
                        "part": pid,
                        "port": q["name"],
                        "category": cand.category,
                        "score": round(s, 2),
                        "why": why,
                        "mass": cand.value("mass"),
                        "name": cand.raw.get("name", ""),
                    }
                )
        rows.sort(key=lambda r: (-r["score"], r["part"], r["port"]))
        return rows[:limit]

    def _score_port(self, port, q, kind):
        if kind == MECH:
            r = self._pairs([port], [q])
            return (r[0]["score"], list(r[0]["why"])) if r else None
        if not (
            types_ok(port.get("type", ""), q.get("type", ""))
            and genders_ok(port.get("gender", ""), q.get("gender", ""))
        ):
            return None
        ra, rb = port.get("role"), q.get("role")
        if ra == rb and ra in ("source", "sink"):
            return None  # source-source / sink-sink cannot be one net
        s = (W_EXACT if port.get("type") == q.get("type") else W_FAMILY) + W_NEUTRAL
        why = [
            "same port type" if port.get("type") == q.get("type") else "paired family"
        ]
        # ratings: the sink must tolerate the source's voltage; the weaker current rating limits the path
        src, snk = (port, q) if ra == "source" or rb == "sink" else (q, port)
        vmax_src, vmax_snk = _num(src.get("v_max")), _num(snk.get("v_max"))
        if vmax_src and vmax_snk and vmax_snk < vmax_src:
            s += W_RATING_BAD
            why.append(f"v_max {vmax_snk} V below the source's {vmax_src} V")
        im_t, im_q = _num(port.get("i_max")), _num(q.get("i_max"))
        if im_t and im_q and im_q < im_t:
            s += W_RATING_BAD
            why.append(f"i_max {im_q} A below this port's {im_t} A")
        return s, why

    def _suggest_swap(self, inst, old: Part, limit) -> list:
        pool = [
            p
            for p in self.lib.parts.values()
            if p.id != old.id and p.category == old.category
        ]
        fam = old.category.split(".")[0]
        pool += [
            p
            for p in self.lib.parts.values()
            if p.id != old.id
            and p.category != old.category
            and p.category.split(".")[0] == fam
        ]
        rows = []
        for cand in pool:
            doc = copy.deepcopy(self.doc)
            try:
                ch = self._op_swap(doc, {"id": inst["id"], "part": cand.id})
            except DesignError:
                continue
            md = None
            if cand.value("mass") is not None and old.value("mass") is not None:
                md = round(cand.value("mass") - old.value("mass"), 6)
            deficits = []
            for k in ch["mates"]["kept"] + ch["wires"]["kept"]:
                try:
                    po, pn_ = old.port(k["old_port"]), cand.port(k["new_port"])
                except DesignError:
                    continue
                for key in ("v_max", "i_max", "i_burst"):
                    a, b = _num(po.get(key)), _num(pn_.get(key))
                    if a and b is not None and b < a:
                        deficits.append(f"{k['new_port']}.{key} {b} < {a}")
            lost = len(ch["mates"]["lost"]) + len(ch["wires"]["lost"])
            same_cat = cand.category == old.category
            tables = set(old.raw.get("tables", {})) - set(cand.raw.get("tables", {}))
            rows.append(
                {
                    "part": cand.id,
                    "category": cand.category,
                    "name": cand.raw.get("name", ""),
                    "mates_kept": len(ch["mates"]["kept"]),
                    "mates_lost": [m["mate"] for m in ch["mates"]["lost"]],
                    "wires_kept": len(ch["wires"]["kept"]),
                    "wires_lost": [w["wire"] for w in ch["wires"]["lost"]],
                    "dangling": ch["dangling"],
                    "rating_deficits": deficits,
                    "missing_tables": sorted(tables),
                    "mass_delta": md,
                    "_k": (
                        not same_cat,
                        lost,
                        len(ch["dangling"]),
                        len(deficits),
                        len(tables),
                        abs(md) if md is not None else 1e9,
                        cand.id,
                    ),
                }
            )
        rows.sort(key=lambda r: r["_k"])
        for r in rows:
            del r["_k"]
        return rows[:limit]


def _didyoumean(x, names, what="") -> str:
    m = difflib.get_close_matches(x, names, n=3)
    return f" (did you mean {', '.join(m)}?)" if m else ""
