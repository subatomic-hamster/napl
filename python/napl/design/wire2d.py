"""The viewer's 2D "flatten" wiring mode, as design_api ops (the viewer's `src/viewer/wire2d.cpp` draws what these return; a
scripted agent drives the same ops through `Backend`, so a recording shows the UI reacting).

    flatten.begin   {instances: [id], ports?: {id: [port]} | ["id.port"]}   select components + the terminals to show; opens the canvas
    flatten.layout  {}                                                      auto-layout the cards (wire direction left to right)
    flatten.move_card {id, x, y}                                            move a card (canvas units)
    flatten.wire    {from, to, awg?, length?}                      drag pin -> pin: the `autowire` op (kind, connector type, gender,
                                                                            voltage rules); a refusal comes back as {"ok": false, "error": reason}
    flatten.set_wire {name, awg?, length?}                                  change a wire's gauge / length (unwire + wire in one edit)
    flatten.unwire  {name}
    flatten.state   {}                                                      the canvas state (cards, pins, wires)
    flatten.end     {}                                                      back to 3D: the wires are already in the design (tubes)

Every reply is {"ok": true, "result": {...}, "flatten": <state>, "state": <design state when the design changed>}.
Canvas state: {"active", "rev", "cards": [{id, part, group, x, y, w, h, pins: [{name, kind, type, gender, role, side, row, wired}]}],
"wires": [{name, from, to, part, awg, length, kind, conductors}], "hidden_wires": n}.
"""

from .library import DesignError
from .wiring import wired_ports

CARD_W, HEAD_H, ROW_H, PAD, COL_GAP, ROW_GAP, MAX_COL_H = 200, 30, 22, 10, 150, 28, 820
WIRE_KINDS = ("power", "signal")


class Flatten:
    def __init__(self):
        self.active = False
        self.rev = 0
        self.cards = {}  # id -> {"id", "x", "y", "pins": [port names]}
        self.msg = ""

    # ---- entry ---------------------------------------------------------------------------------------------------------------
    def handle(self, b, d, tool, a) -> dict:
        op = tool.partition(".")[2]
        fn = getattr(self, "_" + op, None)
        if fn is None or op.startswith("_"):
            raise DesignError(
                f"unknown flatten op {tool!r} (begin layout move_card wire set_wire unwire state end)"
            )
        res, changed = fn(b, d, a or {})
        self.rev += 0 if op == "state" else 1
        out = {"ok": True, "result": res, "flatten": self.state(b, d)}
        if changed:
            out["state"] = b.state(d)
        return out

    # ---- ops -----------------------------------------------------------------------------------------------------------------
    def _begin(self, b, d, a):
        ids = list(a.get("instances") or [])
        if not ids:
            raise DesignError(
                "flatten.begin: pass instances = [ids] (select components first)"
            )
        by_id = {i["id"]: i for i in d.doc["instances"]}
        for i in ids:
            if i not in by_id:
                raise DesignError(f"flatten.begin: unknown instance '{i}'")
        want = a.get("ports")
        if isinstance(want, list):  # ["inst.port", ...]
            m = {}
            for r in want:
                i, _, p = r.partition(".")
                m.setdefault(i, []).append(p)
            want = m
        cards = {}
        for i in ids:
            part = d._part(by_id[i]["part"])
            names = [p["name"] for p in part.ports if p.get("kind") in WIRE_KINDS]
            if want and i in want:
                bad = [p for p in want[i] if p not in names]
                if bad:
                    raise DesignError(
                        f"flatten.begin: {i} has no wire port(s) {bad}; wire ports: {names}"
                    )
                names = [p for p in names if p in want[i]]
            cards[i] = {"id": i, "x": 0.0, "y": 0.0, "pins": names}
        self.cards, self.active = cards, True
        self._layout_cards(b, d)
        return {"cards": len(cards)}, False

    def _layout(self, b, d, a):
        self._need()
        self._layout_cards(b, d)
        return {"cards": len(self.cards)}, False

    def _move_card(self, b, d, a):
        self._need()
        c = self.cards.get(a.get("id"))
        if c is None:
            raise DesignError(f"flatten.move_card: no card '{a.get('id')}'")
        c["x"], c["y"] = float(a["x"]), float(a["y"])
        return {"id": c["id"], "x": c["x"], "y": c["y"]}, False

    def _wire(self, b, d, a):
        self._need()
        for e in ("from", "to"):
            i, _, p = str(a.get(e, "")).partition(".")
            if i not in self.cards or p not in self.cards[i]["pins"]:
                raise DesignError(
                    f"flatten.wire: {e} '{a.get(e)}' is not a pin on the canvas (flatten.begin chose the terminals)"
                )
        before = {w.get("name") for w in d.doc.get("wires", [])}
        op = {"op": "autowire", "a": a["from"], "b": a["to"]}
        if a.get("awg") is not None:
            op["part"] = self._wire_part(d, a["awg"]).id
        if a.get("length") is not None:
            op["length"] = self._length(a["length"])
        r = d.edit([op])
        new = [w["name"] for w in d.doc.get("wires", []) if w.get("name") not in before]
        return {**r.as_dict(), "wires": new, "undo_depth": len(d._undo)}, True

    def _set_wire(self, b, d, a):
        self._need()
        w = next(
            (x for x in d.doc.get("wires", []) if x.get("name") == a.get("name")), None
        )
        if w is None:
            raise DesignError(f"flatten.set_wire: no wire named '{a.get('name')}'")
        new = dict(w)
        if a.get("awg") is not None:
            new["part"] = self._wire_part(d, a["awg"]).id
        if a.get("length") is not None:
            new["length"] = self._length(a["length"])
        r = d.edit([{"op": "unwire", "name": w["name"]}, {"op": "wire", **new}])
        return {**r.as_dict(), "wires": [w["name"]]}, True

    def _unwire(self, b, d, a):
        self._need()
        r = d.edit([{"op": "unwire", "name": a.get("name")}])
        return r.as_dict(), True

    def _state(self, b, d, a):
        return {}, False

    def _end(self, b, d, a):
        self.active, self.cards = False, {}
        return {}, False

    # ---- helpers -------------------------------------------------------------------------------------------------------------
    def _need(self):
        if not self.active:
            raise DesignError("flatten: not active (flatten.begin first)")

    @staticmethod
    def _wire_part(d, awg):
        parts = {
            int(p.value("awg")): p
            for p in d.lib.by_category("wire")
            if p.value("awg") is not None
        }
        if int(awg) not in parts:
            raise DesignError(f"no wire part with AWG {awg}; gauges: {sorted(parts)}")
        return parts[int(awg)]

    @staticmethod
    def _length(x):
        if isinstance(x, dict):
            return x
        return {
            "value": float(x),
            "unit": "m",
            "fidelity": "estimated",
            "rationale": "length set in the 2D wiring view",
        }

    def _port_defs(self, d, iid):
        inst = next(i for i in d.doc["instances"] if i["id"] == iid)
        return {p["name"]: p for p in d._part(inst["part"]).ports}

    def _sides(self, d, c):
        """Pin -> (side, row): sources on the right, sinks on the left, the rest balance the two sides."""
        defs = self._port_defs(d, c["id"])
        left, right = [], []
        for n in c["pins"]:
            role = defs[n].get("role")
            (
                right
                if role == "source"
                else left
                if role == "sink"
                else (left if len(left) <= len(right) else right)
            ).append(n)
        return {
            **{n: ("left", k) for k, n in enumerate(left)},
            **{n: ("right", k) for k, n in enumerate(right)},
        }

    def _size(self, d, c):
        rows = max(1, max((r + 1 for _, r in self._sides(d, c).values()), default=1))
        return CARD_W, HEAD_H + PAD + rows * ROW_H

    def _layout_cards(self, b, d):
        """Columns by wire direction (a wire's `to` card sits right of its `from` card), unwired cards fill columns top-down."""
        ids = sorted(self.cards)
        layer = {i: 0 for i in ids}
        edges = [
            (w["from"].partition(".")[0], w["to"].partition(".")[0])
            for w in d.doc.get("wires", [])
            if w["from"].partition(".")[0] in layer
            and w["to"].partition(".")[0] in layer
        ]
        for _ in range(len(ids)):  # relaxation; capped, so a cycle just stops moving
            moved = False
            for f, t in edges:
                if f != t and layer[t] < layer[f] + 1 and layer[f] + 1 < len(ids):
                    layer[t], moved = layer[f] + 1, True
            if not moved:
                break
        wired = {i for e in edges for i in e}
        col, y, x = 0, 0.0, 40.0
        order = sorted(ids, key=lambda i: (i not in wired, layer[i], i))
        last_layer = None
        for i in order:
            w, h = self._size(d, self.cards[i])
            key = layer[i] if i in wired else -1
            if (last_layer is not None and key != last_layer and key != -1) or (
                y > 0 and y + h > MAX_COL_H
            ):
                col, y = col + 1, 0.0
            if last_layer == -1 and key != -1:
                col, y = col + 1, 0.0
            last_layer = key
            self.cards[i]["x"], self.cards[i]["y"] = (
                x + col * (CARD_W + COL_GAP),
                30.0 + y,
            )
            y += h + ROW_GAP

    # ---- state ---------------------------------------------------------------------------------------------------------------
    def state(self, b, d) -> dict:
        cards, shown = [], set()
        wired = wired_ports(d.doc)
        for c in self.cards.values():
            defs = self._port_defs(d, c["id"])
            sides = self._sides(d, c)
            w, h = self._size(d, c)
            part = next(i["part"] for i in d.doc["instances"] if i["id"] == c["id"])
            grp = next(
                i.get("group", "") for i in d.doc["instances"] if i["id"] == c["id"]
            )
            pins = []
            for n in c["pins"]:
                p = defs[n]
                shown.add(f"{c['id']}.{n}")
                pins.append(
                    {
                        "name": n,
                        "kind": p.get("kind", ""),
                        "type": p.get("type", ""),
                        "gender": p.get("gender", ""),
                        "role": p.get("role", ""),
                        "side": sides[n][0],
                        "row": sides[n][1],
                        "wired": (c["id"], n) in wired,
                    }
                )
            cards.append(
                {
                    "id": c["id"],
                    "part": part,
                    "group": grp,
                    "x": c["x"],
                    "y": c["y"],
                    "w": w,
                    "h": h,
                    "pins": pins,
                }
            )
        rep = []
        try:
            rep = b.state(d)["report"].get("wires", [])
        except Exception:  # a design that does not build has no lengths
            pass
        length = {r["name"]: r.get("length") for r in rep}
        wires, hidden = [], 0
        for w in d.doc.get("wires", []):
            if w["from"] in shown and w["to"] in shown:
                part = d._part(w["part"])
                kind = (
                    d._part(
                        next(
                            i["part"]
                            for i in d.doc["instances"]
                            if i["id"] == w["from"].partition(".")[0]
                        )
                    )
                    .port(w["from"].partition(".")[2])
                    .get("kind", "")
                )
                wires.append(
                    {
                        "name": w.get("name", f"{w['from']}--{w['to']}"),
                        "from": w["from"],
                        "to": w["to"],
                        "part": w["part"],
                        "awg": part.value("awg"),
                        "length": length.get(w.get("name")),
                        "kind": kind,
                        "conductors": w.get("conductors", 1),
                    }
                )
            elif w["from"] in shown or w["to"] in shown:
                hidden += 1
        return {
            "active": self.active,
            "rev": self.rev,
            "cards": cards,
            "wires": wires,
            "hidden_wires": hidden,
        }
