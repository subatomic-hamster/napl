"""The viewer's 2D "flatten" wiring mode, as design_api ops (the viewer's `src/viewer/wire2d.cpp` draws what these return; a
scripted agent drives the same ops through `Backend`, so a recording shows the UI reacting).

    flatten.begin   {instances: [id], ports?: {id: [port]} | ["id.port"]}   select components + the terminals to show; opens the canvas
    flatten.layout  {}                                                      auto-layout: signal flow left to right (see `layout`)
    flatten.move_card {id, x, y}                                            move a card (canvas units)
    flatten.wire    {from, to, awg?, length?}                      drag pin -> pin: the `autowire` op (kind, connector type, gender,
                                                                            voltage rules); a refusal comes back as {"ok": false, "error": reason}.
                                                                            With `part` (+ length, conductors?, included_in?, name?) it is an
                                                                            explicit `wire` op instead: a lead the user chose, bridging unlike connectors
    flatten.set_wire {name, awg?, length?}                                  change a wire's gauge / length (unwire + wire in one edit)
    flatten.unwire  {name}
    flatten.state   {}                                                      the canvas state (cards, pins, wires)
    flatten.end     {}                                                      back to 3D: the wires are already in the design (tubes)

Every reply is {"ok": true, "result": {...}, "flatten": <state>, "state": <design state when the design changed>}.
Canvas state: {"active", "rev", "cards": [{id, part, group, x, y, w, h, band, layer, pins: [{name, kind, type, gender, role, side, row,
wired}]}], "wires": [{name, from, to, part, awg, length, kind, conductors, route: [[x, y], ...], label: {text, x, y, w, h} | null}],
"hidden_wires": n}. `route` = orthogonal polyline from the `from` pin to the `to` pin that stays out of every card
(wire2d_route.py); `label` = where the gauge text goes (centre), null when no free spot is left.

Layout (`layout`): a layered signal-flow drawing. Power band on top: sources (batteries, supplies) in the left layer, then power
modules / PDBs / BECs, then ESCs / controllers / compute, then loads (motors, servos) on the right; every wire pushes its target at
least one layer right of its source. Cards that carry no power terminal (receivers, GPS, sensors) form a second band below with
their own layers, so signal buses stay out of the power flow. Inside a layer the order minimises crossings (barycentre sweeps).
A pin sits on the card side facing its peers (wire targets to the right -> right side), else by its role (source right, sink left).
Card width is sized to the text of its pins so labels never overlap.
"""

from .library import DesignError
from .wire2d_route import place_labels, route_wires
from .wiring import wired_ports

HEAD_H, ROW_H, PAD, COL_GAP, ROW_GAP, BAND_GAP = 30, 22, 10, 170, 30, 90
MIN_W = 190
# text width estimates in canvas units (Roboto 15 px: head 15 px, pin name 13.8 px, pin type 10.8 px, part id 11.7 px), +8 % margin
CW_HEAD, CW_PART, CW_NAME, CW_TYPE, PIN_PAD, MID_GAP = 8.4, 6.6, 7.6, 6.0, 30, 26
WIRE_KINDS = ("power", "signal")
# category prefix -> layer rank (0 = sources on the left ... 3 = loads on the right); longest prefix wins
RANKS = {
    "battery": 0,
    "psu": 0,
    "psu.dcdc": 1,
    "pcb.pm": 1,
    "pcb.pdb": 1,
    "pcb.bec": 1,
    "connector": 1,
    "esc": 2,
    "driver": 2,
    "computer": 2,
    "pcb.fc": 2,
    "pcb.motor_controller": 2,
    "pcb.servo_driver": 2,
    "motor": 3,
    "actuator": 3,
    "effector": 3,
    "wheel": 3,
    "sensor": 0,
    "rc": 0,
    "radio": 0,
    "pcb.gps": 0,
}


def _rank(cat, roles):
    best = None
    for pre, r in RANKS.items():
        if (cat == pre or cat.startswith(pre + ".")) and (
            best is None or len(pre) > len(best[0])
        ):
            best = (pre, r)
    if best:
        return best[1]
    if roles and set(roles) == {"source"}:
        return 0
    if roles and set(roles) == {"sink"}:
        return 3
    return 1


class Flatten:
    def __init__(self):
        self.active = False
        self.touched = (
            False  # a card was moved by hand: wiring then leaves the placement alone
        )
        self.rev = 0
        self.cards = {}  # id -> {"id", "x", "y", "pins": [port names], "sides", "w", "h", "band", "layer"}
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
        if changed and self.active and not self.touched:
            self._layout_cards(
                b, d
            )  # no card was placed by hand: the drawing follows the wiring
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
        self.cards, self.active, self.touched = cards, True, False
        self._layout_cards(b, d)
        return {"cards": len(cards)}, False

    def _layout(self, b, d, a):
        self._need()
        self.touched = False
        self._layout_cards(b, d)
        return {"cards": len(self.cards)}, False

    def _move_card(self, b, d, a):
        self._need()
        c = self.cards.get(a.get("id"))
        if c is None:
            raise DesignError(f"flatten.move_card: no card '{a.get('id')}'")
        c["x"], c["y"] = float(a["x"]), float(a["y"])
        self.touched = True
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
        if a.get(
            "part"
        ):  # an explicit lead (the user picked the wire part): bridges unlike connector families, which autowire refuses
            op = {
                "op": "wire",
                "from": a["from"],
                "to": a["to"],
                "part": a["part"],
                "length": a["length"],
            }
            op.update(
                {
                    k: a[k]
                    for k in ("name", "conductors", "included_in")
                    if a.get(k) is not None
                }
            )
        elif a.get("awg") is not None:
            op["part"] = self._wire_part(d, a["awg"]).id
        if a.get("length") is not None and not a.get("part"):
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

    # ---- layout --------------------------------------------------------------------------------------------------------------
    def _edges(self, d, defs):
        """[(from card, to card, from pin, to pin)] of the wires between shown pins, oriented source -> sink by the pins' roles."""
        out = []
        for w in d.doc.get("wires", []):
            (fi, _, fp), (ti, _, tp) = w["from"].partition("."), w["to"].partition(".")
            if fi == ti or fi not in self.cards or ti not in self.cards:
                continue
            if fp not in self.cards[fi]["pins"] or tp not in self.cards[ti]["pins"]:
                continue
            rf, rt = defs[fi][fp].get("role"), defs[ti][tp].get("role")
            if (
                rf == "sink"
                and rt in ("source", "passthrough")
                or rf == "passthrough"
                and rt == "source"
            ):
                fi, ti, fp, tp = ti, fi, tp, fp
            out.append((fi, ti, fp, tp))
        return out

    def _layout_cards(self, b, d):
        ids = sorted(self.cards)
        defs = {i: self._port_defs(d, i) for i in ids}
        inst = {i["id"]: i for i in d.doc["instances"]}
        cat = {i: d._part(inst[i]["part"]).category for i in ids}
        roles = {
            i: [defs[i][p].get("role") for p in self.cards[i]["pins"]] for i in ids
        }
        band = {
            i: 0
            if any(defs[i][p].get("kind") == "power" for p in self.cards[i]["pins"])
            else 1
            for i in ids
        }
        edges = [e for e in self._edges(d, defs) if band[e[0]] == band[e[1]]]
        layer = {i: _rank(cat[i], roles[i]) for i in ids}
        for _ in range(
            len(ids)
        ):  # relaxation: a wire's target sits right of its source; capped, so a cycle just stops moving
            moved = False
            for f, t, _, _ in edges:
                if layer[t] <= layer[f] and layer[f] + 1 <= 2 * len(ids):
                    layer[t], moved = layer[f] + 1, True
            if not moved:
                break
        cols = {}  # (band, layer) -> [ids]
        for bd in (0, 1):  # compress empty layers per band
            used = sorted({layer[i] for i in ids if band[i] == bd})
            for i in ids:
                if band[i] == bd:
                    layer[i] = used.index(layer[i])
        for i in ids:
            cols.setdefault((band[i], layer[i]), []).append(i)
        nbrs = {i: [] for i in ids}
        for f, t, _, _ in edges:
            nbrs[f].append(t)
            nbrs[t].append(f)
        order = {i: k for col in cols.values() for k, i in enumerate(col)}
        for sweep in range(4):  # barycentre ordering, down then up
            keys = sorted(cols, key=lambda k: (k[0], k[1] if sweep % 2 == 0 else -k[1]))
            for key in keys:
                col = cols[key]

                def bary(i, key=key):
                    ps = [order[n] for n in nbrs[i] if (band[n], layer[n]) != key]
                    return sum(ps) / len(ps) if ps else order[i]

                col.sort(key=lambda i: (bary(i), i))
                for k, i in enumerate(col):
                    order[i] = k
        # sides: facing the peers (by layer), else the role
        peers = {i: {} for i in ids}  # card -> pin -> [peer cards]
        for f, t, fp, tp in edges:
            peers[f].setdefault(fp, []).append(t)
            peers[t].setdefault(tp, []).append(f)
        for w in d.doc.get(
            "wires", []
        ):  # wires across bands / to unoriented ends still pull a pin towards their peer
            for e, o in (("from", "to"), ("to", "from")):
                (ci, _, cp), (oi, _, _) = w[e].partition("."), w[o].partition(".")
                if (
                    ci in self.cards
                    and oi in self.cards
                    and ci != oi
                    and cp in self.cards[ci]["pins"]
                    and oi not in peers[ci].get(cp, [])
                ):
                    peers[ci].setdefault(cp, []).append(oi)
        for i in ids:
            c = self.cards[i]
            left, right = [], []
            for p in c["pins"]:
                role = defs[i][p].get("role")
                ps = peers[i].get(p, [])
                dx = sum((layer[o] - layer[i]) if band[o] == band[i] else 0 for o in ps)
                if dx > 0:
                    side = "right"
                elif dx < 0:
                    side = "left"
                elif role == "source":
                    side = "right"
                elif role == "sink":
                    side = "left"
                else:
                    side = "left" if len(left) <= len(right) else "right"
                (left if side == "left" else right).append(p)
            c["_left"], c["_right"] = left, right
            nr = max(1, len(left), len(right))
            nm = lambda p: (
                len(p) * CW_NAME + len(defs[i][p].get("type", "")) * CW_TYPE + PIN_PAD
            )
            pin_w = (
                max((nm(p) for p in left), default=0)
                + max((nm(p) for p in right), default=0)
                + MID_GAP
            )
            head_w = max(len(i) * CW_HEAD, len(inst[i]["part"]) * CW_PART) + 24
            c["w"], c["h"] = (
                float(max(MIN_W, pin_w, head_w)),
                float(HEAD_H + PAD + nr * ROW_H),
            )
            c["band"], c["layer"] = band[i], layer[i]
        # positions: columns left to right, cards stacked and centred on a common mid line per band
        x, top = 40.0, 30.0
        for bd in (0, 1):
            keys = sorted(k for k in cols if k[0] == bd)
            if not keys:
                continue
            heights = {
                k: sum(self.cards[i]["h"] for i in cols[k])
                + ROW_GAP * (len(cols[k]) - 1)
                for k in keys
            }
            tall = max(heights.values())
            x = 40.0
            for k in keys:
                cw = max(self.cards[i]["w"] for i in cols[k])
                y = top + (tall - heights[k]) / 2
                for i in cols[k]:
                    c = self.cards[i]
                    c["x"], c["y"] = x + (cw - c["w"]) / 2, y
                    y += c["h"] + ROW_GAP
                x += cw + COL_GAP
            top += tall + BAND_GAP
        # pin rows: ordered by the height of their peers (fewer crossings), then their name order
        cy = {i: self.cards[i]["y"] + self.cards[i]["h"] / 2 for i in ids}
        for i in ids:
            c = self.cards[i]
            c["sides"] = {}
            for side, names in (("left", c.pop("_left")), ("right", c.pop("_right"))):

                def key(p):
                    ps = peers[i].get(p, [])
                    return (
                        sum(cy[o] for o in ps) / len(ps) if ps else cy[i],
                        c["pins"].index(p),
                    )

                for r, p in enumerate(sorted(names, key=key)):
                    c["sides"][p] = (side, r)

    # ---- state ---------------------------------------------------------------------------------------------------------------
    def state(self, b, d) -> dict:
        cards, shown = [], set()
        wired = wired_ports(d.doc)
        for c in self.cards.values():
            defs = self._port_defs(d, c["id"])
            sides = c["sides"]
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
                    "w": c["w"],
                    "h": c["h"],
                    "band": c["band"],
                    "layer": c["layer"],
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
        routes = route_wires(cards, wires)
        labels = place_labels(cards, wires, routes)
        for w in wires:
            w["route"], w["label"] = routes[w["name"]], labels.get(w["name"])
        return {
            "active": self.active,
            "rev": self.rev,
            "cards": cards,
            "wires": wires,
            "hidden_wires": hidden,
        }
