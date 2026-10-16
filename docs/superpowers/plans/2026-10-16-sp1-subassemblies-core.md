# SP1-core: sub-assemblies (schema, flatten, mirror, BOM, Backend ops) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** an assembly can include other assemblies as linked, mirrorable, overridable sub-assemblies, flattened at load so the
whole downstream pipeline is unchanged; the build report gains a BOM; the Backend exposes `sub.*` ops.

**Architecture:** new module `python/napl/design/subasm.py` does the expansion (pure function doc -> flat doc + origins);
`assembly.load/loads` call it before `validate`; `assembly.dumps` writes only the unexpanded parent. Mirror is a transform
of placed poses and part swaps applied while expanding, with a post-build pose check. BOM is a pure function of the build
report + library (`python/napl/design/bom.py`). Backend ops live in `python/napl/design/edit.py` (Design) and
`python/napl/design/viewer.py` (Backend dispatch), following the existing `_op_*` pattern.

**Tech Stack:** Python 3.12, tomllib, numpy, pytest; existing napl.design modules.

**Spec:** `docs/superpowers/specs/2026-10-16-quadruped-design.md` (section 3).

## Global Constraints

- Numbers rule unchanged: every numeric leaf is `"<inst>.<param>"` or a provenance table (assembly.py docstring).
- Every existing test and golden state hash unchanged (`ctest --preset dev`, full pytest, `tests/python/test_*_flights.py`).
- No new dependencies. No GPL. Edit files only with Edit/Write tools.
- Compute on hfbox via `tools/remote.sh` with `NAPL_REMOTE_DIR` set (see the worker briefing); laptop = edit + git only.
- Flattened id separator is `:`; prefix depth unbounded; MJCF names and prim paths keep the colon.

## Review Focus

1. A child whose device references a mate name or device_map (not only instance ids) — the prefix must reach every
   reference field of every device type; a missed one silently binds to a parent object of the same name. Test: table-driven,
   fails if any `DEVICE_TYPES` entry lacks a reference-field spec (Task 2).
2. Two sub-assemblies of the same file (4 legs) — no shared mutable dicts between copies (deepcopy); editing one override
   must not change another. Test in Task 3.
3. A child file edited on disk while a parent is open — `Design.reload_children()` must pick it up; stale cache = wrong build.
   Test in Task 6.
4. Relative `file` paths when the parent is loaded from another cwd or via `loads(text, path=None)` — `loads` without a
   path and with `[[subassemblies]]` must raise a clear error, not resolve against cwd. Test in Task 1.
5. Saving a parent after `sub.override` must not write flattened instances into the parent TOML. Test in Task 6.

---

### Task 1: schema keys + load-time expansion skeleton

**Files:**
- Create: `python/napl/design/subasm.py`
- Modify: `python/napl/design/assembly.py` (TOP/ALLOWED, `loads`, `load`, `dumps`, `Assembly.__init__` keeps `raw` doc)
- Test: `tests/python/test_subasm.py`

**Interfaces:**
- Produces: `subasm.expand(doc: dict, path: Path | None, lib_ids: set[str] | None = None, _stack=()) -> tuple[dict, dict]`
  returning `(flat_doc, origins)` where `origins[flat_id] = {"sub": "fl", "file": "leg.toml", "id": "hip_motor"}` for every
  flattened instance / mate / wire / device / device_map / target name.
- Produces: `Assembly.raw` (the unexpanded doc as written), `Assembly.doc` (flattened), `Assembly.origins`.
- `TOP` gains `"exports"`, `"subassemblies"`; `ALLOWED["subassemblies"] = {"id","file","pos","rpy","mirror","params","group","color"}`;
  `exports` keys `{"ports","connectors","params"}`.

- [ ] **Step 1: failing tests** (fixtures written by the test into `tmp_path`, using real library parts already used by
  `assemblies/rover.toml`, e.g. the rover plate/bracket/motor so the leg builds):

```python
def write(p, text): p.write_text(textwrap.dedent(text)); return p

def test_expand_prefixes_and_origins(tmp_path):
    write(tmp_path / "leg.toml", LEG)          # 3 instances, 2 mates, [exports] ports.mount = "bracket.base"
    parent = write(tmp_path / "bot.toml", BOT)  # plate root + 2 [[subassemblies]] l, r + 2 mates to "l:mount", "r:mount"
    asm = A.load(parent)
    ids = [i["id"] for i in asm.doc["instances"]]
    assert ids.count("plate") == 1 and "l:bracket" in ids and "r:motor" in ids
    assert asm.origins["l:motor"] == {"sub": "l", "file": "leg.toml", "id": "motor"}
    assert not any(i.get("root") for i in asm.doc["instances"] if ":" in i["id"])
    assert "subassemblies" not in asm.doc and asm.raw["subassemblies"][0]["id"] == "l"

def test_loads_without_path_rejects_subassemblies():
    with pytest.raises(DesignError, match="needs the parent file's path"):
        A.loads(BOT_TEXT)

def test_cycle(tmp_path):
    write(tmp_path / "a.toml", INCLUDES_B); write(tmp_path / "b.toml", INCLUDES_A)
    with pytest.raises(DesignError, match=r"cycle: a.toml -> b.toml -> a.toml"):
        A.load(tmp_path / "a.toml")

def test_reach_inside_is_error(tmp_path):
    ...  # parent mate b = "l:bracket.base" -> DesignError mentioning "export it in leg.toml [exports]"
```

- [ ] **Step 2:** run `tools/remote.sh run "cd src && uv run pytest tests/python/test_subasm.py -q"` → FAIL (module missing).
- [ ] **Step 3: implement** `expand`: read each child with `tomllib`, recurse (stack of resolved paths for cycles), deepcopy
  child sections, prefix ids, drop `root`, translate parent references `"<sub>:<export>"` through the child's exports
  (ports → `"<sub>:<inst>.<port>"`, connectors → `"<sub>:<inst>.<conn>"`), reject `"<sub>:<inst>.<x>"` from the parent.
  `validate` accepts `:` in ids only for ids present in `origins` (keep the hand-written regex for written ids).
  `dumps` writes `asm.raw`.
- [ ] **Step 4:** tests pass; full `tests/python/test_assembly*.py` + `test_build*.py` still pass.
- [ ] **Step 5:** commit `subasm: [[subassemblies]] + [exports], flatten on load`.

### Task 2: reference rewriting for every section and device type

**Files:** Modify `python/napl/design/subasm.py`; Test `tests/python/test_subasm.py`.

**Interfaces:** Produces `subasm.REF_FIELDS: dict[str, tuple[str, ...]]` — per device `type`, the fields holding instance
ids / mate (joint) names / device-map names / body names; plus fixed tables for mates (`a`, `b`, `fasteners`, `nut`),
wires (`from`, `to`, `length`, `included_in`, connectors pins), targets (`groups` are group labels: prefixed only when the
group is the child's own; `value` refs), and any `"<inst>.<param>"` string anywhere (numbers rule).

- [ ] **Step 1: failing tests:**

```python
def test_every_device_type_has_ref_fields():
    assert set(subasm.REF_FIELDS) >= A.DEVICE_TYPES

def test_device_refs_prefixed(tmp_path):
    ...  # child with a servo device {instance, motor, joint, map} + a device_map; after expand every field starts "l:"
    #     and build(compute) of the parent succeeds with devices bound to the l: bodies (report devices list)

def test_param_refs_prefixed(tmp_path):
    ...  # child wire length = "harness.length" -> "l:harness.length"; child override of an exported param resolves
```

- [ ] **Step 2–4:** derive the table by reading `devices.py::build_devices` and `auto_devices.py` for every type; implement;
  tests pass.
- [ ] **Step 5:** commit `subasm: prefix every reference (devices, maps, wires, targets, param refs)`.

### Task 3: exported-param overrides + independence of copies

**Files:** Modify `subasm.py`; Test `tests/python/test_subasm.py`.

- [ ] **Step 1: failing tests:**

```python
def test_override_applies_to_one_copy(tmp_path):
    ...  # parent l overrides cable_len (exported as "harness.length"); r does not
    asm = A.load(parent)
    inst = {i["id"]: i for i in asm.doc["instances"]}
    assert inst["l:harness"]["params"]["length"]["value"] == 0.32
    assert "length" not in inst["r:harness"].get("params", {})

def test_override_of_unexported_param(tmp_path):
    with pytest.raises(DesignError, match="exports: cable_len"):
        ...
```

- [ ] **Step 2–4:** implement (override → instance `params` entry of the target instance; provenance checked by the existing
  `check_provenance`). Commit `subasm: exported param overrides`.

### Task 4: split-equivalence (the safety net)

**Files:** Create `tests/data/subasm/rover_split/{wheel.toml,rover.toml}` (rover split into a wheel-module child used twice,
right one mirrored? no — non-mirrored placement via mates, mirror comes in Task 5); Test `tests/python/test_subasm_equiv.py`.

- [ ] **Step 1: failing test:**

```python
def test_split_rover_builds_identical(tmp_path):
    a = build.compute(A.load(ROOT / "assemblies/rover.toml"), PARTS, tmp_path / "a")[0]
    b = build.compute(A.load(DATA / "rover_split/rover.toml"), PARTS, tmp_path / "b")[0]
    assert canonical(a) == canonical(b)  # names differ only by prefix: canonical() maps "wl:motor" -> "motor_l" via a table
    assert hash_after(a, 2000) == hash_after(b, 2000)  # napl.Sim from each MJCF, same seed, 2000 ticks, state_hash equal
```

  (If body order differs, MJCF text differs but physics must not: the hash comparison is the binding check; `canonical` is a
  diagnostic.) If the hash differs because of body ordering, make `expand` place child instances at the position of the
  `[[subassemblies]]` entry in the parent order, and document that rule.
- [ ] **Step 2–4:** iterate until equal. Commit `subasm: split-equivalence test on the rover`.

### Task 5: mirror

**Files:** Modify `subasm.py`, `python/napl/design/build.py` (report `mirror_signs`), `python/napl/design/drc.py` (new
finding `MIRROR_UNSUPPORTED`), `python/napl/design/library.py` (read `[mirror]` table: `symmetry`, `opposite`),
`parts/README.md` (document `[mirror]`); add `[mirror]` to the parts used in tests and to symmetric motors/tubes used later.
Test `tests/python/test_subasm_mirror.py`.

**Interfaces:** `[[subassemblies]] mirror = "xz"|"yz"|"xy"`; part `[mirror] symmetry = "xz"` or `opposite = "<id>"`;
`report["mirror_signs"] = {"<joint>": -1 | 1}`.

- [ ] **Step 1: failing tests:**

```python
def test_mirrored_leg_poses_equal_hand_mirror(tmp_path):
    # build parent with l (unmirrored) and r (mirror="xz" about the parent's xz plane); compare world body poses at qpos0:
    # for every body b of l: pos(r:b) == M @ pos(l:b) (1e-12) and R(r:b) == M R(l:b) M S_b where S_b is the part's
    # symmetry fix (identity for an opposite-hand part)
    ...

def test_mirror_unsupported_part_is_drc_error(tmp_path):
    ...  # a part without [mirror] in a mirrored child -> finding code MIRROR_UNSUPPORTED naming the part and instance

def test_mirror_signs(tmp_path):
    ...  # abduction hinge: positive command moves l and r feet outward (check foot y after +0.1 rad: l.y up, r.y down)
```

- [ ] **Step 2–4:** implement: during expansion record `mirror` per flattened instance; in `compute`, when an instance is
  mirrored, (a) swap the part to `opposite` if given, else (b) compose the placement with the part's symmetry reflection so
  the frame stays right-handed; mates use the transformed port frames; joint axes follow; compute `mirror_signs` from
  axis(r) vs M axis(l). Commit `subasm: mirror with symmetry / opposite-hand parts`.

### Task 6: Design + Backend ops

**Files:** Modify `python/napl/design/edit.py` (Design ops), `python/napl/design/viewer.py` (Backend dispatch + JSON shapes
for the viewer), `python/napl/design/uilib.py` (MY PARTS search entries); Test `tests/python/test_subasm_ops.py`.

**Interfaces (Backend ops; JSON in/out like existing ops):**
- `sub.new {name, base}` → opens a new empty child document, returns `{doc_id}`
- `sub.export {kind: "port"|"connector"|"param", ref: "<inst>.<x>", name}` / `sub.unexport {kind, name}`
- `sub.insert {file, id, mate: {a: "<parent inst>.<port>", export: "<port export>"} | pos/rpy, mirror?}`
- `sub.mirror {id, plane}` (creates `<id>_m` mirrored copy or toggles), `sub.override {id, name, value}` (provenance table)
- `sub.open {id}` / `sub.close {}`: document stack; `docs` state returns `[{doc_id, path, crumb, dirty}]`
- `Design.reload_children()`; `uilib.my_parts(root: Path) -> list[dict]` (assemblies with `[exports]`: name, path, exports,
  mass, cost)
- [ ] **Step 1: failing tests:** insert 4 legs (2 mirrored) through ops into a new chassis, override one, save, reload →
  parent TOML has 4 `[[subassemblies]]` entries and no `:` ids; `sub.open fl` → crumb `["dog", "fl (leg.toml)"]`, undo stacks
  separate; edit leg on disk → `reload_children` rebuild changes the parent's mass.
- [ ] **Step 2–4:** implement. Commit `subasm: Design/Backend sub.* ops, MY PARTS listing`.

### Task 7: BOM roll-up + price param migration

**Files:** Create `python/napl/design/bom.py`; modify `build.py` (report `bom`), `parts/**` (migrate the 33 `price_usd` and
`price_eur|bdt|hkd|php` params to `price` USD with a dated conversion `derivation`), `tools/check_parts.py` (validate
`price` unit USD); `format_budget` prints cost. Test `tests/python/test_bom.py`.

**Interfaces:** `report["bom"] = {"lines": [{part, count, unit_price, fidelity}], "by_sub": {"fl": {mass, cost}}, "total":
{mass, cost, estimated_share}, "missing": [part ids]}`.
- [ ] **Step 1: failing tests:** totals equal sum of lines; parts without price listed in `missing`, cost excludes them and
  says so; per-sub subtotals sum to the total minus the parent's own parts.
- [ ] **Step 2–4:** implement; `check_parts` 0 problems. Commit `bom: price param, BOM roll-up per sub-assembly`.

### Task 8: docs + full regression

- [ ] `docs/guide.md` section "Sub-assemblies" (schema, exports, mirror, overrides, BOM, ops) and `parts/README.md` `[mirror]`.
- [ ] Full regression on hfbox: `ctest --preset dev`, full pytest, `check_licenses`, `check_parts` — all green, goldens equal.
- [ ] Commit `docs: sub-assemblies`.
