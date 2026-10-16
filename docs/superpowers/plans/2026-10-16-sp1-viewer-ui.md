# SP1-ui: controls icon, DRC message fix, sub-assembly workspace UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the controls legend moves from the viewport into a top-bar icon popover; the stale status-bar DRC message is fixed;
the viewer gets the sub-assembly workspace (document stack + breadcrumb, New part, Export as..., MY PARTS, Mirror, collapsed
tree node, read-only internals with Open part, hierarchical WIRING block, per-sub mass/BOM).

**Architecture:** C++ ImGui viewer (`src/viewer/viewer.cpp`, `src/viewer/wire2d.cpp`, `src/viewer/theme.h`) calling the
Python Backend ops (`python/napl/design/viewer.py`). Tasks 1-2 have no dependency; tasks 3+ consume the `sub.*` ops of the
SP1-core plan (`docs/superpowers/plans/2026-10-16-sp1-subassemblies-core.md`, Task 6) — start them after SP1-core Task 6 is
merged into `napl-v1` (rebase onto it).

**Tech Stack:** C++20, Dear ImGui, Vulkan viewer, nanobind bridge, pytest + gpu ctest smoke frames.

**Spec:** `docs/superpowers/specs/2026-10-16-quadruped-design.md` (sections 3.5, 3.6).

## Global Constraints

- UI actions are Backend ops (scriptable, recorded in the oplog), never viewer-only state changes of the design.
- CAD-like look: reuse `theme::icon_button`, `theme::tabs`, palette and fonts; no new visual language.
- Every existing gpu test passes (`xvfb-run -a ctest --preset gpu`, `--gpu` lane, GTT < 4 GB, one GPU job at a time).
- Edit files only with Edit/Write tools. Never commit screenshots.

## Review Focus

1. The popover open while switching pages — content must follow the new page, not show stale rows. Test: smoke frame after
   `ui.page WIRING` with the popover open shows the WIRING rows.
2. A narrow window (W < 1000, title hidden) — the icon must not overlap the tabs or the Wiring zoom label. Test: smoke at
   900 px width, assert the icon rect and the zoom rect are disjoint (rects exported by the viewer's test hook).
3. Breadcrumb with an unsaved child — returning up must keep the child's dirty marker and offer save, not drop edits. Test in
   Task 4.
4. Keyboard shortcuts typed in PROGRAM's editor must not toggle the popover. Test: the popover key acts only when no text
   field is focused.
5. A parent whose child file is missing on disk — the tree shows the node in error state with the path, the app does not
   crash. Test in Task 5.

---

### Task 1: controls icon + popover

**Files:** Modify `src/viewer/theme.h` (new `Icon::Keys`), `src/viewer/theme.cpp` (draw it), `src/viewer/viewer.cpp`
(`top_bar` icon left of undo; remove `controls_legend()` call at `build_panes`; new `controls_popover()` with per-page rows;
`keys_open_` default false), `src/viewer/wire2d.cpp` (drop the CONTROLS frame of `legend`, keep LEGEND; `legend_open_[1]`
and the CONTROLS layout constants go), `tests/viewer/...` smoke (follow the existing gpu smoke test that grabs frames).

- [ ] Step 1: failing gpu smoke test: BUILD frame has no `CONTROLS` window (`##vpkeys` absent), top bar has `##qkeys`; click
  → popover lists BUILD rows; `ui.page WIRING` → WIRING rows; PROGRAM rows include the editor shortcuts that exist in the
  P9 IDE (read them from the IDE code, do not invent).
- [ ] Step 2: run → FAIL. Step 3: implement. Step 4: PASS + all gpu tests. Step 5: commit `viewer: controls in a top-bar popover`.

### Task 2: stale DRC message

**Files:** `src/viewer/viewer.cpp` (status bar), test.
- [ ] Step 1: reproduce: the status-bar DRC message keeps the old text (e.g. "DRC 16 errors…") after fixes while the chip
  shows the new count. Find where the message is cached (`systematic-debugging`: trace the message source vs the chip
  source). Failing test: Backend op sequence that changes the error count, then the status text equals the chip count.
- [ ] Step 2-4: fix at the source (single source of truth for both). Commit `viewer: status DRC message follows the DRC count`.

### Task 3: document stack + breadcrumb (after SP1-core Task 6 merged)

**Files:** `viewer.cpp` (top bar title → crumbs from Backend `docs` state; per-document undo display), Backend glue.
- [ ] Failing test: ops `sub.new leg` → crumb `untitled > leg`; `sub.close` → back; dirty marker per crumb. Implement.
  Commit `viewer: document stack breadcrumb`.

### Task 4: New part, Export as..., EXPORTS list, Open part

**Files:** `viewer.cpp` (ribbon BUILD: New part, Mirror; inspector: Export as... on port / connector / param rows;
browser pane EXPORTS section; read-only inspector for `origin` items with "from <file>" and Open part button).
- [ ] Failing ops+smoke tests: export a port via the inspector button path (op recorded `sub.export`), EXPORTS lists it,
  removal works; selecting `fl:hip_motor` shows "from leg.toml" and Open part opens it; unsaved child keeps dirty marker
  after returning up (Review Focus 3). Implement. Commit `viewer: New part, exports, open part`.

### Task 5: MY PARTS + insert + mirror + tree node

**Files:** `viewer.cpp` (`library_page`: MY PARTS category from `uilib.my_parts`; drag-in → `sub.insert` with ghost snap;
browser tree: one collapsible node per sub-assembly, children greyed, mirrored badge, missing-file error state).
- [ ] Failing tests: insert leg into a chassis by drag op, mirror it, tree shows 2 nodes, missing child file → node error
  state, no crash (Review Focus 5). Implement. Commit `viewer: MY PARTS, insert, mirror, sub-assembly tree`.

### Task 6: hierarchical WIRING block + per-sub mass/BOM

**Files:** `src/viewer/wire2d.cpp`, `python/napl/design/wire2d.py` (flatten view: a sub-assembly as one block exposing its
exported connectors; wires into it terminate on those), `viewer.cpp` (status bar cost; mass/BOM panel per sub).
- [ ] Failing tests: parent wiring sheet has 4 leg blocks with exported connectors only; status shows total cost from
  `report.bom`; panel lists per-sub mass and cost. Implement. Commit `viewer: hierarchical wiring block, BOM panel`.

### Task 7: regression + smoke frames reviewed

- [ ] gpu ctest all green; look at the smoke frames (BUILD, LIBRARY MY PARTS, tree, WIRING blocks, popover on each page) and
  describe them in the hand-back (unreadable text, overlaps, clipped labels are failures). Docs: `docs/guide.md` UI section.
