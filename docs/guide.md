# napl user guide

How to go from "nothing installed" to "a robot I designed flies or moves under real flight / servo software, with checks and
evidence". Every command and snippet below was run on the build host (hfbox) while writing this guide, and the output shown is
real (trimmed with `...` where long). Anything not run is marked **[not run]** with the reason. Numbers such as part counts move as
the parts library grows; the ones printed are what the run showed.

Convention: on the host the project venv is `.venv`, so the CLI is `.venv/bin/napl`. Commands are shown as you would type them on
your laptop, wrapped by the remote runner (section 1). Where a block says "inside the wrapper" it is the argument of
`tools/remote.sh run --nolock '...'`.

Contents: 1 Install and run - 2 Parts library - 3 Build a robot - 4 Check it - 5 Simulate quickly - 6 Program it - 7 Agents (MCP) -
8 Limits and fidelity.

---

## 0. What napl is, in one page

* A physics kernel (C++20, MuJoCo-based, deterministic, replayable) with a device layer: battery, ESC + BLDC motor, rotor
  aerodynamics, bus servos, IMU / baro / mag / GNSS, wind. Used from Python (nanobind) and from an MCP server.
* A **parts library** (`parts/`, hundreds of real, purchasable parts) where every number carries a source and a fidelity tag.
* **Assemblies as data** (`assemblies/*.toml`): a robot is instances of library parts, mates between their ports, fasteners, wires
  and device bindings. `napl design build` turns one into MJCF + a report (mass budget, BOM, provenance).
* An **editing API** (`napl.design.Design`, the same operations as the MCP `design_*` tools and the viewer's Design window):
  add / snap / swap / remove / wire / undo, each edit re-checked.
* A **design-rule checker** (DRC / ERC): mechanical, electrical and performance rules, each finding with a code, severity and a
  concrete fix.
* **Fast simulation** of a design (`simulate("hover" | "hold")`), **virtual benches** against vendor thrust tables, and a generated
  **fidelity report**.
* **Programming**: `napl program` runs your ordinary program against the simulated robot over the real protocol: PX4 SITL over MAVLink
  (X500), Feetech STS (SO-101) and Dynamixel Protocol 2.0 (Koch) over a pseudo-terminal. Monitors, a replay hash, fault injection.

Reference material lives in `assemblies/README.md` (schema, editing API, DRC rules), `parts/README.md` (part schema, ports),
`docs/models.md` (device equations), `docs/validation/` (benches, X500 hover), `docs/fidelity.md` (generated).

---

## 1. Install and run on a build host

napl is built and run on a Linux build host; the laptop only edits, runs git and syncs. `tools/remote.sh` is the whole interface.

```
tools/remote.sh bootstrap              one-time: toolchain venv (cmake, ninja, uv) on the host
tools/remote.sh sync                   rsync the working tree to the host (excludes build/, .venv/, .git/, remote-out/)
tools/remote.sh run [--gpu] [--nolock] [--slots N] CMD   sync, then run CMD in the remote source dir; streams output
tools/remote.sh bg [--slots N] NAME CMD   sync, then run CMD in tmux session napl-NAME; output to logs/NAME.log
tools/remote.sh log NAME [N]           last N lines of that log and whether it is still running
tools/remote.sh pull PATH...           copy remote paths (relative to the remote dir) into ./remote-out/
```

CPU lanes: the host has `NAPL_SLOTS` (default 2) slot lock files in `$NAPL_LANES` (default `/workspace/wt/napl-cache/lanes`), shared by all
worktrees, each worth `NAPL_JOBS / NAPL_SLOTS` (8) threads. `run` / `bg` wait until they hold `--slots N` of them (default 1; `--slots 2` =
the whole box, 16 threads) and export that thread count as `NAPL_THREADS`, `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`,
`LP_NUM_THREADS`, `CMAKE_BUILD_PARALLEL_LEVEL` and `CTEST_PARALLEL_LEVEL`. `--nolock` skips the lane (light commands and anything long-lived
that must not block other jobs). Never `flock` a lane file inside a job. The GPU has no lock (`--gpu` only matters when `NAPL_GPU_LOCK`
names a file). Size parallelism from `NAPL_THREADS` (`pytest -n $NAPL_THREADS`; the MC drivers' `--workers` default does, bounded by free memory). A foreground job is killed after `NAPL_TIMEOUT` seconds (default 3600); long work
belongs in `bg`. An ssh connection failure (the host sits behind a funnel that has outages of about 15 minutes) is retried with
backoff, six times.

The host is a parameter, so a dedicated cloud box works the same way (AWS-ready; the baseline is Ubuntu 24.04, GCC 13, Python 3.12):

```
NAPL_REMOTE=<ssh alias>  NAPL_REMOTE_DIR=<abs dir>  NAPL_GPU_LOCK=   tools/remote.sh run '...'
#                                                    ^ empty = dedicated host, no shared GPU lock
# NAPL_JOBS (default 16): build threads;  NAPL_TIMEOUT (default 3600): foreground limit in seconds
```

C++ compiles go through sccache when it is on the host's PATH (`CMakeLists.txt` picks it up; `-DNAPL_SCCACHE=OFF` opts out). `remote.sh`
puts `$NAPL_CACHE_DIR/bin` (default `/workspace/wt/napl-cache`) on PATH and points the cache at `$NAPL_CACHE_DIR/sccache` (4 GB cap), with
every `/workspace/wt/napl*` dir as a base dir, so a clean build of the same commit in another worktree hits the cache. The sccache server is
one per host and reads its base dirs at start: after creating a new worktree dir run `sccache --stop-server` once (idle servers only).

Python tests: `.venv/bin/python -m pytest tests/python -q -n $NAPL_THREADS` (pytest-xdist, in the `dev` extra; serial is about 1 h 54 min). The
`px4` tests claim a free PX4 instance 0..9 per test (flock + port check, preferring `NAPL_PX4_INSTANCE`), so workers never share ports;
more than 10 workers only make the extra px4 tests wait for an instance. A few tests are real-time (PX4, the `napl program` pick/showcase
scenarios) and can flake when the host is oversubscribed: rerun just those. Take the whole box with an outer timeout, e.g.
`NAPL_TIMEOUT=7200 tools/remote.sh run --slots 2 'setsid timeout 7000 .venv/bin/python -m pytest tests/python -q -n $NAPL_THREADS --durations=30'`.

First check that the host works (this was run):

```
$ tools/remote.sh run --nolock '.venv/bin/napl design check assemblies/x500v2.toml | tail -4'
INFO    HOVER_THROTTLE  [mot1, mot2, mot3, mot4]  hover: throttle 50 %; total 12.50 A (mot1 3.79 N 3.12 A; ...)
INFO    THERMAL  [mot1]  hover copper loss 4.4 W = 10 % of the 46 W electrical input at the worst motor mot1 ...
INFO    THRUST_TO_WEIGHT  [mot1, mot2, mot3, mot4]  max static thrust 44.7 N (...) / weight 15.17 N (1.547 kg) = 2.95
0 errors, 6 warnings, 4 info
```

**Local install [not run: the host was already provisioned]** (from `pyproject.toml`): Python >= 3.12, CMake >= 3.25, Ninja, a C++20
compiler; `uv venv && uv pip install -e '.[px4,arm,dev]'` builds the C++ kernel with scikit-build-core into `build/` (the MuJoCo
and FlatBuffers sources are about 700 MB there). Extras: `px4` = MAVSDK-Python (flying X500 under PX4), `arm` = ROBOTIS
`dynamixel_sdk` (the Koch example), `dev` = pytest. The design tools (`napl design ...`, the `napl.design` package) need only the
standard library and numpy; the kernel is needed to simulate. PX4 SITL needs its own build on the host, installed once with
`tools/px4_setup.sh` **[not run]**.

Pass/fail and exit codes: `napl test` / `napl program` return 0 = pass, 1 = a monitor failed, 2 = an error (bad spec, model, args);
`napl design check` returns 1 if any finding is an error.

Housekeeping when running scenarios: `napl program` writes a run directory (`--runs-dir`, default `runs/`) that can be tens of
MB (the replay log). Point `--runs-dir` at `/tmp/...` and delete it afterwards on a small disk.

---

## 2. Browse the parts library

One TOML file per real part under `parts/<domain>/<id>.toml`. Check the whole library (every connecting part needs ports,
every number a source or a derivation / rationale):

```
$ tools/remote.sh run --nolock 'python tools/check_parts.py | tail -1'
653 parts, 9189 values (datasheet=7464, derived=883, estimated=328, literature=509, measured=5), 0 problem(s)
```

Read a part from Python, with the fidelity tag of each number:

```python
from napl.design import Library

lib = Library("parts")
m = lib.get("holybro-2216-kv920")
seen = set()
for k, p in m.params.items():
    if isinstance(p, dict) and p.get("fidelity") not in seen:
        seen.add(p["fidelity"])
        note = p.get("derivation") or p.get("rationale") or ""
        print(f"{k:14s} {p['value']} {p['unit']:6s} {p['fidelity']:10s} src={p.get('src')} {note[:110]}")
src = m.raw["sources"][0]
print(src["id"], src["url"], "|", src["quote"][:90])
```
```
mass           0.064 kg     datasheet  src=S2
stator_diameter 0.022 m      estimated  src=None Holybro publishes no stator dimensions; '2216' is the standard 22 mm stator diameter x 16 mm stator height nam
kt             0.01038 N*m/A  derived    src=None Kt = 60 / (2*pi*Kv) = 60 / (2*pi*920) = 0.010380 N*m/A (SI torque constant equals 1/Kv with Kv in rad/s per V)
S1 https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2.md | Holybro 2216 KV920 Motor (4 pcs) with XT30 Plug
```

### Fidelity tags

| tag | meaning | requires |
|---|---|---|
| `datasheet` | stated by the vendor for this exact part | `src` + verbatim `quote` (in the part's `[[sources]]`) |
| `literature` | peer-reviewed or well-known reference measurement of this part or exact model | `src` + quote |
| `measured` | independent test data of this exact part (e.g. public thrust-stand logs) | `src` + quote |
| `derived` | computed from other sourced values | `derivation` (the arithmetic) |
| `estimated` | nothing published | `rationale` (the closest evidence) |

The rule is enforced by `tools/check_parts.py`: no value without a source unless it is derived or estimated, quotes are verbatim so
anyone can re-fetch the page, no invented URLs. Every estimated value an assembly actually uses is listed in its build report
(`estimated`) and counted in the fidelity report (section 5).

### Ports: how parts connect

A part that connects to anything declares `[[ports]]`: `kind` (`mech`, `power`, `signal`), a `type` (`bolt-pattern.16x16.m3`,
`shaft.5`, `xt60`, `motor-3phase`, ...), gender, pose (`pos`, `axis`, `ref`, `symmetry`), and ratings on power ports (`v_max`,
`i_max`, `role`). Two ports snap together when their types match (or are a paired family such as `shaft.<d>` / `bore.<d>`) and
their genders are male + female or either is neutral. The 2216 motor above has `base` (`bolt-pattern.16x16.m3`), `shaft`,
`prop_adapter` and `phase` (`motor-3phase`).

### Ask what fits

`napl design suggest <assembly> <inst.port | inst>` ranks library parts for an open port, or swap candidates for an instance:

```
$ napl design suggest assemblies/x500v2.toml con1.motor --limit 3
holybro-2216-kv920                             160.0 via port base
        same port type
        category motor.bldc is used on bolt-pattern.16x16.m3 ports here
        stock part (currently on this port)
iflight-xing2-2207-1855kv                      140.0 via port base
        same port type
        category motor.bldc is used on bolt-pattern.16x16.m3 ports here
iflight-xing2-2207-2755kv                      140.0 via port base
...
$ napl design suggest assemblies/x500v2.toml mot1 --limit 2
kde-2315xf-885                                       (keeps 2 mates, loses 0, mass +0.011 kg)
        wires[6].length: 'mot1.cable_length' no longer resolves (kde-2315xf-885 has no param 'cable_length')
kde-2315xf-965                                       (keeps 2 mates, loses 0, mass +0.011 kg)
        wires[6].length: 'mot1.cable_length' no longer resolves (kde-2315xf-965 has no param 'cable_length')
```

The second form lists what a swap would keep, lose, and leave dangling (a reference the new part cannot satisfy). The score is a
design weight (port fit), not physical data. Add `--json` for machine output, `--category` filtering is in the Python API
(`d.suggest("tube1.esc", category="esc")`).

---

## 3. Build a robot

A design is an assembly: open an existing one, or start empty (`Design.new("name")`, then `add(frame, root=True)` and snap
everything else to named open ports; the test `tests/python/test_design_scratch.py` builds a complete X500 that way and gets the
same mass and hover current as the reference to 0.1 %). Edits go through one API with three front ends:

| operation | Python (`napl.design.Design`) | MCP tool `design_edit` op | viewer Design window |
|---|---|---|---|
| open | `Design.open(path)` / `Design.new(name)` | `design_open` with `path` or `new` | `napl design view x.toml` |
| add + snap to the best open port pair | `d.add(part, to="mast")` / `d.snap(part, to="mast")` | `{"op":"add","part":..,"to":..}` | Library tab, "snap here" (ranked placements, prev / next) |
| swap a part | `d.swap(id, part, auto=True)` | `{"op":"swap",...}` | "suggest swaps" + "swap" |
| remove | `d.remove(id)` | `{"op":"remove","id":..}` | toolbar "remove selected" |
| mate / wire by hand | `d.mate(a, b)`, `d.wire(...)` | `mate`, `wire` | |
| auto-wire power / signal | `d.autowire(a)`, `d.autowire_all()` | `autowire`, `autowire_all` | Wiring tab |
| device bindings from parts | `d.auto_devices(fill=...)` | `auto_devices` | (swap refreshes them) |
| set a value | `d.set(id, param, value=.., unit=.., fidelity=.., rationale=..)` | `set` | |
| design header (fixed base, timestep) | `d.edit([{"op": "assembly", "base": "fixed", "timestep": 0.001}])` | `assembly` | |
| undo / redo | `d.undo()`, `d.redo()` | `design_undo {redo?}` | toolbar |
| save | `d.save(path)` | `design_save {path}` (needs approval) | toolbar "save" |

Every edit is validated (the numbers rule: a bare number is refused, give `{value, unit, fidelity, + src & quote | derivation |
rationale}`), the DRC re-runs, and the result carries `changes` plus the DRC delta (`before`, `after`, `new`, `resolved`, `changed`).
A failed edit changes nothing. `d.edit([ops])` applies a list atomically.

### Python (run on the host; saves `my-quad.toml`)

```python
from napl.design import Design

d = Design.open("assemblies/x500v2.toml")  # the design + the parts library
print(len(d.asm.doc["instances"]), "instances,", len(d.check()), "DRC findings")

# what fits the open motor seat of arm connector 1?
for row in d.suggest("con1.motor", limit=3, category="motor.bldc"):
    print(f"{row['part']:30s} score {row['score']}  {row['why'][0]}")

# remove the GPS (its wire and sensor bindings go with it), then snap it back to the best open port on the mast
old = {x["name"]: x["set"] for x in d.doc["devices"] if x["name"] in ("mag", "gnss")}
c = d.remove("gps").changes[0]
print("remove:", c["mates_removed"], c["wires_removed"], [x["name"] for x in c["devices_removed"]])
c = d.snap("holybro-m8n-gps", to="mast").changes[0]
print("snap:", c["snap"]["why"][:2])
c = d.autowire_all().changes[0]
print("autowire_all:", [w["wire"]["name"] for w in c["wired"]], "ambiguous:", len(c["ambiguous"]))

# the sensor bindings come back from the parts; values no part carries stay unset, and the DRC says so
r = d.auto_devices()
print("unset:", r.changes[0]["unset"], "| DRC", r.after)
fill = {
    u["device"]: {"set": {f: old[u["device"]][f] for f in u["fields"]}}
    for u in r.changes[0]["unset"]
}
r = d.auto_devices(fill=fill)  # here: the values the stock design carries (estimates, with rationale)
print("filled | DRC", r.after)

# a swap that does not fit: nothing is invented, the gaps are listed, the DRC delta is shown
r = d.swap("mot1", "kde-2315xf-965")
c = r.changes[0]
print("swap keeps", [m["mate"] for m in c["mates"]["kept"]], "| DRC", r.before, "->", r.after)
print("dangling:", c["dangling"])
print("stale devices:", [s["device"] for s in c["stale_devices"]])
d.undo()
print("undo |", len(d.check()), "findings")
d.save("my-quad.toml")
```
```
49 instances, 10 DRC findings
holybro-2216-kv920             score 160.0  same port type
iflight-xing2-2207-1855kv      score 140.0  same port type
iflight-xing2-2207-2755kv      score 140.0  same port type
remove: ['gps-seat'] ['gps-to-fc'] ['mag', 'gnss']
snap: ['same port type', 'clearance >= 75.0 mm to the nearest instance']
autowire_all: ['fc.gps1--holybro-m8n-gps.gps_out'] ambiguous: 0
unset: [{'device': 'mag', 'fields': ['rate_hz', 'earth_field']}, {'device': 'gnss', 'fields': ['lat0', 'lon0', 'alt0']}] | DRC {'error': 5, 'warning': 6, 'info': 4}
filled | DRC {'error': 0, 'warning': 6, 'info': 4}
swap keeps ['motor1', 'spin1'] | DRC {'error': 0, 'warning': 6, 'info': 4} -> {'error': 1, 'warning': 7, 'info': 0}
dangling: ["wires[6].length: 'mot1.cable_length' no longer resolves (kde-2315xf-965 has no param 'cable_length')"]
stale devices: ['esc1']
undo | 10 findings
```

What to read from this:

* **Snapping** picks a port pair and a rotation by geometry: the `why` list is the score breakdown (port fit, clearance to the
  nearest instance, "upright", inboard of the centre of mass). When several placements are close, `changes[0]["snap"]["alternatives"]`
  ranks them; if every candidate interferes with another instance the edit is refused (`force=True` places anyway).
* **autowire** wires an open power / signal port to the one port that needs it: wire part = the lightest library wire rated for
  the DRC's worst-case current x 1.2, length = port distance x 1.3 rounded up, connectors pinned. Where the choice is the user's
  (which ESC is PWM channel 1) it is reported under `ambiguous`, never guessed.
* **auto_devices** writes the simulation bindings (battery, ESC + motor + prop, servos, loads, sensors) from the parts and
  topology, so a part swap does not need hand-edited device JSON. Values no part carries are listed as `unset`, and the DRC
  reports them as `DEVICE_UNSET` errors (5 here, `rate_hz`, `earth_field`, `lat0`, `lon0`, `alt0`) until you supply them with `fill`.
* **swap** never invents a value: here the KDE motor mates by port, but the wire length referenced `mot1.cable_length`, which the
  KDE part does not have; the DRC delta shows one new error and the `esc1` binding is flagged stale. `swap(..., auto=True)`
  regenerates the bindings in the same edit (one undo reverts both).

### MCP (the same edit through the tool interface; run in-process against the server object)

```python
import asyncio
import json

from mcp import Client
from napl.mcp_server import NaplServer


async def main():
    async with Client(NaplServer(".").server) as c:  # NaplServer(root): paths stay under root

        async def call(tool, **args):
            r = await c.call_tool(tool, args)
            return json.loads(r.content[0].text)

        o = await call("design_open", path="assemblies/x500v2.toml")
        print("open:", len(o["instances"]), "instances")
        ops = [
            {"op": "remove", "id": "gps"},
            {"op": "add", "part": "holybro-m8n-gps", "to": "mast"},
            {"op": "autowire_all"},
        ]
        r = await call("design_edit", ops=ops)
        print("edit:", [ch["op"] for ch in r["changes"]], r["drc"]["before"], "->", r["drc"]["after"])
        r = await call("design_check", severity="error")
        print("errors now:", r["counts"]["error"], "| listed:", len(r["findings"]))
        r = await call("design_undo")
        print("undo:", r["drc"]["after"])
        try:
            await c.call_tool("design_save", {"path": "build/my-quad.toml"})
        except Exception as e:  # this client declares no elicitation, so the destructive save cannot be approved
            print("save refused:", type(e).__name__)


asyncio.run(main())
```
```
open: 49 instances
edit: ['remove', 'add', 'autowire_all'] {'error': 0, 'warning': 6, 'info': 4} -> {'error': 0, 'warning': 6, 'info': 4}
errors now: 0 | listed: 0
undo: {'error': 0, 'warning': 6, 'info': 4}
save refused: MCPError
```

(The three ops are one atomic batch. The mag / gnss bindings that went with the removed GPS are not regenerated by it: add an
`{"op":"auto_devices", "fill": ...}` op as in the Python session. Without them a PX4 flight of the design is refused with
`PX4 bridge config: the design has no mag device`, which is how that gap was found.) For a real agent, start the server and point the client at it
**[not run as a process]**: `napl mcp --root .` (stdio) or `napl mcp --http --port 8765`; with Claude Code
`claude mcp add napl -- napl mcp --root .`. Section 7 lists the tools.

### The viewer **[not run: needs a display and a Vulkan device]**

```
napl design view assemblies/x500v2.toml          # blocks until the window closes
napl mcp --root . --view                         # MCP server + viewer: an agent's design_edit shows up in the window
```

Screenshots of the Design window are not committed; regenerate them on the host with `tools/screenshots.py` (see its docstring).

The window is a front end to the same `Design` object: each control sends one request to `napl.design.viewer.Backend`, which calls
the matching `Design` method, rebuilds and reloads the model keeping camera and selection. The DRC window lists every finding
after each edit (red = error, `*` = new from the last edit; clicking one selects the instance involved). Edits are not saved
until you press save; comments of a hand-written TOML are not kept by a save.

**Layout (after Fusion 360 / Onshape / KiCad).** An application bar (mark, design name, the page tabs BUILD / LIBRARY / WIRING / SIMULATE /
PROGRAM as a workspace switcher, quick undo / redo / save), a ribbon of grouped icon + label tools for the active page (BUILD: Select, Move,
Rotate | Align, Mirror, Duplicate, Remove | Mate (port snapping), Colour | Measure, Check | Undo, Redo | Save; WIRING: Wire, Pin wire |
Autowire | DRC, Fit; each button is an existing function, nothing is decoration), docked panels (BUILD: left BROWSER = the assembly tree
with a visibility eye per part, mates, wires and devices, or PARTS = the library list; right PROPERTIES / DRC / PLACE / HISTORY), and a
status bar (selection, units, snap state, part count, mass, DRC counts, sim time). The 3D view has a view cube (click a face for Front /
Back / Left / Right / Top / Bottom), home (iso), zoom-to-fit, orbit / pan, a cable-emphasis toggle and a scale bar; BUILD uses the light
studio backdrop with a 1 m / 10 cm (and, close up, 1 cm) floor grid, SIMULATE the outdoor sky. Hover pre-highlights and selects in the one
accent blue. MEASURE: click two parts for the centre distance, dx / dy / dz and the clear gap. WIRING draws the sheet like a schematic
(dark sheet, thin grid, nets coloured by kind, junction dots where conductors share a pin, a title block); labels follow a level of
detail: all pin labels when zoomed in, only the wired pins in between, connector titles when zoomed out. Icons are drawn with ImDrawList
primitives (no icon font to license); fonts are the Apache-2.0 Roboto and Cousine of Dear ImGui.

**Camera moves for recordings.** Backend ops (`ui.camera` stays the preset / instant one): `camera.orbit {target?, yaw_deg?, pitch_deg?,
dist?, seconds?=3, relative?=true, ease?=smooth|linear}` glides the camera around `target` (relative: the angles are sweeps added to the
current ones, so `yaw_deg=360` is one full turn; `dist` dollies), `camera.fly {eye, look, seconds?, ease?}` flies the eye and the look-at
point along straight lines. Wall-clock time, any 3D page (BUILD, LIBRARY, SIMULATE).

#### A HUD over the 3D view **[not run here: needs a display and a Vulkan device]**

`Sim.load(model, host=True)` + `sim.show(hud=napl.hud.Hud(), clean=True)` opens the SIMULATE view with no panes; any thread then calls
`hud.set(napl.hud.doc([...items...], seq=n))` and the window draws the latest document every frame: text boxes, label / value tables,
tab rows with an underline, rectangles, lines, images (PNG files, optionally cropped) and groups with their own pixel space (a camera
frame and the boxes drawn on it), plus world-anchored labels, polylines, rings and dots projected with the camera the scene was rendered
with. A `camera=dict(target=, yaw=, pitch=, dist=)` in the document drives the orbit camera. The schema is the comment block of
`src/viewer/hud.h`; the builders are `python/napl/hud.py`. The counter-UAS display (stage tabs, counters, track labels, tracers, the
TURRET CAMERA picture-in-picture) is `python/napl/cuas/hudview.py`; `tools/record_sim.py play|frames|encode` replays a `napl program`
run under the viewer with it and records it (see `examples/cuas/README.md`). `Hud.dry_run(...)` draws a document with no window, which
is what `tests/python/test_hud.py` uses.

**Look.** `napl.render_style("studio,edges")` (or `NAPL_RENDER_STYLE=studio,edges` in the environment) sets the process-wide look of
`Sim.render` and the viewer: `outdoor` (default for simulation: gradient sky with a warm horizon, procedural clouds, sun glare, dark
ground fading into the haze) or `studio` (neutral backdrop, metre grid; the Design window's default), `edges` (dark CAD-style crease
outlines), `noshadows` (the sun shadow map), `nowires`, `boldwires` (cables at least 4 px wide on screen at any zoom; the data keeps their
true diameter; the Design window turns it on for BUILD / LIBRARY / WIRING, toolbar WIRES). Specular follows the MuJoCo material (`specular`, `shininess`, `metallic`,
`roughness`); output is 4x MSAA and sRGB.

**Device and off-screen viewer.** Headless renders (`Sim.render`, `Render`) and the off-screen viewer pick the best Vulkan device
(discrete > integrated > virtual) and refuse a CPU rasteriser (llvmpipe / lavapipe) unless `NAPL_RENDER_ALLOW_CPU=1`;
`NAPL_VK_DEVICE=<substring of the device name>` forces one. `napl-view MODEL --offscreen WxH` (or `NAPL_OFFSCREEN=1`) runs the
viewer with no window, X server or swapchain: scene + UI render into an off-screen image on the GPU, 60 fps cap, and
`napl._napl.viewer_frame()` returns the newest frame as `(seq, (h, w, 4) uint8)`. The recording tools (`tools/design_shot.py`
`grab()`) use it by default; `NAPL_OFFSCREEN=0` goes back to the Xvfb window (llvmpipe). Recording tools store raw frames
(`f*.rgz`; `--keep-frames` adds PNGs) and encode through `tools/videnc.py`: GStreamer VA (`vaav1enc` .webm / `vah264enc` .mp4;
LGPL) when the host has it, `--gpl` tries ffmpeg VAAPI first, else PyAV software (VP9).

**Wires are tubes.** Every wire of the design is drawn from terminal to terminal, one tube per conductor (one per pin pair, at most 16;
`Sim.tubes()` lists them): insulated outer diameter from the wire's AWG (`napl.design.wiretubes.AWG_OD_MM`), colour by function
(power red / black, three-conductor motor leads yellow / green / blue, signal yellow / white / blue / ...). Each conductor starts at
its own pin and leaves the connector along the connector axis. The bundle is routed around the parts (`napl.design.wireroute`:
a distance field of the assembly's collision and visual geometry, A* through the free voxels, string-pulled and rounded to a bend radius
of at least 6 x the conductor diameter): outside the lead-out at the connector no tube point comes within 2 mm of a part
(`tests/python/test_wire_routing.py` samples every assembly). A wire it cannot place falls back to the straight chord. The tubes are
spatial tendons with no stiffness, damping or limit: the dynamics (state hashes) are unchanged. Tube sites are in the model but the
routing itself is a pure function of the geometry (deterministic, cached); the `route` argument of `build.compute` turns it off for
the DRC and edit paths that only need masses.

**2D wiring.** In the "wiring (2D)" window pick components and the terminals to wire, press `flatten >`: the view fades to a canvas
of cards, one per component, laid out as a signal flow: sources (batteries) left, then power modules / PDBs, ESCs / controllers,
loads (motors) right; parts without power terminals (receivers, GPS, sensors) form a second band below. A card draws every real pin
of the component, grouped under its connector (the Pi 5 shows the full 40-pin J8 header with names, plus its camera, USB and power
connectors), coloured by kind (power, ground, signal, no-connect); hover a pin to read its connector, number, direction, level and
alternate functions. Wires are drawn per conductor, from pin to pin, routed orthogonally around the cards and never across one (the
backend routes them: `flatten.state` carries each wire's `route` polyline, each conductor's `strands` and the `label` position). The
rule checks run live on the drawn wires: a pin ring marks a finding (voltage mismatch, two outputs on a net, a power wire with no
ground, a pin already used, an alternate-function conflict) and the side panel gives the message. Drag pin to pin to wire them
(`pinwire`: one conductor between two pins; whole ports still use the `autowire` rules: kind, connector type, gender, voltage; a refusal
shows its reason), click a wire to set gauge / length or delete it, `back to 3D` and the new wires are tubes. Every action is a
design_api op an agent can send (`flatten.begin / layout / move_card / wire / set_wire / unwire / state / end`, see
`python/napl/design/wire2d.py`; `flatten.wire` takes `inst.port` or `inst.CONNECTOR.PIN`); a window watching the same `Backend` shows the
cards appear and each wire being drawn. `flatten.wire` with a `part` (and `length`, `conductors`, `included_in`, `name`, `from_pins`,
`to_pins`) is an explicit lead: it bridges connector families that autowire refuses. When a new wire changes the auto layout the cards
glide to their new places and the wires fade in once they arrive.

**PROGRAM page.** Shows the design as TOML next to the log; `ui.program {file, scroll}` (a Backend op) shows any source file of the
repository instead (the turret / swarm programs, a scenario) and scrolls to a fraction of it; `ui.program {}` returns to the design.
The recorded end-to-end demo (`tools/record_turret.py`, `docs/showcase/turret/README.md`) uses it.

### Build and the mass budget

```
$ napl design build assemblies/x500v2.toml --out /tmp/guide-b
built x500v2: /tmp/guide-b/x500v2.xml (+ report.json), 49 instances, mass 1.5466 kg, 0 warnings; DRC 0 errors, 6 warnings (napl design check for the list)
$ napl design info assemblies/x500v2.toml | tail -3
groups: avionics 0.1148, frame 0.5530, power 0.4774, propulsion 0.4014
target frame kit published weight (Holybro 'Weight: 610g') (['frame']): model 0.5530 vs 0.6100 kg, gap -0.0570 kg (-9.3%), tolerance 0.1058 kg, within, estimated share 0.08
```

`build` writes MJCF with a `napl.devices` block plus `<name>.report.json` (mass budget, BOM, provenance, warnings, DRC findings);
`info` prints the mass budget and writes nothing. Reference assemblies: `x500v2` (Holybro X500 V2 quad), `aos5` (5-inch iFlight
AOS 5), `so101` (SO-101 arm), `koch` (Koch v1.1 arm on Dynamixel servos), `rover` (new, in progress while this guide was written; its DRC run showed 0 findings).
Full schema, snapping maths, fastener selection and mass model: `assemblies/README.md`.

---

## 4. Check it

```
napl design check assemblies/x500v2.toml [--json]     # exit 1 if any finding is an error
```

A finding has a stable `code` (UPPER_SNAKE), a `severity` (`error` | `warning` | `info`), a `message` with the numbers, `where` (the
instance / port / wire / mate ids involved) and a `fix` (a concrete suggestion: a library part id, a fastener id, a mate). Read it
as: severity first, then `where` to see what to look at, then the message numbers, then `fix`. Findings never fail a build; they
are in the build report. Each rule's threshold is either a fact in a part file or a named constant of `python/napl/design/drc.py`
documented as a **design margin** (a choice, not a standard) or with its source: `assemblies/README.md`, "Design rules".

The stock X500 (`napl design check assemblies/x500v2.toml`) has 0 errors, 6 warnings, 4 info. One warning, as printed:

```
WARNING CURRENT_PEAK_OVER_RATING  [esc1-to-mot1, wire part awg-18-silicone-wire]  wire 'esc1-to-mot1' limited by wire part awg-18-silicone-wire: 18.0 A peak, 3.1 A sustained (worst case) above the continuous rating 16 A
        fix: use a heavier wire (rated >= 18 A): awg-10-silicone-wire, awg-12-silicone-wire, awg-14-silicone-wire; or shorten the load
```

Peak (every ESC at full throttle, every servo stalled) above a continuous rating is a warning; sustained current above it is an
error.

### Rule families, with one real finding each

| family | codes (not all) | what they check |
|---|---|---|
| structure / mates | `MATE_INVALID`, `FLOATING_INSTANCE`, `PORT_UNMATED_REQUIRED` | the design is a connected tree of compatible mates |
| mechanical | `FASTENER_THREAD`, `FASTENER_SHORT`, `FASTENER_GRIP`, `FASTENER_UNAVAILABLE`, `INTERFERENCE`, `PROP_OVERLAP`, `PROP_FRAME_CLASH`, `PROP_CLEARANCE`, `CG_OFFSET` | screws vs stack, geometry overlap at rest, rotor discs vs frame / each other / ground, centre of gravity vs thrust centre |
| electrical | `WIRE_*`, `VOLTAGE_OVER_MAX`, `VOLTAGE_BELOW_MIN`, `CURRENT_OVER_RATING`, `CURRENT_PEAK_OVER_RATING`, `BATTERY_C_RATING`, `PSU_OVERLOAD`, `FLOATING_PORT`, `NET_NO_SOURCE` | connectors, voltage windows, currents vs wire / port ratings, pack C rating, unpowered ports |
| performance | `THRUST_TO_WEIGHT`, `HOVER_THROTTLE`, `FLIGHT_TIME`, `THERMAL`, `SERVO_TORQUE`, `SERVO_TORQUE_MARGIN`, `PAYLOAD_LIMIT` | what the parts can actually do |
| simulation readiness | `DEVICE_UNSET` | a device binding lacks a field the simulation needs |

The snippet below seeds one fault per family into the stock X500 and prints the finding (run on the host):

```python
from napl.design import Design


def stock():
    return Design.open("assemblies/x500v2.toml")


def show(code, findings):
    """Print the first finding with this code (a Finding, or the 'after' half of a changed pair)."""
    for f in findings:
        g = f.get("after", f) if isinstance(f, dict) else f.as_dict()
        if g["code"] == code:
            print(f"{g['severity'].upper()} {g['code']} {g['where']}\n  {g['message'][:200]}\n  fix: {g['fix'][:130]}")
            return


# mechanical: an explicit fastener that is too short for the clamp stack
d = stock()
mate = next(m for m in d.asm.doc["mates"] if m["name"] == "clamp1")
mate["fasteners"] = {"part": "iso4762-m3x6", "count": 4, "nut": "m3-flange-locknut"}
show("FASTENER_SHORT", d.check())

# electrical: a 6S pack on 4S ESCs
r = stock().swap("bat", "tattu-6000mah-6s-35c-xt60")
print("6S pack:", r.before, "->", r.after)
show("VOLTAGE_OVER_MAX", r.new)

# performance: 1.2 kg of extra mass in the middle of the frame
r = stock().set("bottom", "mass", value=1.2, unit="kg", fidelity="estimated", rationale="a heavy payload box")
print("1.2 kg payload:", r.before, "->", r.after)
show("CURRENT_OVER_RATING", r.new)
show("THRUST_TO_WEIGHT", r.changed)
```
```
WARNING FASTENER_SHORT ['clamp1', 'iso4762-m3x6']
  mate 'clamp1': iso4762-m3x6 length 0.006 m < L_min 0.0257 m
  fix: use a screw of length >= 0.0257 m: iso4762-m3x38 x4 (fasteners = "auto" selects it)
6S pack: {'error': 0, 'warning': 6, 'info': 4} -> {'error': 5, 'warning': 10, 'info': 1}
ERROR VOLTAGE_OVER_MAX ['esc1.power_in', 'bat.power_out']
  esc1.power_in is rated v_max 16.8 V but its net (bat.power_out) reaches 25.2 V
  fix: replace holybro-x500v2-esc-blheli-s-20a by a part with the same port rated >= 25.2 V: (none in the library); or use a lower-voltag
1.2 kg payload: {'error': 0, 'warning': 6, 'info': 4} -> {'error': 1, 'warning': 6, 'info': 3}
ERROR CURRENT_OVER_RATING ['pm02-to-pdb', 'wire part awg-12-silicone-wire']
  wire 'pm02-to-pdb' limited by wire part awg-12-silicone-wire: 72.2 A peak, 25.9 A sustained vs rating 20 A
  fix: use a heavier wire (rated >= 72 A): awg-8-silicone-wire; or shorten the load
WARNING THRUST_TO_WEIGHT ['mot1', 'mot2', 'mot3', 'mot4']
  max static thrust 44.7 N (motor tables scaled to pack nominal voltage) / weight 26.41 N (2.693 kg) = 1.69 (< 2.0; margins: 2.0 community rule of thumb, 1.3 leaves 23 % thrust for control)
  fix: cut 0.41 kg, or fit motors with >= 13.2 N max thrust each: tmotor-mn3508-kv700, tmotor-mn4010-kv475, iflight-xing2-2207-2755kv
```

More real findings, from the reference designs (`napl design check assemblies/so101.toml`, `.../aos5.toml`):

```
WARNING SERVO_TORQUE_MARGIN  [servo2, shoulder_lift]  servo2 (shoulder_lift): worst static gravity torque 0.833 N*m at [shoulder_pan 110 deg, ...] vs stall 1.323 N*m at 5 V (margin 1.59) < 2: no reserve for acceleration, payload or a voltage sag ...
        fix: use a servo with stall torque >= 1.67 N*m at 5 V, or lighten / shorten the arm
INFO    PAYLOAD_LIMIT  [servo2]  max extra end-effector mass 121 g at moving_jaw (limited by servo2, margin >= 1 at every sampled pose)
INFO    PROP_CLEARANCE  [prop1]  at rest the lowest disc (prop1) is 29 mm = 0.44 R above the ground: below z / R = 0.6, where the Cheeseman-Bennett model stops being valid; ground effect is strong, not estimated at take-off / landing ...
```

When the build itself is impossible (unknown part, missing mass) the result is `BUILD_FAILED` / `CHECKS_SKIPPED` instead of a crash;
a rule that crashes becomes `DRC_INTERNAL_ERROR` and the other rules still run.

---

## 5. Simulate quickly

`Design.simulate(kind)` (and the MCP tool `design_simulate`) builds the design and runs it in the kernel. There is no CLI for it;
it needs the C++ kernel.

* `"hover"` (multirotor): a built-in controller (altitude PID + attitude PD, ground-truth state; not your code) drives only the ESC
  throttles through the command bus. Reports pass (height within `tol` for `hold_s`), altitude error, hover throttle, currents,
  battery state-of-charge slope and endurance to the DRC's 20 % reserve.
* `"hold"` (arm): servos torque on at `pose` (default zero); reports sag (|joint - goal|), torque and current per servo.

```python
from napl.design import Design

r = Design.open("assemblies/x500v2.toml").simulate("hover", out="/tmp/guide-sim", altitude=2.0, hold_s=10)
keys = ("pass", "mass_kg", "thrust_to_weight", "hover_throttle", "hover_current_A", "altitude_error_max_m", "endurance_s")
print({k: round(r[k], 3) if isinstance(r[k], float) else r[k] for k in keys})

r = Design.open("assemblies/so101.toml").simulate("hold", out="/tmp/guide-sim")
for name, s in r["servos"].items():
    print(f"{name} {s['joint']:14s} sag {s['sag_max_rad']:.3f} rad  holding torque {s['torque_max_Nm']:.2f} N m  current {s['current_max_A']:.2f} A")
```
```
{'pass': True, 'mass_kg': 1.547, 'thrust_to_weight': 2.56, 'hover_throttle': 0.426, 'hover_current_A': 11.544, 'altitude_error_max_m': 0.099, 'endurance_s': 1284.18}
servo1 shoulder_pan   sag 0.000 rad  holding torque 0.00 N m  current 0.00 A
servo2 shoulder_lift  sag 0.118 rad  holding torque 0.58 N m  current 0.51 A
servo3 elbow_flex     sag 0.092 rad  holding torque 0.44 N m  current 0.39 A
servo4 wrist_flex     sag 0.022 rad  holding torque 0.12 N m  current 0.10 A
servo5 wrist_roll     sag 0.000 rad  holding torque 0.00 N m  current 0.00 A
servo6 gripper        sag 0.002 rad  holding torque 0.00 N m  current 0.00 A
```

Hover throttle 0.426 and 11.5 A match what PX4 flies (0.427, 11.58 A, section 6) and the static DRC estimate (12.5 A, throttle
50 %). Note the DRC's thrust-to-weight (2.95) is higher than the simulated one (2.56): the DRC scales the vendor thrust tables to the
pack's nominal voltage, the simulation calibrates thrust against the live battery (inferred, not separately tested). `simulate` with
no `out` leaves a temporary build directory under `/tmp` each call; pass `out`.

Whole-design sweeps (swap candidates, wind and turbulence, endurance Monte Carlo): `napl explore sweep|wind|mc` **[not run]**.
Declarative tests of a plain MJCF / URDF model (no design): `napl test spec.toml --model m.xml [--set path.attr=v1,v2]` **[not run]**.

### Benches against vendor tables

`napl bench thrust <motor>` reproduces a vendor thrust-stand table from the device physics. Datasheet values (Kv, R, I0) are fixed;
the prop's static C_T and C_Q are fitted on three rows; the error is reported on the held-out rows:

```
$ napl bench thrust holybro-2216-kv920
| motor | table | prop | rows | held-out | thrust MAPE % | current MAPE % | rpm MAPE % | power MAPE % | max err % | fit rows (throttle %) |
| holybro-2216-kv920 | main | holybro-1045-prop | 15 | 12 | 5.3 | 2.0 | 2.1 | 1.7 (elec) | 9.5 | 30.0, 65.0, 100.0 |
-> /workspace/wt/napl/src/build/bench/holybro-2216-kv920.json
$ napl bench hover
## X500 V2 hover estimate (open loop, 2216 fit)
- AUW 1.547 kg (...); per-motor thrust 3.79 N; avionics 0.146 A from the battery ...
- Start of hover: duty 0.419 (vendor-equivalent throttle 43 %), 5403 rpm, 2.82 A and 45.5 W per motor, pack 16.12 V, pack current 11.44 A.
- Endurance to cutoff: **23.7 min** (4850 mAh used; ...); to 80 % depth of discharge (SoC 0.2): **20.4 min**. Holybro: "Flight time: ~18 minutes hover ..."
```

`napl bench all` does every tabled motor and writes `build/bench` and `docs/validation/thrust-stands.md` (the committed copy has
17 motor / table pairs; thrust MAPE between 1.5 and 12 %, current 2 to 11 % on held-out rows). The X500 comparison with a real PX4
flight of the assembly is `docs/validation/x500-hover.md`.

### Fidelity report

```
$ napl report fidelity --out /tmp/guide-fid.md --json /tmp/guide-fid.json --build-dir /tmp/guide-fid-build --no-servo
fidelity: /tmp/guide-fid.md + /tmp/guide-fid.json: 632 parts, 9058 values (3.5 % estimated), 5 assemblies
```

The generated page (default `docs/fidelity.md`; the committed copy can lag the library, regenerate it) has: (1) the library's
fidelity counts by directory and category, (2) per assembly the mass-weighted fidelity by group, the published-figure targets
(frame kit 0.553 kg vs Holybro's 0.610 kg, inside tolerance) and every estimated value the build actually uses with its
rationale, (3) per model (battery, motor + ESC, rotor, servo, sensors, environment) the parameter provenance, validation
evidence and known gaps, (4) the determinism / replay tests. `--no-servo` skips the servo bench.

---

## 6. Program it

`napl program <scenario.toml>` runs **your** ordinary program (a Python script or anything with a command line) against a
simulated robot over the same protocol the real hardware speaks, and records evidence. The program is user code run on your
machine: this is CLI-only on purpose (there is no MCP tool for it; agents author declarative data, never code), and its limits are
resource limits plus a process-group kill, not a security sandbox.

```
napl program <scenario.toml> [--runs-dir DIR]      # exit 0 pass, 1 fail, 2 error
```

A scenario (TOML; paths relative to the scenario file; the full schema is the docstring of `python/napl/program.py`):

```toml
name = "so101-pick"
[model]                          # assembly = "x.toml" | mjcf = "robot.xml" | script = "scene.py" (build(outdir) -> MJCF)
script = "scene.py"
[interfaces.feetech]             # or dynamixel, or px4: exactly one (px4 + feetech together = the combo, below)
servos = { 1 = "/devices/servo1", 2 = "/devices/servo2" }
baud = 1000000
[[setup]]                        # optional SetAttr commands before the run (logged, replayed)
paths = ["/devices/servo1"]
attr = "torque_enable"
values = [1]
[program]
cmd = ["python", "{dir}/pick.py"]     # "python" = this interpreter, {dir} = the scenario directory
[limits]                         # all optional: wall_s sim_s rate cpu_s address_space_mb open_files
wall_s = 90
[[monitors]]                     # evidence the run is judged on; ALL must pass, with the program exiting 0
name = "cube_within_1cm_of_target"
path = "/world/cube"
attr = "pos"
indices = [0, 1]
ref = [0.20, 0.04]               # the norm of (value - ref) is the signal
op = "final"                     # always (every sample in [lo, hi]) | eventually | final (last sample)
hi = 0.01
```

The program finds the robot through environment variables: `NAPL_SERIAL` (a pty speaking Feetech STS or Dynamixel Protocol 2.0) or
`NAPL_MAVLINK` (`udp://:14540`, for MAVSDK). The same code runs on real hardware by pointing `NAPL_SERIAL` at `/dev/ttyACM0`.

**Monitors** sample simulation signals (`path` + `attr` [+ `index` / `indices` / `ref`, `frame` = a body prim]) or, for PX4, trace
columns: `alt` (height above the start), `dist_home` (horizontal m from the start), `tilt` (deg), or any `trace.csv` column.
`from` / `to` are sim seconds; `phase = "<program name>"` or `"fault:<fault name>"` makes them relative to that phase's start.
The monitors printed for a run are what it was judged on; the run passes only if the program exits 0, does not time out, and every
monitor passes.

**Replay hash.** Every run records a replay log and replays it with `napl-replay`; `result.json["replay"]` holds the final tick,
the state hash and `match`. A run whose log does not replay to its own hash is a determinism bug. (For PX4 the flight itself is not
lockstep-deterministic run to run, but each run's own log replays to its own hash.)

A run directory (`<runs-dir>/<name>-<time>/`) holds `result.json`, `program.log`, `replay.naplog`, `trace.csv` and for PX4
`bridge.log`, `px4.log`, `bridge.toml`.

### SO-101 over the Feetech STS protocol

`examples/so101/pick.py` is an unmodified-style protocol client (the same code with `NAPL_SERIAL=/dev/ttyACM0` drives the real arm,
after calibrating its constants): it moves to a hover pose, grips a 20 mm cube and carries it 9 cm. Run on the host (37 s wall):

```
$ napl program examples/so101/pick.scenario.toml --runs-dir /tmp/guide-runs
PASS so101-pick  (program rc=0, 37.5 s wall)
  ok   cube_within_1cm_of_target: final range [0.001316, 0.09] final 0.00132
  ok   cube_on_table: final range [0.007253, 0.09188] final 0.007593
  ok   cube_was_lifted: eventually range [0.007253, 0.09188] final 0.007593
  ok   no_servo_alarm:/devices/servo1: always range [0, 0] final 0
  ...
  replay: {'available': True, 'tick': 37498, 'state_hash': '5df7d9dd2915d846', 'match': True}
run dir: /tmp/guide-runs/so101-pick-20261007-113631
```

(`program.log` ended with `done: 980 cycles, status errors [], bad frames 0`.)

### Koch v1.1 over Dynamixel Protocol 2.0

`examples/koch/pick.py` uses the unmodified ROBOTIS `dynamixel_sdk` the way LeRobot's koch_follower does (torque off, operating mode,
PID, torque on, GroupSyncWrite / GroupSyncRead at 50 Hz); two XL430 and four XL330 servos are modelled per bus id:

```
$ napl program examples/koch/pick.scenario.toml --runs-dir /tmp/guide-runs
[scene] table top z = 0.000000 m
PASS koch-pick  (program rc=0, 21.0 s wall)
  ok   cube_within_1cm_of_target: final range [0.0001771, 0.1006] final 0.0006617
  ok   cube_on_table: final range [0.009653, 0.07017] final 0.009993
  ...
  replay: {'available': True, 'tick': 21125, 'state_hash': 'fc41aee7d231af09', 'match': True}
```

(The scene script's own `[scene]` line is printed first. This block combines two runs of the scenario: the header and monitor lines
are from the second, the replay line from the first; both passed. The hash of the second run was not captured.)

### X500 under real PX4 SITL

PX4 v1.17 SITL runs in lockstep with the physics (`napl-px4-bridge`, wall time = sim time); your program talks MAVLink with MAVSDK.
Several users can share the host: pick a free instance with `NAPL_PX4_INSTANCE` (the stack binds tcp 4560+N and udp 14540+N /
14550+N / 18570+N). This was run on the host with instance 6:

```
$ NAPL_PX4_INSTANCE=6 HOVER_HOLD_S=5 napl program examples/x500/hover.design.scenario.toml --runs-dir /tmp/guide-runs
PASS x500-hover-design  (program rc=0, 36.3 s wall)
  ok   max_altitude: always range [-0.004049, 4.747] final -0.0001147
  ok   reached_5m: eventually range [-0.004049, 4.747] final -0.0001147
  ok   stays_upright: always range [0.0001188, 1.596] final 0.004562
  ok   stays_near_home: always range [0, 0.4617] final 0.3698
  ok   landed_on_ground: final range [-0.004049, 4.747] final -0.0001147
  replay: {'available': True, 'tick': 36300, 'state_hash': 'd94050da7ddd5932', 'match': True}
```

(`HOVER_HOLD_S=5` shortens the hold from the script's default of 20 s.) The program is
`examples/x500/hover.py`: connect, wait for GPS + home, arm, take off to 5 m, hold, land.

#### Flying a saved design

You do not write a bridge file: `[design]` points at a saved design (any `Design.save` output or an assembly) and a bare
`[interfaces.px4]`; the world and the bridge config are generated from the design. Take the `my-quad.toml` saved in section 3 (the
quad whose GPS was removed and re-snapped, with its sensors regenerated) and fly it:

```toml
name = "my-quad-hover"
[design]
path = "my-quad.toml"
[interfaces.px4]
[program]
cmd = ["python", "<repo>/examples/x500/hover.py"]    # this run used the absolute path of the repo on the host
[[monitors]]
name = "reached_5m"
column = "alt"
op = "eventually"
lo = 4.5
[[monitors]]
name = "landed_on_ground"
column = "alt"
op = "final"
lo = -0.2
hi = 0.2
```
```
$ NAPL_PX4_INSTANCE=6 HOVER_HOLD_S=5 napl program fly.scenario.toml --runs-dir /tmp/guide-runs
PASS my-quad-hover  (program rc=0, 35.2 s wall)
  ok   reached_5m: eventually range [-0.003992, 4.888] final -0.0001147
  ok   landed_on_ground: final range [-0.003992, 4.888] final -0.0001147
  replay: {'available': True, 'tick': 35248, 'state_hash': '0062125763409695', 'match': True}
```

The generated `<run dir>/bridge.toml` starts with `# napl-px4-bridge config generated from design 'x500v2' by napl.design.px4cfg (motor
order checked against 4001_quad_x / 4902_napl_x500v2)` and lists `esc = ["esc1", "esc2", "esc3", "esc4"]`: PX4 motor N is the ESC wired
to `fc.pwmN`.

Before PX4 starts, the design is checked against PX4's quad-X airframe (`PX4_MOTOR_ORDER`, `PX4_PROP_DIRECTION`, `PX4_PROP_THRUST`,
`PX4_PWM_*`, `PX4_MOTOR_COUNT` / `NO_ROTOR` / `GEOMETRY`) and a mismatch is refused with its fix. Plugging PWM outputs 1 and 2 into each
other's ESC (checked without starting PX4, run on the host):

```python
from napl.design import Design
from napl.design.px4cfg import check_px4

d = Design.open("assemblies/x500v2.toml")
print("stock:", [f.code for f in check_px4(d.asm, d.build("/tmp/guide-b")["xml"])])
# plug the leads of PWM outputs 1 and 2 into each other's ESC
ends = {}
for x in d.doc["wires"]:
    for p in (x["from"], x["to"]):
        if p in ("fc.pwm1", "fc.pwm2"):
            ends[p] = (x["name"], x["to"] if x["from"] == p else x["from"])
for name, _ in ends.values():
    d.unwire(name)
d.autowire("fc.pwm1", ends["fc.pwm2"][1], included_in="esc2")
d.autowire("fc.pwm2", ends["fc.pwm1"][1], included_in="esc1")
for f in check_px4(d.asm, d.build("/tmp/guide-b")["xml"]):
    print(f.severity.upper(), f.code, f.where, "\n   ", f.message[:200], "\n    fix:", f.fix[:200])
```
```
stock: []
ERROR PX4_MOTOR_ORDER ['fc.pwm1', 'esc2']
    PX4 motor 1 (fc.pwm1 -> esc2) drives the rotor at rear-left, but in 4001_quad_x / 4902_napl_x500v2 motor 1 is front-right (CCW): PX4 would apply its roll / pitch / yaw commands to the wrong rotors and ...
    fix: wire fc.pwm1 -> esc1.signal (the front-right ESC); full order: fc.pwm1 -> esc1, fc.pwm2 -> esc2, fc.pwm3 -> esc3, fc.pwm4 -> esc4
ERROR PX4_MOTOR_ORDER ['fc.pwm2', 'esc1']
    PX4 motor 2 (fc.pwm2 -> esc1) drives the rotor at front-right, but in 4001_quad_x / 4902_napl_x500v2 motor 2 is rear-left (CCW): PX4 would apply its roll / pitch / yaw commands to the wrong rotors and ...
    fix: wire fc.pwm2 -> esc2.signal (the rear-left ESC); full order: fc.pwm1 -> esc1, fc.pwm2 -> esc2, fc.pwm3 -> esc3, fc.pwm4 -> esc4
```

Only the quad-X airframe 4902 is supported; PX4 parameters (rate gains, `BAT1_N_CELLS 4`) stay those of the airframe file. Measured
with this path (`tests/python/test_px4_design.py`): the stock X500 and a quad built from an empty design hover at throttle 0.425 /
11.5 A; with the four motors swapped for the KDE 2315XF-965 (`swap(auto=True)`) 0.409 / 11.3 A.

#### A companion computer, GPS-denied (vision odometry into EKF2)
A drone that navigates by its own cameras keeps its autonomy off the flight controller: the companion computer estimates the pose, plans, and talks
to PX4 over a second MAVLink link (`python/napl/companion.py`, MAVSDK, BSD-3). Three things differ from the flight above:

* **No GPS.** Leave `gnss` out of the bridge config (`px4/bridge_skydio.toml`): the bridge sends no HIL_GPS, and the scenario sets `EKF2_GPS_CTRL=0`.
* **Vision in.** `Link.odometry(...)` sends ODOMETRY (VIO estimator type, LOCAL_NED, FRD to NED attitude, the pad as origin); `EKF2_EV_CTRL=15` fuses
  position, height, velocity and yaw. `Link.obstacle_distance(...)` sends OBSTACLE_DISTANCE (72 bins of 5 deg, body FRD) built from a depth image.
  PX4 v1.17 delivers it to the `obstacle_distance` topic but its collision prevention runs only in Position mode, not Offboard, so the avoidance in an
  offboard flight is the companion planner's.
* **Companion data from the bridge.** The program needs the IMU stream PX4 gets: set `ctl = true` under `[interfaces.px4]` and the bridge's
  `--ctl` socket (`napl.ctl.RemoteSim`, NAPL_CTL in the program's environment) serves `imu <since>`, the buffered HIL_SENSOR gyro / accel records
  (250 Hz, the last 16 s, each with its sample counter). `[interfaces.px4.params]` passes `PX4_PARAM_*` overrides (applied before the airframe file).
  `monitor_trace = "companion.csv"` judges the monitors on the file the program writes (the mission signals) instead of the bridge's `trace.csv`.

Arming is done in Offboard (stream set-points, `offboard.start()`, `arm()`): MAVSDK's `is_armable` is judged in the current mode (Hold), which needs a
global position. A complete example with the measured results: `examples/skydio/README.md`, section "On PX4"; short test: `tests/python/test_px4_skydio.py`.

### Both arm and drone in one world (the showcase) **[not run: about 3 minutes, over the 2 minute budget; GPU render not needed]**

`examples/showcase/arm_loads_drone.scenario.toml` has `[interfaces.px4]` and `[interfaces.feetech]` together: one physics world
(the bridge's, in lockstep with PX4) holds the parts-built SO-101 on a shelf and the parts-built X500. Program `arm` drives the arm over
its Feetech pty (served from that same world over the bridge's control socket) and puts a 63 g stainless cube on the drone's top plate
while PX4 idles disarmed; program `fly` then flies the loaded drone (takeoff, hover, a 4 m hop, land). Monitors use
`phase = "arm"` / `"fly"` and `frame = "/world/bottom"` (the payload's position in the drone's frame). Run it with
`napl program examples/showcase/arm_loads_drone.scenario.toml` on the host with PX4 (the scenario header budgets about 60 s PX4
readiness, 90 s for the arm program, 420 s wall limit). What it looks like: `docs/showcase/arm_loads_drone.apng` (an animated PNG).

### Fault injection

`[[faults]]` in a scenario scripts a failure through the same bus (a logged, replayed `SetAttr`), so the replay hash still matches:

```toml
[[faults]]
name = "esc0_out"
kind = "motor_out"               # see the table
target = "/devices/esc0"
after = "program+3"              # sim s after the program started ...
log_marker = "HOLDING"           # ... counted from when this text first appears in its log (or: at = 40.0 sim s)
duration = 10.0                  # optional: undone after this long; else lasts to the end
```

| kind | what it does |
|---|---|
| `motor_out` | ESC open circuit: no drive, the rotor coasts on friction and prop torque |
| `motor_degraded` | chipped prop: thrust and torque x (1 - `thrust_loss`, default 0.3) |
| `battery_cell_fault` | one weak / dead cell: `r_extra_ohm`, `ocv_drop_v` in the Thevenin pack |
| `gnss_outage` / `gnss_jump` | no fix (the bridge sends fix_type 0) / position step `east, north, up` |
| `baro_drift` | pressure bias ramp `rate_pa_s` (default 1 Pa/s, about 0.085 m/s of altitude) |
| `imu_bias_step` | `gyro` / `accel` vectors added to the current bias |
| `servo_overload` / `servo_stall` | external torque `torque_nm` / joint seized |
| `comms_drop` | the serial pty discards the host's bytes for `duration`; the real SDK sees timeouts. Bus-side only: not in the replay log |

A monitor can look at the fault window: `phase = "fault:esc0_out"`, `from = -2.0` (2 s before it), `to = 10.0`. A fault that never fired
or failed to apply is a failed monitor. Two examples were run on the host.

SO-101 with a 0.5 s serial drop in the middle of a move (3 s wall):

```
$ napl program examples/faults/so101_comms_drop.scenario.toml --runs-dir /tmp/guide-runs
PASS so101-comms-drop  (program rc=0, 3.1 s wall)
  ok   pan_reaches_target: final range [0.00299, 0.5] final 0.00299
  ok   pan_lags_while_the_link_is_down: always range [0.1442, 0.1442] final 0.1442
  ok   no_servo_alarm: always range [0, 0] final 0
  replay: {'available': True, 'tick': 3091, 'state_hash': 'f0bb0dff74390b83', 'match': True}
```

The program's own log: `done: 13 read timeouts (longest run 13 cycles), last position 0.497 rad, bad frames 0`; `result.json["faults"]` is
`link_loss, comms_drop, start_t 0.766, end_t 1.271`. The arm held its last goal (0.144 rad of the 0.5 rad move) while the link was down
and resumed when it came back.

X500 under PX4 with one ESC open 3 s after the hold begins (PX4 instance 6, 55 s wall):

```
$ NAPL_PX4_INSTANCE=6 napl program examples/faults/x500_motor_out.scenario.toml --runs-dir /tmp/guide-runs
PASS x500-motor-out  (program rc=0, 55.3 s wall)
  ok   hovered_before_the_fault: always range [4.788, 4.961] final 4.961
  ok   crash_detected_altitude: eventually range [-0.4568, 4.969] final -0.4546
  ok   crash_detected_flipped: eventually range [0.4755, 180] final 179.9
  replay: {'available': True, 'tick': 55268, 'state_hash': 'b0147e29922c3b07', 'match': True}
```

The monitors detect the crash from ground truth; what PX4 itself noticed is in `program.log`: only `Preflight Fail: Attitude failure (roll)`.
With three of four rotors a quad cannot hold attitude, so this scenario passes by *confirming* the crash; the interesting fact is
that PX4 does not know (section 8). Other measured faults on PX4 (`docs/phase3-plan.md`, **not re-run for this guide**): a 10 s GNSS outage:
PX4 rides it out 5.0 s, then fails over to LAND and lands in place with 0.2 m drift; a +60 mOhm cell: PX4's battery voltage reading
falls 0.66 V.

---

## 7. Agents: the MCP server

```
napl mcp [--root DIR] [--http --host H --port N] [--allow-destructive] [--max-steps N] [--max-wall-s S] [--view]
```

stdio by default, Streamable HTTP with `--http`. **[Not run as a process; the tool list and the approval behaviour below were
exercised in-process against `NaplServer`.]** Every path argument is confined to `--root`; arguments are validated against the
advertised JSON schemas. The session has the capabilities `read` and `control`; destructive tools need approval. The tools, as the
server advertised them (read-only / destructive hints are the MCP annotations):

| group | tools | notes |
|---|---|---|
| simulation state | `get_entity_state`, `list_prims`, `render`, `raycast`, `get_run` | read-only (`render` needs a Vulkan device) |
| simulation control | `set_entity_state`, `step_simulation`, `snapshot`, `restore`, `spawn_entity`, `run_test`, `sweep` | `run_test` / `sweep` take a declarative TestSpec; evidence lands in `runs/<run_id>/` |
| design | `design_open`, `design_edit`, `design_suggest`, `design_check`, `design_build`, `design_simulate`, `design_undo` | `design_suggest` and `design_check` are read-only; the rest are control; builds go to `<root>/build/design` |
| **destructive** | `load_model`, `delete_entity`, `delete_run`, `design_save` | annotated `destructive_hint = true`; need approval |
| viewer (`--view`) | `play_simulation`, `pause_simulation` | added when the server is viewer-backed |

`napl program` is deliberately **not** a tool: agents write declarative data (designs, TestSpecs), never code.

**Approval.** A destructive tool asks the client for approval by MCP elicitation, or is pre-approved by `--allow-destructive`. Run
in-process against a design open on the server, `design_save` to a path under the root gave:

```
client declares no elicitation                  -> the call fails (MCPError: client did not declare the elicitation capability)
client declines                                 -> error "refused, nothing changed: not approved"
client accepts                                  -> {"saved": "build/guide-mcp.toml", "bytes": 32638}
a path outside the root (/tmp/x.toml)           -> error "path '/tmp/x.toml' is outside the server root"
```

(The saved file was deleted afterwards.) `design_edit` is one atomic batch of up to 100 ops (`add`, `remove`, `swap`, `mate`, `unmate`,
`wire`, `unwire`, `autowire`, `autowire_all`, `auto_devices`, `device_map`, `device`, `target`, `set`) and returns what changed plus
the DRC delta, so an agent loop is: `design_open` -> `design_suggest` -> `design_edit` -> read the DRC delta -> `design_simulate` ->
`design_undo` if worse -> `design_save` (approved). The E1 test (`tests/python/test_design_mcp.py`) runs exactly this.

---

## 8. Limits and fidelity

What the numbers are, and where they are known to be wrong. The generated fidelity report (section 5) is the authority and updates
with the library; the points below are the ones the project's own docs and commit messages state.

**Provenance.** At this writing 653 parts, 9189 tagged values: about 81 % datasheet, 5.5 % literature, 9.6 % derived, 3.6 %
estimated (the 3.5 % in the fidelity run above is the same count a few parts earlier). The frame group's mass is 90 % `derived`
(CAD density x printed volume, not weighed) and 8 % estimated; the X500 build uses 95 estimated values, each with a rationale in the report
(battery internal resistance, ESC loss figures, motor thermal constants, GNSS latency and origin, ...).

**Validated, and against what.**
* Thrust stands: the 2216 reproduces its vendor table to 5.3 % (thrust), 2.0 % (current) on held-out rows; across all 17
  tables 1.5 to 12 % thrust and 2 to 11 % current. The Reynolds / rotor-speed factor is a pooled trend from 7 UIUC APC props, not this
  prop.
* X500 hover under real PX4: throttle 0.427, per-motor current 2.86 A (4.7 % under the vendor table's 3.00 A), pack 11.58 A,
  endurance 21.4 min to a 20 % reserve. Holybro's "~18 min" claim cannot be reproduced or refuted: it states no AUW, battery C
  rating, cutoff or profile, and the vendor table itself implies 20.5 min. No real flight log exists.
* Servos: PID gains are calibrated by a bench; there is no measured step / stall data from a real SO-101.
  `pid_scale` (the register-to-duty scaling) is a calibration knob, not the vendor's number.

**Known gaps (stated in the docs / commit messages).**
* **XL430 limit cycle.** Koch's XL430 servos with the e-Manual default D = 4000 limit-cycle at 500 Hz (duty -0.22 / +0.35, +-1
  encoder count) while holding `shoulder_lift`; the mean torque still equals gravity to 1 %. Cause: the D term differences the
  quantised count error every 1 ms with no filter. A hold sweep puts the onset between D = 500 (quiet) and 1000. The real firmware's derivative
  filter and time base are unpublished, so it is not modelled; a real-servo measurement would set it. With P = 400 the XL330 elbow holds
  zero with a 5.6 deg sag.
* **Motor-out with no PX4 ESC feedback.** In SITL, PX4 gets no ESC telemetry: with an ESC open the quad flipped within about 1.3 s and
  PX4 printed only `Preflight Fail: Attitude failure (roll)`; no failsafe, no mode change (SITL gives PX4 no ESC feedback; that a
telemetry-capable ESC on hardware would tell PX4 more is an inference, not tested).
* **ESC / battery.** BLHeli_S is modelled as an averaged buck stage with estimated loss numbers (no ripple, no body-diode
  freewheel loss). Battery: one RC pair, no temperature, ageing or hysteresis; r0 / r1 are estimated, so voltage sag is not validated.
  3.1 W (6 %) of the vendor's electrical input at the hover point is unexplained.
* **Rotor aero.** Quasi-static C_T(J), C_Q(J): no inflow dynamics, flapping, blade flex, Mach, wake interaction; thrust at forward speed
  borrows an APC-SF shape. Ground effect is Cheeseman-Bennett, invalid below z / R = 0.6 (the DRC says so).
* **Sensors.** Flat-earth GNSS, one Gauss-Markov time constant for all axes; IMU without vibration or misalignment; one uniform magnetic
  field; ISA standard day, one frozen wind vector.
* **PX4 path.** Only the quad-X airframe 4902; the flight is not run-to-run deterministic (each run replays to its own hash);
  EKF height reference was moved to the barometer (`px4/params/napl_sitl.params`) because the datasheet-accuracy GNSS vertical noise
  made the hold miss its window; the cause was not isolated beyond that (inferred).
* **DRC.** Interference of meshes is sampled, not exact (a thin sliver between sample points is missed); the servo pose grid is a lower
  bound of the true worst case; no avionics current in the hover current; the 1.5 d tapped-engagement and the 2-pitch nut protrusion are rules of
  thumb, not sourced facts; thresholds named "design margin" are choices. A motor without a thrust table has no hover model.
* **Electrical (Koch).** The 12 V net does not carry the reducer's input current, so the Koch power checks undercount.
* **Programs** run as you, with resource limits only; the user's code is not sandboxed.

If a number matters to a decision, read its fidelity tag in the part file and the model's row in `docs/models.md`; if the tag says
`estimated`, the rationale tells you what it was based on and what would replace it.
