"""MCP server (`napl mcp`): the command bus as tools, for agents.

Tool input schemas are the flatc JSON Schema of the command tables (`_napl.schema_json`); annotations come from each
table's `cap` attribute (`_napl.command_cap`). The session is read+control: destructive commands are refused by the
C++ bus unless one call is elevated after approval (`--allow-destructive` or MCP elicitation). Every path argument is
resolved (symlinks, `..`) and must stay under the root. Long operations (run_test, sweep) run on a background thread
over a fork() of the loaded sim and are polled with get_run (no MCP Tasks: the v2 SDK dropped them).

Design tools (design_open / design_edit / design_suggest / design_check / design_build / design_simulate / design_save /
design_undo) work on one in-memory napl.design.Design: opening and editing it is `control`, checking / suggesting is
`read`, writing a design file (design_save) is destructive (approval as above). Builds go to <root>/build/design.

Run / Pause are exposed only by a host-backed server (`napl mcp --view`, NaplServer(view=True)): the World then lives
on a C++ Host thread, which can free-run it, so the tools `play_simulation` (Run{rate}) and `pause_simulation` (Pause)
exist (names after ROS 2 simulation_interfaces' PLAYING / PAUSED states). On a bare World both would be silent
no-ops, and a tool that does nothing would mislead an agent, so the default tool list does not have them.

With --view the viewer window runs on the main thread (GLFW needs it on macOS) and this server's asyncio loop on a
background thread; every tool call goes through the Host's queue, so what the agent does shows up live in the viewer,
and the viewer's own UI actions are bus commands (source "viewer") in the same replay log.
"""

import json
import re
import shutil
import threading
from pathlib import Path
from typing import Annotated

import numpy as np
from jsonschema import Draft202012Validator
from jsonschema.exceptions import best_match
from mcp.server import MCPServer
from mcp.server.elicitation import ElicitationResult
from mcp.server.mcpserver.context import Context
from mcp.server.mcpserver.exceptions import ToolError
from mcp.server.mcpserver.resolve import Elicit, Resolve
from mcp.server.mcpserver.tools import Tool
from mcp.server.mcpserver.utilities.func_metadata import ArgModelBase, FuncMetadata
from mcp.server.mcpserver.utilities.types import Image
from mcp_types import ToolAnnotations
from pydantic import BaseModel, ConfigDict, Field

from . import Sim, _napl
from .design import Design, DesignError
from .design.drc import _default_parts
from .report import run_id as report_run_id
from .report import write_report

SESSION_CAPS = ["read", "control"]
MAX_SWEEP_RUNS = 1000
MAX_STEPS_PER_CALL = 100000  # one step_simulation / run_test length; calls run on the event loop, so keep it finite
MAX_WALL_S = 60.0  # wall-clock budget of one run_test / sweep (also bounds step_simulation's rollback path)

# tool name -> (command table, description). Names follow ROS 2 simulation_interfaces where a verb exists.
COMMAND_TOOLS = {
    "get_entity_state": (
        "Query",
        "Read `attr` of the prim at `path` (e.g. /world/pole/hinge, qpos) -> values + SI unit. frame: '' or 'world', "
        "or a body prim path. Empty path lists all prims.",
    ),
    "set_entity_state": (
        "SetAttr",
        "Set `attr` of the prim at `path` (same attr names as get_entity_state).",
    ),
    "step_simulation": ("Step", "Advance the simulation by n ticks."),
    "snapshot": ("Snapshot", "Save the current state; returns snapshot_id."),
    "restore": ("Restore", "Restore a snapshot by id."),
    "load_model": (
        "LoadModel",
        "Load an MJCF/URDF model (path under the server root), replacing the current simulation. Destructive.",
    ),
    "spawn_entity": (
        "Spawn",
        "Add an MJCF/URDF model (path under the server root) to the running simulation as the new prim `at` "
        "('<parent body prim>/<name>', e.g. /world/arm2), optionally placed by pos (3) / quat (4: w x y z). Existing "
        "entities keep their state exactly; snapshots taken earlier can no longer be restored.",
    ),
    "delete_entity": (
        "Delete",
        "Remove the body prim `path` and everything beneath it from the running simulation. Destructive: its state "
        "is lost, and snapshots taken earlier can no longer be restored.",
    ),
    "run_test": (
        "RunTest",
        "Run a declarative test spec on a copy of the current state. Returns run_id at once; poll get_run. "
        "Evidence lands in runs/<run_id>/.",
    ),
    "sweep": (
        "Sweep",
        "Parameter sweep (grid or seeded random) of a test spec on copies of the current state. Returns run_id at "
        "once; poll get_run.",
    ),
    "render": (
        "Render",
        "Render the current state from a camera (pos, lookat: world frame, +z up; fovy degrees) and return a PNG image. "
        "Needs a Vulkan device (error otherwise).",
    ),
    "raycast": (
        "Raycast",
        "Lidar: distances in metres (-1 = miss) from `origin` along each direction (flat list of 3n numbers), "
        "full double precision.",
    ),
}
# Added only when the server is host-backed (see the module docstring).
HOST_TOOLS = {
    "play_simulation": (
        "Run",
        "Free-run the simulation in the background at `rate` sim-seconds per wall-second (0 = as fast as possible) "
        "until pause_simulation. step_simulation and the other tools still work meanwhile.",
    ),
    "pause_simulation": (
        "Pause",
        "Freeze the simulation tick (free-running stops at the next tick boundary; step_simulation still advances it).",
    ),
}
RUN_ID = {
    "type": "object",
    "properties": {"run_id": {"type": "string"}},
    "required": ["run_id"],
    "additionalProperties": False,
}
NO_ARGS = {"type": "object", "properties": {}, "additionalProperties": False}


# --- design tools (napl.design.Design): schemas -------------------------------------------------------------------------
def _obj(props: dict, required=()) -> dict:
    return {
        "type": "object",
        "properties": props,
        "required": list(required),
        "additionalProperties": False,
    }


_S, _N = {"type": "string"}, {"type": "number"}
# the numbers rule: a number is a "<inst>.<param>" reference or a provenance table (never a bare number)
PROVENANCE = _obj(
    {
        "value": {"anyOf": [_N, {"type": "array", "items": _N, "minItems": 1}]},
        "unit": _S,
        "fidelity": {
            "enum": ["datasheet", "literature", "measured", "derived", "estimated"]
        },
        "src": _S,
        "quote": _S,
        "derivation": _S,
        "rationale": _S,
    },
    ["value", "unit", "fidelity"],
)
JOINT = _obj(
    {
        "type": {"enum": ["hinge", "slide", "free"]},
        "range": {
            "anyOf": [
                PROVENANCE,
                _S,
                {
                    "type": "array",
                    "items": {"anyOf": [_S, PROVENANCE]},
                    "minItems": 2,
                    "maxItems": 2,
                },
            ]
        },
        "reverse": {"type": "boolean"},
        "spin": {"type": "boolean"},
    },
    ["type"],
)
FASTENERS = {
    "anyOf": [
        {"const": "auto"},
        _obj(
            {"part": _S, "count": {"type": "integer", "minimum": 1}, "nut": _S},
            ["part"],
        ),
    ]
}


# auto_devices fill: {device-name glob: {"set": {field: provenance table}, binding field: value}}
FILL = {"type": "object", "additionalProperties": {"type": "object"}}


def _op(name: str, props: dict, required=()) -> dict:
    return _obj({"op": {"const": name}, **props}, ["op", *required])


EDIT_OPS = [
    _op(
        "add",
        {
            "part": _S,
            "id": _S,
            "to": _S,
            "port": _S,
            "k": {"type": "integer", "minimum": 0},
            "root": {"type": "boolean"},
            "group": _S,
            "offboard": {"type": "boolean"},
            "strict": {"type": "boolean"},
            "force": {"type": "boolean"},
            "joint": JOINT,
            "fasteners": FASTENERS,
            "nut": _S,
            "mate": _S,
        },
        ["part"],
    ),
    _op("remove", {"id": _S}, ["id"]),
    _op(
        "swap",
        {
            "id": {"anyOf": [_S, {"type": "array", "items": _S, "minItems": 1}]},
            "part": _S,
            "auto": {"type": "boolean"},
            "overwrite": {"type": "boolean"},
            "fill": FILL,
        },
        ["id", "part"],
    ),
    _op(
        "mate",
        {
            "a": _S,
            "b": _S,
            "k": {"type": "integer", "minimum": 0},
            "joint": JOINT,
            "fasteners": FASTENERS,
            "nut": _S,
            "name": _S,
            "closure": {"type": "boolean"},
            "force": {"type": "boolean"},
        },
        ["a", "b"],
    ),
    _op("unmate", {"name": _S, "a": _S, "b": _S}),
    _op(
        "wire",
        {
            "from": _S,
            "to": _S,
            "part": _S,
            "length": {"anyOf": [_S, PROVENANCE]},
            "conductors": {"type": "integer", "minimum": 1},
            "included_in": _S,
            "connectors": _obj({"from": _S, "to": _S}),
            "name": _S,
        },
        ["from", "to", "part", "length"],
    ),
    _op("unwire", {"name": _S}, ["name"]),
    _op(
        "autowire",
        {
            "a": _S,
            "b": _S,
            "kind": {"enum": ["power", "signal"]},
            "part": _S,
            "length": {"anyOf": [_S, PROVENANCE]},
            "included_in": _S,
        },
        ["a"],
    ),
    _op(
        "autowire_all",
        {
            "kind": {"enum": ["power", "signal"]},
            "included_in": {"type": "object", "additionalProperties": _S},
        },
    ),
    _op(
        "device_map",
        {"name": _S, "map": {"type": "object", "additionalProperties": _S}},
        ["name", "map"],
    ),
    _op("device", {"device": {"type": "object"}}, ["device"]),
    _op(
        "auto_devices",
        {
            "scope": {"anyOf": [_S, {"type": "array", "items": _S, "minItems": 1}]},
            "overwrite": {
                "anyOf": [{"type": "boolean"}, {"type": "array", "items": _S}]
            },
            "fill": FILL,
            "ocv": _S,
        },
    ),
    _op(
        "target",
        {
            "name": _S,
            "value": {"anyOf": [_S, PROVENANCE]},
            "groups": {"type": "array", "items": _S},
            "all": {"type": "boolean"},
        },
        ["name", "value"],
    ),
    _op(
        "set",
        {
            "id": _S,
            "param": _S,
            "value": {"anyOf": [_N, PROVENANCE]},
            "unit": _S,
            "fidelity": PROVENANCE["properties"]["fidelity"],
            "src": _S,
            "quote": _S,
            "derivation": _S,
            "rationale": _S,
            "unset": {"type": "boolean"},
        },
        ["id", "param"],
    ),
]
DESIGN_TOOLS = {
    "design_open": (
        "control",
        "Open an assembly TOML (path under the server root) as the working design (replaces the current one), or start "
        "an EMPTY design with `new` = a name (no path): then design_edit add {part, root = true} places the frame and "
        "everything else snaps to it. Optional `parts` = a parts directory under the root (default: <root>/parts, else "
        "the repo's). Returns instance counts and the design-rule findings.",
        _obj({"path": _S, "parts": _S, "new": _S}),
    ),
    "design_edit": (
        "control",
        "Edit the in-memory design: a list of ops applied atomically (all or none), then the design-rule check re-runs. "
        "ops: add {part, id?, to?, port?, k?, force?}: with `to` ('inst' or 'inst.port') the part snaps to the best "
        "placement: every open compatible port pair x rotation k is built and scored (interference with non-mated "
        "instances, clearance, upright, centre of mass inboard); the ranked alternatives with reasons are in "
        "changes[].snap, and the edit is refused when every placement interferes (force = true places anyway); "
        "remove {id}; swap {id | [ids], part}: re-mates "
        "by port name then compatible type and reports kept / lost mates + wires and references that no longer "
        "resolve; mate {a, b, k?, joint?, fasteners?, force?} (k omitted: the best rotation is searched like add); "
        "unmate {name | a+b}; wire {from, to, part, length}; unwire {name}; autowire {a, b?, kind?, part?, length?}: "
        "wire the open power / signal port a to the open port that needs it (or b); wire part = lightest library wire "
        "rated for the DRC's worst-case current x 1.2, length = port distance x 1.3 rounded up, mating connectors "
        "pinned; the reasons are in changes[].reasons; autowire_all {kind?}: every open port with one sensible target; "
        "changes[] lists wired / ambiguous (not guessed) / unmatched; "
        "auto_devices {scope?, overwrite?, fill?, ocv?}: derive the simulation device bindings from the parts and the "
        "topology (battery, ESC + motor, rotor + prop with spin direction, servos, power boards, avionics loads, sensors); "
        "generated bindings are regenerated, hand-written ones kept unless overwrite; fields no part carries are listed "
        "under `unset` (supply them with fill = {device glob: {set: {field: provenance table}}}); swap {id, part, auto?} "
        "lists stale_devices and with auto = true refreshes the generated bindings in the same edit; "
        "assembly {base?: free | fixed, timestep?, description?}: the design's header (a fixed base welds the root to the world); "
        "set {id, param, value, unit, fidelity, src+quote | derivation | rationale}. Numbers rule: a bare "
        "number is refused; use a provenance table. Returns what changed and the DRC delta (new / resolved / changed "
        "findings).",
        _obj(
            {
                "ops": {
                    "type": "array",
                    "items": {"oneOf": EDIT_OPS},
                    "minItems": 1,
                    "maxItems": 100,
                }
            },
            ["ops"],
        ),
    ),
    "design_suggest": (
        "read",
        "Ranked library parts for `target`: 'inst.port' = parts (and the port they would use) that mate / connect it, "
        "ratings checked on power ports; 'inst' = replacement candidates for a swap, ranked by fit (mates / wires kept, "
        "references resolved, ratings, mass delta); a joint name (a mate with `joint`) = servos ranked by holding-torque "
        "margin at that joint's worst static pose.",
        _obj(
            {
                "target": _S,
                "limit": {"type": "integer", "minimum": 1, "maximum": 100},
                "category": _S,
            },
            ["target"],
        ),
    ),
    "design_check": (
        "read",
        "Run the design-rule check on the in-memory design. `severity` = the least severe level to list.",
        _obj({"severity": {"enum": ["error", "warning", "info"]}}),
    ),
    "design_build": (
        "control",
        "Build the in-memory design to <root>/build/design/<name>.xml (+ .report.json). Returns paths, mass and finding counts.",
        NO_ARGS,
    ),
    "design_simulate": (
        "control",
        "Build, then simulate in the kernel. kind 'hover' (multirotor): built-in altitude + attitude controller on the "
        "ESC throttles; reports hover throttle, current, altitude error, endurance from the battery SoC slope; pass = "
        "height within +-tol of `altitude` for `hold_s`. kind 'hold' (arm): servos torque-on at `pose` (rad per servo / "
        "joint name, default 0); reports sag and current.",
        _obj(
            {
                "kind": {"enum": ["hover", "hold"]},
                "altitude": {"type": "number", "exclusiveMinimum": 0, "maximum": 100},
                "climb_s": {"type": "number", "minimum": 0.5, "maximum": 60},
                "hold_s": {"type": "number", "exclusiveMinimum": 0, "maximum": 60},
                "tol": {"type": "number", "exclusiveMinimum": 0},
                "pose": {"type": "object", "additionalProperties": _N},
                "settle_s": {"type": "number", "minimum": 0.1, "maximum": 30},
                "window_s": {"type": "number", "minimum": 0.05, "maximum": 10},
                "sag_tol": {"type": "number", "exclusiveMinimum": 0},
            },
            ["kind"],
        ),
    ),
    "design_save": (
        "destructive",
        "Write the in-memory design to a .toml file under the server root (overwrites). Destructive: needs client "
        "elicitation or server --allow-destructive.",
        _obj({"path": _S}, ["path"]),
    ),
    "design_undo": (
        "control",
        "Undo the last edit (or redo it with redo = true); returns the DRC delta of the step.",
        _obj({"redo": {"type": "boolean"}}),
    ),
}


class _Args(ArgModelBase):
    """Accepts any arguments: the SDK already validated them against the tool's JSON Schema."""

    model_config = ConfigDict(extra="allow", arbitrary_types_allowed=True)

    def model_dump_one_level(self) -> dict:
        return dict(self.model_extra or {})


class _Confirm(BaseModel):
    confirm: bool = Field(description="Allow this destructive operation")


def _schema(table: str) -> dict:
    """Input schema of one command table: its flatc definition plus the definitions it references."""
    defs = json.loads(_napl.schema_json)["definitions"]
    keep, todo = {}, [f"napl_{table}"]
    while todo:
        n = todo.pop()
        if n not in keep:
            keep[n] = defs[n]
            todo += re.findall(r"#/definitions/(\w+)", json.dumps(defs[n]))
    return {**keep[f"napl_{table}"], "definitions": keep}


class NaplServer:
    def __init__(
        self,
        root=None,
        allow_destructive: bool = False,
        max_steps_per_call: int = MAX_STEPS_PER_CALL,
        max_wall_s: float = MAX_WALL_S,
        view: bool = False,
        parts=None,
    ):
        self.max_steps_per_call = max_steps_per_call
        self.max_wall_s = max_wall_s
        self.root = Path(root or Path.cwd()).resolve()
        self.allow_destructive = allow_destructive
        self.view = view  # host-backed: the World lives on a Host thread (viewer-capable); adds play/pause tools
        self.loaded = (
            threading.Event()
        )  # set when the first model is loaded (the viewer opens then)
        self.sim = None  # napl.Sim with SESSION_CAPS (one world; calls run on the event loop, so they are serialised)
        self.model = None
        self.jobs = {}  # run_id -> {"status", "result" | "error"}
        self.design = None  # the working napl.design.Design (one per server)
        self.design_lock = threading.RLock()  # the viewer's Design window (--view) edits the same Design from its own thread
        self.parts = parts  # default parts directory of design_open (None: <root>/parts if it exists, else the repo's)
        tools = []
        for name, (table, desc) in {
            **COMMAND_TOOLS,
            **(HOST_TOOLS if view else {}),
        }.items():
            if name in ("load_model", "delete_entity"):
                tools.append(self._approved_tool(name, table, desc))
                continue
            fn = {
                "spawn_entity": self._spawn,
                "run_test": self._run_test,
                "sweep": self._sweep,
                "render": self._render,
                "raycast": self._raycast,
            }.get(name)
            tools.append(
                self._tool(
                    name,
                    fn or self._command(table),
                    _schema(table),
                    _napl.command_cap(table),
                    desc,
                )
            )
        tools += [
            self._tool(
                "list_prims",
                self._list_prims,
                NO_ARGS,
                "read",
                "List every prim path in the loaded model.",
            ),
            self._tool(
                "get_run",
                self._get_run,
                RUN_ID,
                "read",
                "Status (pending/running/done/error) and result of a run_id.",
            ),
            self._delete_run_tool(),
            self._save_design_tool(),
        ]
        for name, (cap, desc, schema) in DESIGN_TOOLS.items():
            if (
                name != "design_save"
            ):  # destructive: built with its approval resolver above
                fn = self._validated(self._locked(getattr(self, f"_{name}")), schema)
                tools.append(self._tool(name, fn, schema, cap, desc))
        self.server = MCPServer("napl", tools=tools)

    @staticmethod
    def _annotations(name, cap) -> ToolAnnotations:
        return ToolAnnotations(
            title=name,
            read_only_hint=cap == "read",
            destructive_hint=cap == "destructive",
        )

    def _locked(self, fn):
        async def locked(ctx, **args):
            with self.design_lock:  # no await inside the design tools, so the lock is never held across a suspension
                return await fn(ctx, **args)

        return locked

    @staticmethod
    def _validated(fn, schema):
        """The SDK does not check arguments against the advertised schema (the command tools rely on the C++ bus for that);
        the design tools do it here, reporting the most relevant violation."""
        validator = Draft202012Validator(schema)

        async def checked(ctx, **args):
            err = best_match(validator.iter_errors(args))
            if (
                err is not None
                and err.validator == "oneOf"
                and isinstance(err.instance, dict)
            ):
                # an edit op: report the violation inside the branch its "op" names, not the oneOf as a whole
                branch = [
                    e
                    for e in err.context
                    if err.validator_value[e.relative_schema_path[0]]
                    .get("properties", {})
                    .get("op", {})
                    .get("const")
                    == err.instance.get("op")
                ]
                err = best_match(branch) or err
            if err is not None:
                where = "/".join(str(p) for p in err.absolute_path) or "<arguments>"
                hint = (
                    " (numbers rule: a number here is a '<inst>.<param>' reference or a provenance table "
                    "{value, unit, fidelity, src+quote | derivation | rationale}, never a bare number)"
                    if isinstance(err.instance, (int, float))
                    and not isinstance(err.instance, bool)
                    and err.validator in ("anyOf", "oneOf")
                    else ""
                )
                raise ToolError(
                    f"invalid arguments at {where}: {err.message[:300]}{hint}"
                )
            return await fn(ctx, **args)

        return checked

    @classmethod
    def _tool(cls, name, fn, schema, cap, desc) -> Tool:
        ann = cls._annotations(name, cap)
        return Tool(
            fn=fn,
            name=name,
            description=desc,
            parameters=schema,
            fn_metadata=FuncMetadata(arg_model=_Args),
            is_async=True,
            context_kwarg="ctx",
            annotations=ann,
        )

    # --- sandbox and permissions -------------------------------------------------------------------------------

    def _confine(self, p) -> Path:
        r = (
            self.root / str(p)
        ).resolve()  # resolves symlinks and `..`; an absolute `p` replaces the root
        if not r.is_relative_to(self.root):
            raise ToolError(f"path '{p}' is outside the server root")
        return r

    def _run_dir(self, run_id: str) -> Path:
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", run_id) or run_id in (".", ".."):
            raise ToolError(f"invalid run_id '{run_id}'")
        d = self._confine("runs") / run_id
        if d.is_symlink():
            raise ToolError(f"invalid run_id '{run_id}'")
        return d

    # Destructive tools take an `approval` parameter filled by an SDK resolver before the body runs: with
    # --allow-destructive it is pre-accepted, otherwise the client is asked (MCP elicitation). A client that cannot
    # be asked gets a protocol error and the body never runs; decline/cancel is refused by _require.
    def _ask(self, what: str):
        if self.allow_destructive:
            return _Confirm(
                confirm=True
            )  # a plain value is injected as an accepted outcome
        return Elicit(f"napl wants to {what}. Allow?", _Confirm)

    async def _ask_load(self, path: str):
        p = self._confine(
            path
        )  # sandbox first: never ask the user to approve an outside path
        return self._ask(f"load model '{p.name}' (replaces the current simulation)")

    async def _ask_delete_entity(self, path: str):
        return self._ask(
            f"delete '{path}' and everything beneath it from the simulation"
        )

    async def _ask_delete(self, run_id: str):
        self._run_dir(run_id)
        return self._ask(f"delete run '{run_id}' (its runs/ directory)")

    async def _ask_save(self, path: str):
        p = self._confine(path)
        return self._ask(
            f"write the design to '{p.relative_to(self.root)}'"
            + (" (overwrites the existing file)" if p.exists() else "")
        )

    def _save_design_tool(self) -> Tool:
        async def design_save(
            ctx: Context,
            path: str,
            approval: Annotated[ElicitationResult[_Confirm], Resolve(self._ask_save)],
        ):
            self._require(approval)
            return self._design_save_to(path)

        cap, desc, schema = DESIGN_TOOLS["design_save"]
        t = Tool.from_function(
            design_save,
            name="design_save",
            description=desc,
            annotations=self._annotations("design_save", cap),
        )
        t.parameters = schema
        return t

    @staticmethod
    def _require(approval) -> None:
        if approval.action != "accept" or not approval.data.confirm:
            raise ToolError("refused, nothing changed: not approved")

    def _approved_tool(self, name: str, table: str, desc: str) -> Tool:
        """A destructive command tool taking one `path`: approval is resolved (and the sandbox checked) before the body."""
        ask, run = {
            "load_model": (self._ask_load, self._load),
            "delete_entity": (self._ask_delete_entity, self._delete_entity),
        }[name]

        async def fn(
            ctx: Context,
            path: str,
            approval: Annotated[ElicitationResult[_Confirm], Resolve(ask)],
        ):
            self._require(approval)
            return run(path)

        t = Tool.from_function(
            fn,
            name=name,
            description=desc,
            annotations=self._annotations(name, _napl.command_cap(table)),
        )
        t.parameters = _schema(table)  # the flatc schema, not the stub's signature
        return t

    def _delete_run_tool(self) -> Tool:
        async def delete_run(
            ctx: Context,
            run_id: str,
            approval: Annotated[ElicitationResult[_Confirm], Resolve(self._ask_delete)],
        ):
            self._require(approval)
            return self._delete(run_id)

        return Tool.from_function(
            delete_run,
            name="delete_run",
            description="Delete the runs/<run_id>/ evidence directory. Destructive: needs client elicitation or "
            "server --allow-destructive.",
            annotations=self._annotations("delete_run", "destructive"),
        )

    def _need_sim(self) -> Sim:
        if self.sim is None:
            raise ToolError("no model loaded: call load_model first")
        return self.sim

    def _call(self, cmd: str, args: dict, elevate: bool = False) -> dict:
        r = self._need_sim().call({"cmd_type": cmd, "cmd": args}, elevate=elevate)
        if not r["ok"]:
            raise ToolError(f"{r['error']}: {r.get('message', '')}")
        return {
            "tick": r["tick"],
            **{k: r[k] for k in ("unit", "prims", "snapshot_id") if r.get(k)},
        }

    # --- tools -------------------------------------------------------------------------------------------------

    def _command(self, table: str):
        async def fn(ctx, **args):
            if table == "Query" and args.get(
                "path"
            ):  # direct read: full float64 precision (the JSON edge rounds)
                try:
                    values, unit = self._need_sim().query(
                        args["path"], args.get("attr", ""), args.get("frame", "")
                    )
                except (KeyError, ValueError) as e:
                    raise ToolError(str(e).strip("'\"")) from None
                return {"values": values.tolist(), "unit": unit}
            return self._call(table, args)

        return fn

    async def _spawn(self, ctx, **args):
        return self._call(
            "Spawn", {**args, "path": str(self._confine(args.get("path", "")))}
        )

    def _delete_entity(self, path: str) -> dict:
        return self._call(
            "Delete", {"path": path}, elevate=True
        )  # approved by the resolver

    async def _render(self, ctx, **args):
        sim = self._need_sim()
        if not args.get("pos") or not args.get("lookat"):
            raise ToolError("render needs pos and lookat (3 numbers each)")
        try:  # runs on the event loop like every sim call (a Sim is not thread-safe)
            rgba, depth = sim.render(
                args["pos"],
                args["lookat"],
                args.get("fovy", 45.0),
                args.get("width", 320),
                args.get("height", 240),
            )
        except (ValueError, RuntimeError) as e:
            raise ToolError(str(e)) from None
        h, w = depth.shape
        info = {"width": w, "height": h, "center_depth_m": float(depth[h // 2, w // 2])}
        return [Image(data=_napl.encode_png(rgba), format="png"), info]

    async def _raycast(self, ctx, origin=None, directions=None):
        sim = self._need_sim()
        if not origin or not directions or len(directions) % 3:
            raise ToolError(
                "raycast needs origin (3 numbers) and directions (3n numbers)"
            )
        try:
            d = sim.raycast(
                origin, np.asarray(directions, dtype=np.float64).reshape(-1, 3)
            )
        except (ValueError, RuntimeError) as e:
            raise ToolError(str(e)) from None
        return {"distances": d.tolist()}

    # --- design tools ------------------------------------------------------------------------------------------
    def _need_design(self) -> Design:
        if self.design is None:
            raise ToolError("no design open: call design_open first")
        return self.design

    @staticmethod
    def _counts(findings) -> dict:
        sev = [f["severity"] if isinstance(f, dict) else f.severity for f in findings]
        return {s: sev.count(s) for s in ("error", "warning", "info")}

    def _parts_dir(self, parts) -> Path:
        if parts:
            return self._confine(parts)
        if self.parts:
            return Path(self.parts)
        return (
            self.root / "parts" if (self.root / "parts").is_dir() else _default_parts()
        )

    async def _design_open(self, ctx, path=None, parts=None, new=None):
        if (path is None) == (new is None):
            raise ToolError("design_open: give exactly one of `path` or `new`")
        pd = self._parts_dir(parts)
        p = self._confine(path) if path else None
        try:
            d = Design.open(p, pd) if p else Design.new(new, pd)
            findings = d.check()
        except DesignError as e:
            raise ToolError(str(e)) from None
        self.design = d
        doc = d.doc
        return {
            "name": d.asm.name,
            "path": str(p.relative_to(self.root)) if p else None,
            "parts_dir": str(pd),
            "library_parts": len(d.lib.parts),
            "instances": [{"id": i["id"], "part": i["part"]} for i in doc["instances"]],
            "mates": len(doc.get("mates", [])),
            "wires": len(doc.get("wires", [])),
            "findings": self._counts(findings),
            "errors": [f.as_dict() for f in findings if f.severity == "error"],
        }

    async def _design_edit(self, ctx, ops):
        d = self._need_design()
        try:
            r = d.edit(ops)
        except DesignError as e:
            raise ToolError(
                f"{e}" if "nothing changed" in str(e) else f"{e} (nothing changed)"
            ) from None
        return {**r.as_dict(), "undo_depth": len(d._undo)}

    async def _design_suggest(self, ctx, target, limit=10, category=None):
        try:
            return {
                "target": target,
                "suggestions": self._need_design().suggest(target, limit, category),
            }
        except DesignError as e:
            raise ToolError(str(e)) from None

    async def _design_check(self, ctx, severity="info"):
        keep = {"error": 0, "warning": 1, "info": 2}
        fs = self._need_design().check()
        return {
            "counts": self._counts(fs),
            "findings": [f.as_dict() for f in fs if keep[f.severity] <= keep[severity]],
        }

    async def _design_build(self, ctx):
        d = self._need_design()
        try:
            r = d.build(self._confine("build/design"))
        except DesignError as e:
            raise ToolError(str(e)) from None
        rep = r["report"]
        return {
            "assembly": rep["assembly"],
            "xml": str(r["xml"].relative_to(self.root)),
            "report": str(r["report_path"].relative_to(self.root)),
            "mass_kg": rep["mass"]["total"],
            "instances": len(rep["instances"]),
            "warnings": [w["code"] for w in rep["warnings"]],
            "unresolved": len(rep["unresolved"]),
            "findings": self._counts(rep["findings"]),
        }

    async def _design_simulate(self, ctx, kind, **opts):
        d = self._need_design()
        allowed = {
            "hover": {"altitude", "climb_s", "hold_s", "tol"},
            "hold": {"pose", "settle_s", "window_s", "sag_tol"},
        }[kind]
        if set(opts) - allowed:
            raise ToolError(
                f"simulate {kind}: unknown options {sorted(set(opts) - allowed)} (allowed: {sorted(allowed)})"
            )
        if kind == "hover":
            opts["wall_s"] = self.max_wall_s or 600.0
        try:
            return d.simulate(kind, out=self._confine("build/design"), **opts)
        except (DesignError, RuntimeError) as e:
            raise ToolError(str(e)) from None

    def _design_save_to(self, path: str) -> dict:
        p = self._confine(path)
        if p.suffix != ".toml":
            raise ToolError("design_save writes a .toml file")
        try:
            p.parent.mkdir(parents=True, exist_ok=True)
            with self.design_lock:
                self._need_design().save(p)
        except (DesignError, OSError) as e:
            raise ToolError(str(e)) from None
        return {"saved": str(p.relative_to(self.root)), "bytes": p.stat().st_size}

    async def _design_undo(self, ctx, redo=False):
        d = self._need_design()
        try:
            return (d.redo() if redo else d.undo()).as_dict()
        except DesignError as e:
            raise ToolError(str(e)) from None

    async def _list_prims(self, ctx):
        return {"prims": self._call("Query", {"path": ""}).get("prims", [])}

    def _load(self, path: str) -> dict:
        p = self._confine(path)
        try:
            if (
                self.sim is None
            ):  # first load: nothing to replace; the session is created restricted
                self.sim = Sim.load(
                    p,
                    root=self.root,
                    max_sweep_runs=MAX_SWEEP_RUNS,
                    max_wall_s=self.max_wall_s,
                    caps=SESSION_CAPS,
                    max_steps_per_call=self.max_steps_per_call,
                    host=self.view,
                )
            else:  # the C++ bus checks the path against the session root and the (elevated) capability
                self._call("LoadModel", {"path": str(p)}, elevate=True)
        except RuntimeError as e:
            raise ToolError(str(e)) from None
        self.model = p
        self.loaded.set()
        return {"model": str(p.relative_to(self.root)), "tick": self.sim.tick}

    async def _run_test(self, ctx, spec=None):
        if not spec:
            raise ToolError("run_test needs a spec")
        return await self._start(
            ctx, "run_test", report_run_id(spec, self.model), {"spec": spec}
        )

    async def _sweep(self, ctx, **args):
        if not args.get("spec"):
            raise ToolError("sweep needs a spec")
        name = "sweep-" + (args["spec"].get("name") or "test")
        return await self._start(
            ctx, "sweep", report_run_id({"name": name, "args": args}, self.model), args
        )

    async def _start(self, ctx, kind: str, rid: str, args: dict) -> dict:
        sim = self._need_sim()
        old = self.jobs.get(rid)
        if old and old["status"] in ("pending", "running"):
            return {"run_id": rid, "status": old["status"]}
        fork = sim.fork()  # a Sim is not thread-safe; the job gets its own copy, the live sim is untouched
        job = self.jobs[rid] = {"status": "pending"}
        threading.Thread(
            target=self._work,
            args=(rid, job, kind, fork, args, self.model, self._confine("runs")),
            daemon=True,
        ).start()
        await ctx.report_progress(0, 1, f"{kind} {rid} started; poll get_run")
        return {"run_id": rid, "status": "pending"}

    @staticmethod
    def _work(rid, job, kind, fork, args, model, runs_dir) -> None:
        job["status"] = "running"
        try:
            if kind == "run_test":
                write_report(fork.run_test(args["spec"]), args["spec"], model, runs_dir)
            else:
                opts = {
                    k: args[k]
                    for k in ("mode", "n", "seed", "threads", "base")
                    if k in args
                }
                rows = fork.sweep(args["spec"], args.get("params", []), **opts)
                (runs_dir / rid).mkdir(parents=True, exist_ok=True)
                (runs_dir / rid / "result.json").write_text(
                    json.dumps(
                        {"run_id": rid, "model": str(model), "rows": rows}, indent=2
                    )
                )
            job.update(
                result=json.loads((runs_dir / rid / "result.json").read_text()),
                status="done",
            )
        except Exception as e:
            job.update(error=f"{type(e).__name__}: {e}", status="error")

    async def _get_run(self, ctx, run_id):
        d = self._run_dir(run_id)
        if run_id in self.jobs:
            return {"run_id": run_id, **self.jobs[run_id]}
        if (d / "result.json").is_file():  # written by an earlier server process
            return {
                "run_id": run_id,
                "status": "done",
                "result": json.loads((d / "result.json").read_text()),
            }
        raise ToolError(f"not_found: no run '{run_id}'")

    def _delete(self, run_id: str) -> dict:
        d = self._run_dir(run_id)
        if self.jobs.get(run_id, {}).get("status") in ("pending", "running"):
            raise ToolError(f"run '{run_id}' is still running")
        if not d.is_dir():
            raise ToolError(f"not_found: no run '{run_id}'")
        shutil.rmtree(d)
        self.jobs.pop(run_id, None)
        return {"deleted": run_id}


def serve(
    root=None,
    http: bool = False,
    host: str = "127.0.0.1",
    port: int = 8765,
    allow_destructive: bool = False,
    max_steps_per_call: int = MAX_STEPS_PER_CALL,
    max_wall_s: float = MAX_WALL_S,
    view: bool = False,
) -> None:
    if view and not _napl.has_viewer:
        raise RuntimeError("this build of napl has no viewer, so --view is unavailable")
    srv = NaplServer(root, allow_destructive, max_steps_per_call, max_wall_s, view)

    def run() -> None:
        if http:
            srv.server.run(transport="streamable-http", host=host, port=port)
        else:
            srv.server.run()

    if not view:
        run()
        return

    # Viewer on this (the main) thread, the MCP asyncio loop on a background one; the viewer opens once an agent has
    # loaded a model. Closing the window ends the process; the client going away closes the window.
    def serve_mcp() -> None:
        try:
            run()
        finally:
            srv.loaded.set()  # wake the main thread if no model was ever loaded
            if srv.sim is not None:
                srv.sim.close_viewer()

    threading.Thread(target=serve_mcp, name="napl-mcp", daemon=True).start()
    srv.loaded.wait()
    if srv.sim is not None:
        # The window's Design panel edits the server's own Design (whatever design_open made current), under its lock; a
        # change the agent makes shows up in the window within a second, and the other way round.
        from .design.viewer import Backend

        backend = Backend(
            lambda: srv.design,
            srv._confine("build/design"),
            lock=srv.design_lock,
            save_to=srv._design_save_to,
            max_wall_s=srv.max_wall_s or 600.0,
        )
        backend.auto_thumbs = True  # missing library thumbnails are rendered in the background (build/thumbs)
        srv.sim.show(design=backend)
