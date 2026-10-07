"""MCP server (`napl mcp`): the command bus as tools, for agents.

Tool input schemas are the flatc JSON Schema of the command tables (`_napl.schema_json`); annotations come from each
table's `cap` attribute (`_napl.command_cap`). The session is read+control: destructive commands are refused by the
C++ bus unless one call is elevated after approval (`--allow-destructive` or MCP elicitation). Every path argument is
resolved (symlinks, `..`) and must stay under the root. Long operations (run_test, sweep) run on a background thread
over a fork() of the loaded sim and are polled with get_run (no MCP Tasks: the v2 SDK dropped them).

Not exposed: Run / Pause (they need the C++ Host thread, which Python does not bind; on a bare World they are
no-ops, and a tool that silently does nothing would mislead an agent).
"""

import json
import re
import shutil
import threading
from pathlib import Path
from typing import Annotated

from mcp.server import MCPServer
from mcp.server.elicitation import ElicitationResult
from mcp.server.mcpserver.context import Context
from mcp.server.mcpserver.exceptions import ToolError
from mcp.server.mcpserver.resolve import Elicit, Resolve
from mcp.server.mcpserver.tools import Tool
from mcp.server.mcpserver.utilities.func_metadata import ArgModelBase, FuncMetadata
from mcp_types import ToolAnnotations
from pydantic import BaseModel, ConfigDict, Field

from . import Sim, _napl
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
}
RUN_ID = {
    "type": "object",
    "properties": {"run_id": {"type": "string"}},
    "required": ["run_id"],
    "additionalProperties": False,
}
NO_ARGS = {"type": "object", "properties": {}, "additionalProperties": False}


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
    ):
        self.max_steps_per_call = max_steps_per_call
        self.max_wall_s = max_wall_s
        self.root = Path(root or Path.cwd()).resolve()
        self.allow_destructive = allow_destructive
        self.sim = None  # napl.Sim with SESSION_CAPS (one world; calls run on the event loop, so they are serialised)
        self.model = None
        self.jobs = {}  # run_id -> {"status", "result" | "error"}
        tools = []
        for name, (table, desc) in COMMAND_TOOLS.items():
            if name == "load_model":
                tools.append(self._load_model_tool(desc))
                continue
            fn = {"run_test": self._run_test, "sweep": self._sweep}.get(name)
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
        ]
        self.server = MCPServer("napl", tools=tools)

    @staticmethod
    def _annotations(name, cap) -> ToolAnnotations:
        return ToolAnnotations(
            title=name,
            read_only_hint=cap == "read",
            destructive_hint=cap == "destructive",
        )

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

    async def _ask_delete(self, run_id: str):
        self._run_dir(run_id)
        return self._ask(f"delete run '{run_id}' (its runs/ directory)")

    @staticmethod
    def _require(approval) -> None:
        if approval.action != "accept" or not approval.data.confirm:
            raise ToolError("refused, nothing changed: not approved")

    def _load_model_tool(self, desc: str) -> Tool:
        async def load_model(
            ctx: Context,
            path: str,
            approval: Annotated[ElicitationResult[_Confirm], Resolve(self._ask_load)],
        ):
            self._require(approval)
            return self._load(path)

        t = Tool.from_function(
            load_model,
            name="load_model",
            description=desc,
            annotations=self._annotations("load_model", _napl.command_cap("LoadModel")),
        )
        t.parameters = _schema(
            "LoadModel"
        )  # the flatc schema, not the stub's signature
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
                )
            else:  # the C++ bus checks the path against the session root and the (elevated) capability
                self._call("LoadModel", {"path": str(p)}, elevate=True)
        except RuntimeError as e:
            raise ToolError(str(e)) from None
        self.model = p
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
) -> None:
    s = NaplServer(root, allow_destructive, max_steps_per_call, max_wall_s).server
    if http:
        s.run(transport="streamable-http", host=host, port=port)
    else:
        s.run()
