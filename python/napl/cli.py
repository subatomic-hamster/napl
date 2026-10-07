"""`napl test <spec.toml|spec.json> [--model PATH] [--runs-dir DIR] [--set path.attr=v1,v2 ...]` and
`napl mcp [--http] [--host H] [--port N] [--root DIR] [--allow-destructive] [--max-steps N] [--max-wall-s S]` (see mcp_server.py).

Exit code 0 = all monitors pass, 1 = a monitor failed (or the solver diverged), 2 = error (bad spec, model, args).
"""

import argparse
import json
import sys
import tomllib
from pathlib import Path

from . import Sim
from .report import write_report


class SpecError(Exception):
    pass


def _parse_set(s: str) -> dict:
    lhs, eq, rhs = s.partition("=")
    path, dot, attr = lhs.rpartition(".")
    if not (eq and dot and path and attr and rhs):
        raise SpecError(f"bad --set '{s}': expected path.attr=value[,value...]")
    try:
        return {
            "path": path,
            "attr": attr,
            "values": [float(v) for v in rhs.split(",")],
        }
    except ValueError:
        raise SpecError(f"bad --set '{s}': values must be numbers") from None


def _load_spec(path: Path) -> dict:
    try:
        with open(path, "rb") as f:
            return json.load(f) if path.suffix == ".json" else tomllib.load(f)
    except (
        OSError,
        ValueError,
    ) as e:  # tomllib.TOMLDecodeError and json.JSONDecodeError are ValueErrors
        raise SpecError(f"cannot read spec '{path}': {e}") from None


def run(args) -> int:
    spec_path = Path(args.spec)
    spec = _load_spec(spec_path)
    model = spec.pop("model", None)  # CLI-level key, not part of TestSpec
    if args.model:
        model = args.model
    elif model:
        model = spec_path.parent / model
    if not model:
        raise SpecError("no model: give --model or a top-level `model` key in the spec")
    spec["settings"] = spec.get("settings", []) + [
        _parse_set(s) for s in args.set
    ]  # --set applies last
    try:
        sim = Sim.load(model)
        result = sim.run_test(
            spec
        )  # unknown fields / bad ops come back as the parser's message
    except (RuntimeError, ValueError) as e:
        raise SpecError(str(e)) from None
    run_dir = write_report(result, spec, model, args.runs_dir)
    print(
        f"{'PASS' if result['pass'] else 'FAIL'} {spec.get('name', '')}  ({result['end_tick'] - result['start_tick']} ticks, dt={result['dt']})"
    )
    for m in result["monitors"]:
        line = f"  {'ok  ' if m['pass'] else 'FAIL'} {m['name']}: range [{m['min_value']:.4g}, {m['max_value']:.4g}] rms {m['rms']:.4g} final {m['final_value']:.4g}"
        if m["hit"]:
            line += f"; at tick {m['hit_tick']} (t={m['hit_time']:.4g} s) value {m['hit_value']:.4g}"
        print(line)
    if result["diverged"]:
        print("  solver diverged")
    if result["fail_monitor"]:
        print(
            f"  first violation: {result['fail_monitor']} at tick {result['fail_tick']} (t={result['fail_time']:.4g} s)"
        )
    print(f"run dir: {run_dir}")
    return 0 if result["pass"] else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="napl")
    sub = p.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("test", help="run a declarative test spec")
    t.add_argument("spec")
    t.add_argument("--model", help="MJCF/URDF path (overrides the spec's `model` key)")
    t.add_argument("--runs-dir", default="runs")
    t.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="PATH.ATTR=V[,V]",
        help="extra setting, applied after the spec's",
    )
    m = sub.add_parser("mcp", help="MCP server (stdio by default)")
    m.add_argument(
        "--http", action="store_true", help="Streamable HTTP instead of stdio"
    )
    m.add_argument("--host", default="127.0.0.1")
    m.add_argument("--port", type=int, default=8765)
    m.add_argument(
        "--root", default=".", help="directory every path argument must stay under"
    )
    m.add_argument(
        "--allow-destructive",
        action="store_true",
        help="skip the approval prompt for destructive tools (load_model, delete_run)",
    )
    m.add_argument(
        "--max-steps",
        type=int,
        default=100000,
        help="session limit: ticks per step_simulation / run_test (default %(default)s)",
    )
    m.add_argument(
        "--max-wall-s",
        type=float,
        default=60.0,
        help="session limit: wall seconds per run_test / sweep, 0 = none (default %(default)s)",
    )
    try:
        args = p.parse_args(argv)  # argparse itself exits 2 on bad usage
        if args.cmd == "mcp":
            from .mcp_server import serve

            serve(
                args.root,
                args.http,
                args.host,
                args.port,
                args.allow_destructive,
                args.max_steps,
                args.max_wall_s,
            )
            return 0
        return run(args)
    except SpecError as e:
        print(f"napl: error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
