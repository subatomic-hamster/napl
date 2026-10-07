"""`napl test <spec.toml|spec.json> [--model PATH] [--runs-dir DIR] [--set path.attr=v1,v2 ...]` and
`napl mcp [--http] [--host H] [--port N] [--root DIR] [--allow-destructive] [--max-steps N] [--max-wall-s S] [--view]`
(see mcp_server.py), and `napl design build|info|check <assembly.toml> [--parts DIR] [--out DIR] [--json]` (see
assemblies/README.md; `check` exits 1 if the design-rule check finds an error).

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


def _feetech_port(args) -> int:
    import time

    from .feetech import FeetechPort

    try:
        servos = {int(i): p for i, _, p in (s.partition("=") for s in args.servo)}
        sim = Sim.load(args.model, host=True)
        sim.call({"cmd_type": "Run", "cmd": {"rate": 1.0}})
        port = FeetechPort(sim, servos, baud=args.baud, symlink=args.symlink)
    except (RuntimeError, ValueError, OSError) as e:
        raise SpecError(str(e)) from None
    print(
        f"feetech port: {args.symlink or port.device_path}  (pty {port.device_path})",
        flush=True,
    )
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        port.close()
    return 0


def _parts_dir(arg) -> Path:
    if arg:
        return Path(arg)
    repo = (
        Path(__file__).resolve().parents[2] / "parts"
    )  # source / editable install: <repo>/python/napl/cli.py
    return repo if repo.is_dir() else Path("parts")


def design(args) -> int:
    from .design import DesignError, Library, build, check, compute, format_budget

    try:
        parts = _parts_dir(args.parts)
        if not parts.is_dir():
            raise SpecError(f"parts directory '{parts}' not found (use --parts)")
        if args.design_cmd == "info":
            _, report = compute(args.assembly, parts, ".")
            print(format_budget(report))
            return 0
        if args.design_cmd == "suggest":
            from .design import Design

            rows = Design.open(args.assembly, parts).suggest(args.target, args.limit)
            if args.json:
                print(json.dumps(rows, indent=2))
            for r in rows if not args.json else []:
                where = (
                    f" via port {r['port']}"
                    if "port" in r
                    else f" (keeps {r['mates_kept']} mates, loses {len(r['mates_lost'])}, mass {r['mass_delta']:+} kg)"
                )
                print(f"{r['part']:44s} {r.get('score', '')!s:>7}{where}")
                for w in (
                    r.get("why", []) + r.get("mates_lost", []) + r.get("dangling", [])
                ):
                    print(f"        {w}")
            return 0
        if args.design_cmd == "check":
            findings = check(args.assembly, Library(parts))
            if args.json:
                print(json.dumps([f.as_dict() for f in findings], indent=2))
            else:
                for f in findings:
                    print(
                        f"{f.severity.upper():7s} {f.code}  [{', '.join(f.where)}]  {f.message}"
                    )
                    if f.fix:
                        print(f"        fix: {f.fix}")
                n = {
                    s: sum(f.severity == s for f in findings)
                    for s in ("error", "warning", "info")
                }
                print(f"{n['error']} errors, {n['warning']} warnings, {n['info']} info")
            return 1 if any(f.severity == "error" for f in findings) else 0
        r = build(args.assembly, parts, args.out)
    except DesignError as e:
        raise SpecError(str(e)) from None
    rep = r["report"]
    fnd = rep["findings"]
    print(
        f"built {rep['assembly']}: {r['xml']} (+ report.json), {len(rep['instances'])} instances, "
        f"mass {rep['mass']['total']:.4f} kg, {len(rep['warnings'])} warnings; "
        f"DRC {sum(f['severity'] == 'error' for f in fnd)} errors, {sum(f['severity'] == 'warning' for f in fnd)} warnings "
        "(napl design check for the list)"
    )
    return 0


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
        "--view",
        action="store_true",
        help="open the Vulkan viewer (main thread) once a model is loaded, with the MCP loop on a background thread; "
        "needs a display and a Vulkan device; adds play_simulation / pause_simulation",
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
    f = sub.add_parser(
        "feetech-port",
        help="serve simulated servos on a pty speaking the Feetech STS protocol (real time)",
    )
    f.add_argument("model")
    f.add_argument(
        "--servo",
        action="append",
        default=[],
        required=True,
        metavar="ID=/devices/NAME",
    )
    f.add_argument("--baud", type=int, default=1_000_000)
    f.add_argument("--symlink", help="also create this stable symlink to the pty")
    d = sub.add_parser(
        "design", help="assemblies: build MJCF + report, mass budget (no kernel needed)"
    )
    dsub = d.add_subparsers(dest="design_cmd", required=True)
    for name, h in (
        ("build", "write <out>/<name>.xml and <name>.report.json"),
        ("info", "print the mass budget, write nothing"),
        ("check", "design-rule check (DRC/ERC); exit 1 if any error"),
        (
            "suggest",
            "ranked library parts for an open port ('inst.port') or swap candidates for an instance",
        ),
    ):
        s = dsub.add_parser(name, help=h)
        s.add_argument("assembly")
        if name == "suggest":
            s.add_argument("target", help="'inst.port' or 'inst'")
            s.add_argument("--limit", type=int, default=10)
            s.add_argument("--json", action="store_true")
        s.add_argument(
            "--parts",
            default=None,
            help="parts directory (default <repo>/parts or ./parts)",
        )
        if name == "check":
            s.add_argument(
                "--json", action="store_true", help="print the findings as JSON"
            )
        if name == "build":
            s.add_argument(
                "--out",
                default="build/assemblies",
                help="output directory (default %(default)s)",
            )
    from . import bench

    bench.add_parser(sub)
    try:
        args = p.parse_args(argv)  # argparse itself exits 2 on bad usage
        if args.cmd == "bench":
            return bench.main_bench(args)
        if args.cmd == "design":
            return design(args)
        if args.cmd == "feetech-port":
            return _feetech_port(args)
        if args.cmd == "mcp":
            from .mcp_server import serve

            try:
                serve(
                    args.root,
                    args.http,
                    args.host,
                    args.port,
                    args.allow_destructive,
                    args.max_steps,
                    args.max_wall_s,
                    args.view,
                )
            except (
                RuntimeError
            ) as e:  # --view: no display / Vulkan device / viewer in this build
                raise SpecError(str(e)) from None
            return 0
        return run(args)
    except SpecError as e:
        print(f"napl: error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
