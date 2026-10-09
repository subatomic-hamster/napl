#!/usr/bin/env python3
"""List every electrical part in parts/ that still has no pin-level connectors (parts/README.md "Connectors and pins").

Electrical = the part declares at least one power or signal port. The list is the work queue for filling in pinouts from the
vendor datasheets; `--done` lists the parts that have them.

Usage: python3 tools/pinout_todo.py [parts_dir] [--done]
"""

import collections
import pathlib
import sys
import tomllib


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    root = pathlib.Path(
        args[0] if args else pathlib.Path(__file__).resolve().parents[1] / "parts"
    )
    done_mode = "--done" in sys.argv
    todo, done = [], []
    for f in sorted(root.glob("*/*.toml")):
        doc = tomllib.loads(f.read_text(encoding="utf-8"))
        ports = [
            p for p in doc.get("ports", []) if p.get("kind") in ("power", "signal")
        ]
        if not ports or all(p.get("type") == "wire-lead" for p in ports):
            continue  # no electrical interface, or a bare conductor (hook-up wire: nothing to pin out)
        (done if doc.get("connectors") else todo).append(
            (doc.get("category", ""), doc["id"], len(ports))
        )
    show = done if done_mode else todo
    by_cat = collections.defaultdict(list)
    for cat, pid, n in show:
        by_cat[cat.split(".")[0]].append((cat, pid, n))
    for top in sorted(by_cat):
        print(f"[{top}] {len(by_cat[top])}")
        for cat, pid, n in by_cat[top]:
            print(f"  {pid:56s} {cat:28s} {n} port(s)")
    print(
        f"{len(todo)} electrical parts without connectors, {len(done)} with ({len(todo) + len(done)} electrical in total)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
