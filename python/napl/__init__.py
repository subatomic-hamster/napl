"""napl: thin Python front end over the C++ kernel (`_napl`). No simulation logic lives here."""

import json
from os import fspath

import numpy as np

from . import _napl

__all__ = ["Sim"]


class Sim:
    """One World. Not thread-safe per instance; separate Sims run in parallel (the GIL is released)."""

    def __init__(self, core: "_napl.Sim"):
        self._c = core

    @classmethod
    def load(
        cls,
        path,
        root=None,
        max_sweep_runs: int = 100000,
        max_wall_s: float = 0.0,
        caps=("read", "control", "destructive"),
        max_steps_per_call: int = 2**31 - 1,
    ) -> "Sim":
        """Load MJCF/URDF. `root` confines later LoadModel commands (default: the model's directory).
        Session limits: `max_sweep_runs` (larger sweeps -> limit_exceeded), `max_wall_s` (0 = none; a RunTest or
        sweep running longer fails with limit_exceeded and changes nothing). `caps` is the session's capability
        set, enforced by the C++ bus (a command whose `cap` is missing gets permission_denied); forks inherit it.
        `max_steps_per_call`: a Step with larger n, or a RunTest/Sweep longer than this, gets limit_exceeded."""
        return cls(
            _napl.Sim(
                fspath(path),
                "" if root is None else fspath(root),
                max_sweep_runs,
                max_wall_s,
                list(caps),
                max_steps_per_call,
            )
        )

    def step(self, n: int = 1) -> int:
        """Advance n steps through the command bus (logged); returns the tick."""
        return self._c.step(n)

    @property
    def tick(self) -> int:
        return self._c.tick

    @property
    def diverged(self) -> bool:
        return self._c.diverged

    @property
    def caps(self) -> list:
        return self._c.caps

    def call(self, cmd, elevate: bool = False) -> dict:
        """Run one napl.Envelope given as dict or JSON text; returns the napl.Reply as a dict (never raises
        for command-level failures: check reply["ok"]). `elevate=True` runs this one call with all capabilities
        (the caller has obtained approval); the session itself is unchanged."""
        return json.loads(
            self._c.call(cmd if isinstance(cmd, str) else json.dumps(cmd), elevate)
        )

    def query(self, path: str, attr: str, frame: str = ""):
        """-> (float64 ndarray, unit string). Raises KeyError / ValueError for unknown / invalid requests."""
        return self._c.get(path, attr, frame)

    def set(self, path: str, attr: str, values, frame: str = "") -> dict:
        """SetAttr through the bus (logged). Raises ValueError if the reply is not ok."""
        vals = np.atleast_1d(np.asarray(values, dtype=np.float64)).tolist()
        r = self.call(
            {
                "cmd_type": "SetAttr",
                "cmd": {"path": path, "attr": attr, "frame": frame, "values": vals},
            }
        )
        if not r["ok"]:
            raise ValueError(r["message"])
        return r

    def run_test(self, spec: dict) -> dict:
        """RunTest (a napl.TestSpec as a dict) through the bus; returns the napl.TestResult dict with
        `fail_snapshot` as bytes (feed to restore()). Steps this Sim. Raises ValueError if the reply is not ok."""
        r = self.call({"cmd_type": "RunTest", "cmd": {"spec": spec}})
        if not r["ok"]:
            raise ValueError(r["message"])
        res = r["result"]
        res["fail_snapshot"] = bytes(res.get("fail_snapshot", []))
        return res

    def sweep(
        self,
        spec: dict,
        params: list,
        mode: str = "grid",
        n: int = 0,
        seed: int = 0,
        threads: int = 1,
        base: int = 0,
    ) -> list:
        """Sweep command: run `spec` (a TestSpec dict) once per parameter point on clones of the current state (`base` =
        a bus snapshot id instead, 0 = now). params: [{"path","attr","values":[...]}] (mode "grid": cartesian product,
        first param slowest) or [{"path","attr","lo","hi"}] (mode "random": `n` runs, run i seeded by seed+i). Returns one
        dict per run, ordered by index: index, params (dict "path.attr" -> value), pass, diverged, error, monitors.
        This Sim is not modified. The GIL is released while running, so it can be called from a background thread
        (do not use this same Sim from another thread meanwhile; fork() it first). `threads` is capped at 6.
        Raises ValueError (message starts with the error kind, e.g. "limit_exceeded") if the sweep is refused."""
        r = self.call(
            {
                "cmd_type": "Sweep",
                "cmd": {
                    "base": base,
                    "spec": spec,
                    "params": params,
                    "mode": mode,
                    "n": n,
                    "seed": seed,
                    "threads": threads,
                },
            }
        )
        if not r["ok"]:
            raise ValueError(f"{r['error']}: {r['message']}")
        names = r["sweep"].get("params", [])
        return [
            {
                "index": row.get("index", 0),
                "params": dict(zip(names, row.get("params", []))),
                "pass": row.get("pass", False),
                "diverged": row.get("diverged", False),
                "error": row.get("error", ""),
                "monitors": row.get("monitors", []),
            }
            for row in r["sweep"].get("rows", [])
        ]

    def fork(self) -> "Sim":
        """Independent copy of the current state (World::clone); no log, no snapshots."""
        return Sim(self._c.clone())

    def save(self) -> bytes:
        return self._c.save()

    def restore(self, state: bytes) -> None:
        self._c.restore(state)

    def state_hash(self) -> int:
        return self._c.state_hash()

    def start_log(self) -> None:
        """Start the replay log; call on a fresh Sim, before any snapshot."""
        self._c.start_log()

    def log(self):
        """Complete replay-log bytes, or None if logging is off."""
        return self._c.log()
