"""G0a profiler hook (docs/gpu-physics-g0.md): put this directory on PYTHONPATH and every python process of a
`napl program` run times the napl.Sim API. Buckets: step (Solver step: MuJoCo + C++ devices), render, marshal
(query/set/call/properties, i.e. get/set through the bus), load. Outermost call only, so nested calls (set -> call) count once.
Env: G0A_OUT=<json path> (written at exit by processes that stepped), G0A_CPROF=<pstats path> (also run cProfile)."""

import atexit
import json
import os
import sys
import time

_T0 = time.perf_counter()
_OUT = os.environ.get("G0A_OUT")


def _install():
    import napl

    acc = dict(step=0.0, render=0.0, marshal=0.0, load=0.0)
    cnt = dict(step=0, render=0, marshal=0, load=0)
    st = dict(depth=0, first=None, last=None, ticks=0)
    perf = time.perf_counter

    def wrap(fn, bucket, is_step=False):
        def w(*a, **k):
            if st["depth"]:
                return fn(*a, **k)
            st["depth"] = 1
            t = perf()
            if st["first"] is None:
                st["first"] = t
            try:
                return fn(*a, **k)
            finally:
                t1 = perf()
                st["depth"] = 0
                st["last"] = t1
                acc[bucket] += t1 - t
                cnt[bucket] += 1
                if is_step and a[1:]:
                    st["ticks"] += a[1]
                elif is_step:
                    st["ticks"] += 1

        return w

    for name, v in list(vars(napl.Sim).items()):
        if name.startswith("_") or name in ("show", "close_viewer"):
            continue
        b = "step" if name == "step" else "render" if "render" in name else "marshal"
        if isinstance(v, property):
            setattr(napl.Sim, name, property(wrap(v.fget, b)))
        elif isinstance(v, classmethod):
            f = v.__func__
            setattr(napl.Sim, name, classmethod(wrap(f, "load")))
        elif callable(v):
            setattr(napl.Sim, name, wrap(v, b, name == "step"))

    prof = None
    if os.environ.get("G0A_CPROF"):
        import cProfile

        prof = cProfile.Profile()
        prof.enable()

    def dump():
        if not cnt["step"]:
            return
        end = perf()
        if prof:
            prof.disable()
            prof.dump_stats(os.environ["G0A_CPROF"])
        d = dict(
            argv=sys.argv, proc_wall_s=end - _T0, loop_s=(st["last"] - st["first"]),
            pre_loop_s=st["first"] - _T0, post_loop_s=end - st["last"], ticks=st["ticks"],
            **{k + "_s": v for k, v in acc.items()}, **{k + "_calls": v for k, v in cnt.items()},
        )  # fmt: skip
        with open(_OUT, "w") as f:
            json.dump(d, f)

    atexit.register(dump)


if _OUT:
    try:
        _install()
    except ImportError:
        pass
