"""Process-local native execution policy and bounded batch admission.

Apple defaults to background; Linux retains maximum throughput. Configuration
changes apply at complete solver steps. Children inherit launch-time policy, not
later parent changes. Duty governs solver steps, not arbitrary Python code.
"""
import concurrent.futures
import contextvars
import os
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile

from . import _napl

_batch_profile = contextvars.ContextVar("napl_batch_profile", default=None)


def configure(profile: str) -> None:
    """Request background, balanced or maximum; safety downshifts still apply."""
    _napl.runtime_configure(profile)
    os.environ["NAPL_PROFILE"] = profile


def status(gb_per_worker: float = 1.0) -> dict:
    """Requested/effective policy, admission budget and actual OS telemetry."""
    return _napl.runtime_status(gb_per_worker)


class ResourcePressureError(RuntimeError):
    """A batch could not complete; no truncated successful aggregate is returned."""


def worker_limit(requested=None, gb_per_worker=1.0):
    cap = status(gb_per_worker)["workers"]
    if _batch_profile.get() is not None:
        cap = min(cap, 1)
    if requested is not None:
        if isinstance(requested, bool) or not isinstance(requested, int) or requested < 1:
            raise ValueError("workers must be a positive integer")
        cap = min(cap, requested)
    if cap == 0:
        raise ResourcePressureError("worker admission rejected: " + status(gb_per_worker)["reason"])
    return cap


def child_environment(extra=None, *, batch=False, profile=None):
    """An explicit child environment; never mutates the parent environment."""
    env = {**os.environ, **(extra or {})}
    inherited_profile = _batch_profile.get()
    env["NAPL_PROFILE"] = profile or inherited_profile or status()["requested_profile"]
    if batch or inherited_profile is not None:
        env["NAPL_WORKER_CEILING"] = "1"
        env["NAPL_SIMULATION_WORKER"] = "1"  # dedicated QoS on import; napl.program clears it for controller programs
        for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
            env.setdefault(name, "1")
    return env


def _thread_job(fn, job, profile):
    token = _batch_profile.set(profile)
    try:
        return fn(job)
    finally:
        _batch_profile.reset(token)


def _process_job(fn, job, env):
    # Explicit Popen env is essential: multiprocessing's spawn inherits the parent
    # environment, which cannot safely be changed during concurrent launches.
    module = fn.__module__
    if "<locals>" in fn.__qualname__:
        raise TypeError("process batch jobs must be module-level functions")
    # Like multiprocessing spawn: always recover the caller's __main__ so jobs may carry
    # main-defined classes even when the target function itself is imported.
    main = sys.modules["__main__"]
    main_path = getattr(main, "__file__", None)
    sys.modules.setdefault("__mp_main__", main)
    with tempfile.TemporaryDirectory(prefix="napl-batch-") as temp:
        result_path = Path(temp) / "result.pickle"
        header = (module, main_path, fn.__qualname__, str(result_path), sys.path)
        payload = pickle.dumps(header) + pickle.dumps(job)
        run = subprocess.run([sys.executable, "-m", "napl._batch_worker"], input=payload, env=env)
        if run.returncode or not result_path.is_file():
            raise RuntimeError(f"batch child failed with exit {run.returncode}")
        ok, result = pickle.loads(result_path.read_bytes())
        if not ok:
            raise result
        return result


def _bounded_results(fn, jobs, workers=None, *, gb_per_worker=1.0, threads=False):
    launch_cap = worker_limit(workers, gb_per_worker)
    profile = _batch_profile.get() or status()["requested_profile"]
    env = child_environment(batch=True, profile=profile)
    iterator = iter(jobs)
    active = {}
    exhausted = False
    index = 0
    # Threads supervise child processes (or existing subprocess-based drivers).
    # No eager process-pool queue; at most the current admission ceiling is live.
    with concurrent.futures.ThreadPoolExecutor(max_workers=launch_cap) as pool:
        while active or not exhausted:
            cap = min(launch_cap, worker_limit(gb_per_worker=gb_per_worker))
            while len(active) < cap and not exhausted:
                cap = min(launch_cap, worker_limit(gb_per_worker=gb_per_worker))
                if len(active) >= cap:
                    break
                try:
                    job = next(iterator)
                except StopIteration:
                    exhausted = True
                    break
                future = pool.submit(_thread_job, fn, job, profile) if threads else pool.submit(_process_job, fn, job, env)
                active[future] = index
                index += 1
            if active:
                done, _ = concurrent.futures.wait(active, timeout=.1,
                    return_when=concurrent.futures.FIRST_COMPLETED)
                for future in done:
                    yield active.pop(future), future.result()


def bounded_map(fn, jobs, workers=None, *, gb_per_worker=1.0, threads=False):
    """Return input-ordered results, or raise after active jobs finish on rejection.

    Each process job has a fresh interpreter and child-specific environment. This
    prevents nested oversubscription without global spawn/environment mutations;
    it does not promise persistent worker import caches.
    """
    results = {}
    for index, result in _bounded_results(fn, jobs, workers, gb_per_worker=gb_per_worker, threads=threads):
        results[index] = result
    return [results[index] for index in range(len(results))]


class BatchPool:
    """Bounded scheduler retaining an initial ceiling across multiple batch maps."""
    def __init__(self, workers=None, *, threads=False, gb_per_worker=1.0):
        self.workers = worker_limit(workers, gb_per_worker)
        self.threads = threads
        self.gb_per_worker = gb_per_worker

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def map(self, fn, jobs, chunksize=1):
        return bounded_map(fn, jobs, self.workers, gb_per_worker=self.gb_per_worker, threads=self.threads)

    def imap_unordered(self, fn, jobs, chunksize=1):
        for _, result in _bounded_results(fn, jobs, self.workers,
                                         gb_per_worker=self.gb_per_worker, threads=self.threads):
            yield result
