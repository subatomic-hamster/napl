"""The dog's locomotion software (SP4); imports nothing of the simulator: loco/hw.py is the only module that touches the robot."""

import ctypes
import os

# The MPC / WBC matrices are tiny (144 x 144): a multi-threaded BLAS spends its time in thread hand-offs (366 ms median per MPC solve with
# the BLAS default on a loaded 16-core box, 6 ms with one thread). The controller must not depend on the caller's environment, so pin to one
# thread here: the environment for libraries not yet loaded, and the loaded OpenBLAS (numpy / scipy wheels) through its C entry point.
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy  # noqa: F401  (loads the wheel's OpenBLAS so that it can be pinned below)
import scipy.linalg  # noqa: F401


def pin_blas_threads(n=1):
    """Set the thread count of every OpenBLAS mapped into this process; returns how many libraries were set."""
    done = 0
    try:
        with open("/proc/self/maps") as f:
            libs = {
                ln.split()[-1]
                for ln in f
                if "openblas" in ln.lower() and ln.split()[-1].startswith("/")
            }
    except OSError:
        return 0
    for path in libs:
        try:
            lib = ctypes.CDLL(path)
        except OSError:
            continue
        for sym in (
            "openblas_set_num_threads",
            "scipy_openblas_set_num_threads",
            "openblas_set_num_threads64_",
            "scipy_openblas_set_num_threads64_",
        ):
            fn = getattr(lib, sym, None)
            if fn is not None:
                fn(ctypes.c_int(n))
                done += 1
                break
    return done


pin_blas_threads(1)
