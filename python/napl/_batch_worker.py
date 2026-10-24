"""Private subprocess entry: trusted parent pickle, never an external RPC interface."""
import importlib
import pickle
import runpy
import sys
from pathlib import Path
import types


def main():
    module, main_path, name, result_path, paths = pickle.load(sys.stdin.buffer)
    sys.path[:] = paths
    try:
        if main_path:
            sys.path.insert(0, str(Path(main_path).resolve().parent))
            namespace = runpy.run_path(main_path, run_name="__mp_main__")
            loaded = types.ModuleType("__mp_main__")
            loaded.__dict__.update(namespace)
            sys.modules["__mp_main__"] = loaded
            sys.modules["__main__"] = loaded
        if module in ("__main__", "__mp_main__"):
            if not main_path:
                raise TypeError("process batch functions require an importable source file")
            value = sys.modules["__mp_main__"]
        else:
            value = importlib.import_module(module)
        for part in name.split("."):
            value = getattr(value, part)
        job = pickle.load(sys.stdin.buffer)
        result = (True, value(job))
    except Exception as error:
        result = (False, error)
    try:
        data = pickle.dumps(result)
    except Exception as error:
        data = pickle.dumps((False, RuntimeError(f"batch result could not be serialized: {error}")))
    Path(result_path).write_bytes(data)


if __name__ == "__main__":
    main()
