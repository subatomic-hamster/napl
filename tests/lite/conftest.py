import os


def pytest_addoption(parser):
    parser.addoption(
        "--fw-core",
        default=None,
        help="run the twin-rig tests (rig.py, apprig.py) against the firmware's C core: path to libcuas_lite_fwcore.so, or 'auto' for build/fw-host/",
    )


def pytest_configure(config):
    v = config.getoption("--fw-core")
    if v:
        os.environ["NAPL_FW_CORE"] = v
