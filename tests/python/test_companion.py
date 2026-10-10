"""napl.companion: the MAVLink messages a companion computer sends PX4 (ODOMETRY for EKF2 external vision, OBSTACLE_DISTANCE for collision prevention),
checked as field dicts, without PX4 or MAVSDK. The frame conversion is the one of src/px4/frames.h (tests/cpp/test_px4.cpp has the same cases)."""

import json

import numpy as np
from napl import companion as C


def rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])  # fmt: skip


def test_attitude_convention_matches_the_bridge():
    s = np.sqrt(0.5)
    assert np.allclose(
        C.q_ned_frd([1, 0, 0, 0]), [s, 0, 0, s]
    )  # identity napl attitude: nose East = yaw +90 deg in NED
    assert np.allclose(
        C.q_ned_frd([s, 0, 0, s]), [1, 0, 0, 0]
    )  # napl yaw +90 deg (nose North) -> identity


def test_rotation_commutes_with_the_frame_change():
    # R_ned_frd v_frd == enu_to_ned(R_enu_flu v_flu): several attitudes and vectors (the C++ test's)
    for raw in (
        [0.8, 0.3, -0.2, 0.45],
        [0.1, 0.9, 0.2, -0.3],
        [-0.5, 0.2, 0.7, 0.4],
        [0.3, -0.1, 0.05, -0.9],
    ):
        q = np.array(raw) / np.linalg.norm(raw)
        for v in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [0.3, -1.2, 2.5]):
            v = np.array(v, float)
            assert np.allclose(
                rot(C.q_ned_frd(q)) @ (v * [1, -1, -1]),
                C.enu_to_ned(rot(q) @ v),
                atol=1e-12,
            )


def test_odometry_fields():
    f = C.odometry_fields(
        12.5,
        [1.0, 2.0, 3.0],
        [1, 0, 0, 0],
        [0.5, 1.5, -2.5],
        w_flu=(0.1, 0.2, 0.3),
        sigma=(0.2, 0.1, 0.03, 0.05),
        reset_counter=2,
    )
    assert (f["x"], f["y"], f["z"]) == (2.0, 1.0, -3.0)  # E1 N2 U3 -> N2 E1 D-3
    assert (f["vx"], f["vy"], f["vz"]) == (1.5, 0.5, 2.5)
    assert np.allclose(f["q"], [np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
    assert (f["rollspeed"], f["pitchspeed"], f["yawspeed"]) == (
        0.1,
        -0.2,
        -0.3,
    )  # FLU -> FRD
    assert (
        f["time_usec"] == 12_500_000 and f["frame_id"] == 1 and f["child_frame_id"] == 1
    )  # MAV_FRAME_LOCAL_NED
    assert (
        f["estimator_type"] == 3 and f["reset_counter"] == 2
    )  # MAV_ESTIMATOR_TYPE_VIO
    pc = f["pose_covariance"]
    assert len(pc) == 21 and len(f["velocity_covariance"]) == 21
    # the packed upper triangle's diagonal: x y z roll pitch yaw
    assert np.allclose(
        [pc[i] for i in (0, 6, 11, 15, 18, 20)],
        [0.04, 0.04, 0.04, 0.03**2, 0.03**2, 0.05**2],
    )
    assert sum(1 for x in pc if x) == 6
    assert np.allclose([f["velocity_covariance"][i] for i in (0, 6, 11)], 0.01)
    json.dumps(f)  # MavlinkDirect takes the field dict as JSON


def test_obstacle_distance_bins():
    pts = [
        [2.0, 0.0, 0.0],  # 2 m dead ahead -> bin 0
        [
            0.0,
            -3.0,
            0.1,
        ],  # 3 m to the right (FLU y < 0) -> bearing 90 deg clockwise -> bin 18
        [0.0, 4.0, -0.2],  # 4 m to the left -> bearing 270 deg -> bin 54
        [1.0, 0.0, 0.0],  # nearer, same bin as the first
        [
            5.0,
            0.0,
            2.0,
        ],  # above the height band: its bin is seen but this point does not count
        [-5.0, 0.0, 0.0],  # behind (bin 36), seen
        [0.1, 0.0, 0.0],  # inside the sensor's minimum range: ignored
        [np.nan, 0.0, 0.0],
    ]
    f = C.obstacle_distance_fields(3.0, pts, min_m=0.28, max_m=10.0)
    d = f["distances"]
    assert (
        len(d) == 72
        and f["frame"] == 12
        and f["increment"] == 5
        and f["increment_f"] == 5.0
        and f["angle_offset"] == 0.0
    )
    assert d[0] == 100 and d[18] == 300 and d[54] == 400  # cm
    assert d[36] == 500
    assert (
        f["min_distance"] == 28
        and f["max_distance"] == 1000
        and f["time_usec"] == 3_000_000
    )
    assert (
        d[1] == C.UNKNOWN and d[40] == C.UNKNOWN
    )  # bins no point falls in: not "clear", unknown
    f = C.obstacle_distance_fields(
        0.0, [[5.0, 0.0, 2.0]]
    )  # a bin with a point only outside the band: seen, nothing in range = max_distance + 1
    assert f["distances"][0] == 1001 and f["distances"][1] == C.UNKNOWN
    json.dumps(f)


def test_stack_offboard_port_follows_rcs_routing():
    # px4-rc.mavlink: offboard remote port 14540 + n, but 14549 for every instance above 9 (a MAVSDK program must listen where PX4 sends)
    from napl import px4

    for n, port in ((0, 14540), (5, 14545), (9, 14549), (10, 14549), (12, 14549)):
        st = px4.Stack("cfg.toml", "/nonexistent", instance=n)
        assert st.mavlink == f"udp://:{port}" and port in st.udp
