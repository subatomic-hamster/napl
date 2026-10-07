"""M11: napl.gym.SimEnv: gymnasium's env checker, seeded determinism, bounds from the model's ctrlrange."""

import warnings
from pathlib import Path

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env
from napl.gym import SimEnv

PENDULUM = Path(__file__).resolve().parents[2] / "tests" / "data" / "pendulum.xml"
HINGE = "/world/pole/hinge"


def pendulum_env(**kw) -> SimEnv:
    cfg = dict(
        obs=[(HINGE, "qpos"), (HINGE, "qvel")],
        actions=["/actuators/torque"],
        frame_skip=5,
        max_steps=50,
        reset_noise=[(HINGE, "qpos", 0.1), (HINGE, "qvel", 0.05)],
        reward_obs=0,
        target=0.5,
    )
    return SimEnv(PENDULUM, **{**cfg, **kw})


def rollout(env: SimEnv, seed: int, action_seed: int = 99, steps: int = 60):
    """Episode under a seeded action sequence; every observable, exact."""
    rng = np.random.default_rng(action_seed)
    obs, info = env.reset(seed=seed)
    trace = [(obs.copy(), 0.0, False, False, info["state_hash"])]
    for _ in range(steps):
        obs, r, term, trunc, info = env.step(rng.uniform(-5, 5, size=1))
        trace.append((obs.copy(), r, term, trunc, info["state_hash"]))
        if term or trunc:
            break
    return trace


def same(a, b) -> bool:
    return len(a) == len(b) and all(
        np.array_equal(x[0], y[0]) and x[1:] == y[1:] for x, y in zip(a, b)
    )


def test_check_env_passes():
    env = pendulum_env()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        check_env(env)
    msgs = [str(w.message) for w in caught]
    print("check_env warnings:", msgs)
    # Advisory only, by design: actions are in raw ctrl units (model ctrlrange), observations are physical and
    # unbounded, and the env is not built through gymnasium.make (no spec). Anything else is a real finding.
    advisory = (
        "symmetric and normalized",
        "minimum value is -infinity",
        "maximum value is infinity",
        "not having a spec",
    )
    assert [m for m in msgs if not any(a in m for a in advisory)] == []


def test_bounds_from_ctrlrange_and_unlimited_needs_explicit_bounds():
    env = pendulum_env()
    assert env.action_space.low.tolist() == [
        -5.0
    ] and env.action_space.high.tolist() == [5.0]
    assert env.observation_space.shape == (2,)
    with pytest.raises(ValueError, match="no ctrlrange"):
        pendulum_env(
            actions=["/actuators/servo"]
        )  # position servo: no ctrlrange in the model
    env = pendulum_env(
        actions=["/actuators/servo"], action_bounds={"/actuators/servo": (-1, 1)}
    )
    assert env.action_space.high.tolist() == [1.0]


def test_same_seed_rollouts_identical_different_seeds_differ():
    a, b = rollout(pendulum_env(), 7), rollout(pendulum_env(), 7)  # two separate envs
    assert len(a) == 51 and a[-1][3]  # truncated at max_steps
    assert same(a, b)
    env = pendulum_env()  # one env, back to back (reset wipes the previous episode)
    assert same(rollout(env, 7), a) and same(rollout(env, 7), a)
    c = rollout(pendulum_env(), 8)
    assert not np.array_equal(a[0][0], c[0][0])  # initial noise differs
    assert a[-1][4] != c[-1][4]  # final state_hash differs
    assert (
        a[-1][4] != rollout(pendulum_env(), 7, action_seed=100)[-1][4]
    )  # actions matter too


def test_reset_determinism():
    env = pendulum_env()
    o1, i1 = env.reset(seed=3)
    for _ in range(10):
        env.step(env.action_space.sample())
    o2, i2 = env.reset(seed=3)  # after a dirty episode: identical to the first reset
    assert np.array_equal(o1, o2) and i1 == i2 and i1["tick"] == 0
    o3, i3 = env.reset(seed=4)
    assert not np.array_equal(o1, o3) and i1["state_hash"] != i3["state_hash"]


def test_default_reward_and_truncation():
    env = pendulum_env(reset_noise=[], max_steps=3)
    env.reset(seed=0)
    obs, r, term, trunc, _ = env.step(np.array([0.0]))
    assert r == -((obs[0] - 0.5) ** 2) and not term and not trunc
    env.step(np.array([0.0]))
    assert env.step(np.array([0.0]))[3]
