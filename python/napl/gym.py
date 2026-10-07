"""Gymnasium env over a napl.Sim, configured declaratively (no code from configs is ever executed).

    env = SimEnv(
        "pendulum.xml",
        obs=[("/world/pole/hinge", "qpos"), ("/world/pole/hinge", "qvel")],   # (prim, attr) reads, concatenated
        actions=["/actuators/torque"],                                         # ctrl prims; bounds = model ctrlrange
        frame_skip=5, max_steps=100,
        reset_noise=[("/world/pole/hinge", "qpos", 0.1)],                      # (prim, attr, std): base + std * N(0, 1)
        reward_obs=0, target=0.5,                                              # reward = -(obs[0] - 0.5) ** 2
    )

Everything goes through the command bus (SetAttr / Step / restore), so a Sim started with `start_log()` replays an
episode. reset() restores a snapshot taken at construction, so it costs one state copy, and reset(seed=s) is fully
determined by s (initial-state noise comes from gymnasium's `np_random`). Custom rewards / terminations: subclass and
override `reward()` / `terminated()`.
"""

import gymnasium as gym
import numpy as np

from . import Sim

__all__ = ["SimEnv"]


class SimEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        model,
        obs,
        actions,
        *,
        action_bounds=None,
        frame_skip: int = 1,
        max_steps: int = 200,
        reset_noise=(),
        reward_obs: int = 0,
        target: float = 0.0,
    ):
        """model: MJCF/URDF path. obs: [(prim, attr)] reads. actions: [actuator prim] (one scalar ctrl each).
        action_bounds: {actuator prim: (lo, hi)}, overriding the model's ctrlrange (required where it is unlimited).
        frame_skip: ticks per step(). max_steps: steps per episode (then truncated). reset_noise: [(prim, attr, std)]
        added to that attribute's snapshot value at reset (every component, independent normals).
        reward_obs / target: the built-in reward is -(obs[reward_obs] - target) ** 2."""
        self._sim = Sim.load(model)
        self._obs = [(str(p), str(a)) for p, a in obs]
        self._acts = [str(a) for a in actions]
        self._noise = [(str(p), str(a), float(s)) for p, a, s in reset_noise]
        self._frame_skip, self._max_steps = int(frame_skip), int(max_steps)
        self._reward_obs, self._target = int(reward_obs), float(target)
        bounds = dict(action_bounds or {})
        lo, hi = [], []
        for act in self._acts:
            if act in bounds:
                lo_a, hi_a = bounds[act]
            else:
                lo_a, hi_a = self._sim.query(act, "ctrlrange")[0]
            if not (np.isfinite(lo_a) and np.isfinite(hi_a)):
                raise ValueError(
                    f"{act}: model has no ctrlrange; pass action_bounds={{'{act}': (lo, hi)}}"
                )
            lo.append(lo_a)
            hi.append(hi_a)
        self.action_space = gym.spaces.Box(np.array(lo), np.array(hi), dtype=np.float64)
        n = len(self._read_obs())
        if not 0 <= self._reward_obs < n:
            raise ValueError(
                f"reward_obs {self._reward_obs} outside the {n}-element observation"
            )
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (n,), dtype=np.float64)
        self._base = self._sim.save()
        self._t = 0

    def _read_obs(self) -> np.ndarray:
        return np.concatenate([self._sim.query(p, a)[0] for p, a in self._obs])

    def reward(self, obs: np.ndarray, action: np.ndarray) -> float:
        return -float((obs[self._reward_obs] - self._target) ** 2)

    def terminated(self, obs: np.ndarray) -> bool:
        return bool(self._sim.diverged)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._sim.restore(self._base)
        for prim, attr, std in self._noise:
            base = self._sim.query(prim, attr)[0]
            self._sim.set(
                prim, attr, base + std * self.np_random.standard_normal(base.shape)
            )
        self._t = 0
        return self._read_obs(), self._info()

    def step(self, action):
        a = np.clip(
            np.asarray(action, dtype=np.float64),
            self.action_space.low,
            self.action_space.high,
        )
        for prim, v in zip(self._acts, a):
            self._sim.set(prim, "ctrl", v)
        self._sim.step(self._frame_skip)
        self._t += 1
        obs = self._read_obs()
        return (
            obs,
            self.reward(obs, a),
            self.terminated(obs),
            self._t >= self._max_steps,
            self._info(),
        )

    def _info(self) -> dict:
        return {"tick": self._sim.tick, "state_hash": self._sim.state_hash()}
