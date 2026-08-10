#!/usr/bin/env python3
"""Run the healthy 22-output walking policy on the 28-muscle model.

The original policy continues to observe and control only the original 22 leg
muscles. The six lumbar muscles exist in MuJoCo but receive zero control, have
no policy outputs/weights, and are hidden from the legacy policy observation.
The lumbar joint is locked at neutral for this compatibility baseline.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = (
    REPO_ROOT
    / "rl_train/results/train_session_20251226-131706/session_config.json"
)
DEFAULT_CHECKPOINT = (
    REPO_ROOT
    / "rl_train/results/train_session_20251226-131706/trained_models/model_29892608"
)
INTEGRATED_MODEL = "models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml"
LEG_ACTUATOR_COUNT = 22
LUMBAR_ACTUATORS = (
    "ercspn_r",
    "ercspn_l",
    "intobl_r",
    "intobl_l",
    "extobl_r",
    "extobl_l",
)


def actuator_names(model) -> tuple[str, ...]:
    import mujoco

    return tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        for index in range(model.nu)
    )


def legacy_observation(
    observation: np.ndarray, qpos_count: int, qvel_count: int
) -> np.ndarray:
    """Remove the six appended lumbar activations from a 28-muscle observation."""
    lumbar_start = qpos_count + qvel_count + LEG_ACTUATOR_COUNT
    lumbar_end = lumbar_start + len(LUMBAR_ACTUATORS)
    if observation.size < lumbar_end:
        raise RuntimeError(
            f"Observation has {observation.size} values and does not contain "
            "the expected six appended lumbar activations"
        )
    return np.concatenate((observation[:lumbar_start], observation[lumbar_end:]))


def zero_lumbar_action(action: np.ndarray, model) -> np.ndarray:
    """Append six exact zeros to the legacy policy's 22 controls."""
    flat_action = np.asarray(action, dtype=np.float64).reshape(-1)
    if flat_action.size != LEG_ACTUATOR_COUNT:
        raise RuntimeError(
            f"Healthy checkpoint emitted {flat_action.size} actions, expected 22"
        )
    padded = np.zeros(model.nu, dtype=flat_action.dtype)
    padded[:LEG_ACTUATOR_COUNT] = flat_action
    return padded


def enable_neutral_lock(model, data) -> int:
    """Enable the test-only lock and reset the lumbar coordinate to neutral."""
    import mujoco

    equality_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "lumbar_neutral_lock"
    )
    joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "lumbar_extension"
    )
    if equality_id < 0 or joint_id < 0:
        raise RuntimeError("The integrated model lacks its lumbar joint or neutral lock")
    data.eq_active[equality_id] = 1
    data.qpos[model.jnt_qposadr[joint_id]] = 0.0
    data.qvel[model.jnt_dofadr[joint_id]] = 0.0
    mujoco.mj_forward(model, data)
    return equality_id


def validate_compatibility(model, policy, observation: np.ndarray) -> None:
    names = actuator_names(model)
    if model.nu != 28 or model.na != 28:
        raise RuntimeError(
            f"Integrated model must have nu=28 and na=28, got "
            f"nu={model.nu}, na={model.na}"
        )
    if names[LEG_ACTUATOR_COUNT:] != LUMBAR_ACTUATORS:
        raise RuntimeError(
            "The final six actuators are not the expected lumbar muscles: "
            f"{names[LEG_ACTUATOR_COUNT:]}"
        )
    if policy.action_space.shape != (LEG_ACTUATOR_COUNT,):
        raise RuntimeError(
            f"Checkpoint action space is {policy.action_space.shape}, expected (22,)"
        )
    if observation.shape != policy.observation_space.shape:
        raise RuntimeError(
            f"Adapted observation is {observation.shape}, but checkpoint expects "
            f"{policy.observation_space.shape}"
        )


def build_environment(config_path: Path, render: bool):
    from rl_train.envs.environment_handler import EnvironmentHandler
    from rl_train.train.train_configs.config_imitation import (
        ImitationTrainSessionConfig,
    )

    config = EnvironmentHandler.get_session_config_from_path(
        str(config_path), ImitationTrainSessionConfig
    )
    config.env_params.model_path = INTEGRATED_MODEL
    config.env_params.enable_lumbar_joint = False
    config.env_params.lumbar_joint_fixed_angle = 0.0
    config.env_params.flag_random_ref_index = False
    environment = EnvironmentHandler.create_environment(
        config, is_rendering_on=render, is_evaluate_mode=True
    )
    if render:
        environment.viewer_setup(render_actuator=True, render_tendon=True)
    return config, environment


def load_legacy_policy(checkpoint: Path):
    from stable_baselines3 import PPO
    from rl_train.train.policies.rl_agent_human import HumanActorCriticPolicy

    return PPO.load(
        str(checkpoint),
        custom_objects={"policy_class": HumanActorCriticPolicy},
        device="cpu",
    )


def reset_episode(env, config) -> np.ndarray:
    observation, _ = env.reset()
    equality_id = enable_neutral_lock(env.sim.model, env.sim.data)
    if not env.sim.data.eq_active[equality_id]:
        raise RuntimeError("Lumbar neutral lock did not remain active")
    env.sim.data.ctrl[LEG_ACTUATOR_COUNT:] = 0.0
    env.sim.data.act[LEG_ACTUATOR_COUNT:] = 0.0
    env.sim.forward()
    return legacy_observation(
        env.get_obs(),
        len(config.env_params.observation_joint_pos_keys),
        len(config.env_params.observation_joint_vel_keys),
    )


def run_smoke_test(args: argparse.Namespace) -> int:
    """Validate the adapter and physics without constructing a macOS viewer."""
    import mujoco

    with args.config.open("r", encoding="utf-8") as config_file:
        config_dict = json.load(config_file)
    env_params = config_dict["env_params"]
    qpos_count = len(env_params["observation_joint_pos_keys"])
    qvel_count = len(env_params["observation_joint_vel_keys"])

    model = mujoco.MjModel.from_xml_path(str(REPO_ROOT / INTEGRATED_MODEL))
    data = mujoco.MjData(model)
    stand = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, stand)
    equality_id = enable_neutral_lock(model, data)
    policy = load_legacy_policy(args.checkpoint)

    full_observation_size = policy.observation_space.shape[0] + len(
        LUMBAR_ACTUATORS
    )
    full_observation = np.zeros(full_observation_size, dtype=np.float32)
    observation = legacy_observation(
        full_observation, qpos_count, qvel_count
    )
    validate_compatibility(model, policy, observation)
    action, _ = policy.predict(observation, deterministic=True)
    padded_action = zero_lumbar_action(action, model)

    for _ in range(args.smoke_steps):
        data.ctrl[:] = padded_action
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise RuntimeError("Non-finite state in compatibility smoke test")
        if not np.allclose(data.ctrl[LEG_ACTUATOR_COUNT:], 0.0):
            raise RuntimeError("A lumbar actuator received nonzero control")
        if not data.eq_active[equality_id]:
            raise RuntimeError("Lumbar neutral lock became inactive")

    print("Healthy 28-muscle compatibility model ready")
    print("  checkpoint action/observation interface: 22 actions / 44 values")
    print("  physical model: 28 muscles")
    print("  lumbar controls and policy weights: none (six exact zeros)")
    print("  lumbar joint: locked at neutral")
    print(f"PASS: {args.smoke_steps}-step compatibility smoke test")
    return 0


def run(args: argparse.Namespace) -> int:
    if args.smoke_test:
        return run_smoke_test(args)

    config, env = build_environment(args.config, render=True)
    try:
        policy = load_legacy_policy(args.checkpoint)
        observation = reset_episode(env, config)
        validate_compatibility(env.sim.model, policy, observation)

        print("Healthy 28-muscle compatibility model ready")
        print("  legacy policy outputs: 22")
        print("  physical model controls: 28")
        print("  lumbar controls: six exact zeros")
        print("  lumbar policy weights: none")
        print("  lumbar joint: locked at neutral")

        completed_steps = 0
        while not env.sim.renderer._user_exit:
            action, _ = policy.predict(observation, deterministic=True)
            padded_action = zero_lumbar_action(action, env.sim.model)
            next_observation, _, terminated, truncated, _ = env.step(padded_action)
            if not np.allclose(env.sim.data.ctrl[LEG_ACTUATOR_COUNT:], 0.0):
                raise RuntimeError("A lumbar actuator received nonzero control")
            observation = legacy_observation(
                next_observation,
                len(config.env_params.observation_joint_pos_keys),
                len(config.env_params.observation_joint_vel_keys),
            )
            completed_steps += 1

            episode_finished = (
                terminated
                or truncated
                or completed_steps % args.episode_steps == 0
            )
            if episode_finished:
                observation = reset_episode(env, config)

        return 0
    finally:
        env.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument(
        "--episode-steps",
        type=int,
        default=1000,
        help="maximum visual steps before replaying from the same gait phase",
    )
    parser.add_argument("--smoke-test", action="store_true")
    parser.add_argument("--smoke-steps", type=int, default=20)
    args = parser.parse_args()
    if args.episode_steps <= 0 or args.smoke_steps <= 0:
        parser.error("step counts must be positive")
    for label, path in (("config", args.config), ("checkpoint", args.checkpoint)):
        if not path.exists() and not path.with_suffix(".zip").exists():
            parser.error(f"{label} not found: {path}")
    return args


def main() -> int:
    os.chdir(REPO_ROOT)
    return run(parse_args())


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Stopped")
        raise SystemExit(130)
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
