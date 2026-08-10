#!/usr/bin/env python3
"""Interactively inspect the lumbar donor and integrated MyoAssist models.

Run this script through ``mjpython`` on macOS.  The animation is kinematic: it
changes only the requested lumbar coordinate and never trains or updates a
policy.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time

import mujoco
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class ModelSpec:
    path: Path
    coordinate: str
    tracked_body: str
    extended_angle: float
    flexed_angle: float
    distance: float


MODEL_SPECS = {
    "abdomen": ModelSpec(
        path=REPO_ROOT / "myosuite/simhive/myo_sim/torso/myotorso_abdomen.xml",
        coordinate="lumbar_extension",
        tracked_body="lumbar",
        extended_angle=0.15,
        flexed_angle=-0.40,
        distance=1.35,
    ),
    "full": ModelSpec(
        path=REPO_ROOT / "myosuite/simhive/myo_sim/torso/myotorso.xml",
        coordinate="flex_extension",
        tracked_body="torso",
        extended_angle=0.20,
        flexed_angle=-0.40,
        distance=1.35,
    ),
    "integrated": ModelSpec(
        path=REPO_ROOT / "models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml",
        coordinate="lumbar_extension",
        tracked_body="torso",
        extended_angle=0.15,
        flexed_angle=-0.40,
        distance=3.0,
    ),
}

# The full model constrains these physical coordinates to its public
# flex_extension coordinate.  Updating them explicitly makes a kinematic
# preview match the equality constraints without advancing the simulation.
FULL_FLEXION_MAP = {
    "Abs_r3": 0.6429,
    "L4_L5_FE": 0.185,
    "L3_L4_FE": 0.204,
    "L2_L3_FE": 0.231,
    "L1_L2_FE": 0.255,
}


def joint_qpos_address(model: mujoco.MjModel, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"Joint {name!r} was not found")
    return int(model.jnt_qposadr[joint_id])


def set_flexion_coordinate(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    spec: ModelSpec,
    value: float,
) -> None:
    data.qpos[joint_qpos_address(model, spec.coordinate)] = value
    if spec.coordinate == "flex_extension":
        for joint_name, scale in FULL_FLEXION_MAP.items():
            data.qpos[joint_qpos_address(model, joint_name)] = scale * value


def reset_to_inspection_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    model_name: str,
) -> None:
    mujoco.mj_resetData(model, data)
    if model_name == "integrated":
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(model, data, key_id)


def apply_display_activation(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    activation: float,
) -> None:
    if model.nu:
        data.ctrl[:] = activation
    for actuator_id in range(model.nu):
        activation_address = int(model.actuator_actadr[actuator_id])
        activation_count = int(model.actuator_actnum[actuator_id])
        if activation_address >= 0 and activation_count > 0:
            data.act[
                activation_address : activation_address + activation_count
            ] = activation


def configure_visuals(option: mujoco.MjvOption) -> None:
    option.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = True
    option.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = True
    option.geomgroup[3] = True


def configure_camera(
    model: mujoco.MjModel,
    camera: mujoco.MjvCamera,
    spec: ModelSpec,
) -> None:
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, spec.tracked_body
    )
    if body_id < 0:
        raise RuntimeError(f"Body {spec.tracked_body!r} was not found")
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = body_id
    camera.distance = spec.distance
    camera.azimuth = 90.0
    camera.elevation = 0.0


def render_snapshots(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    model_name: str,
    spec: ModelSpec,
    output_dir: Path,
    activation: float,
) -> None:
    from PIL import Image

    output_dir.mkdir(parents=True, exist_ok=True)
    option = mujoco.MjvOption()
    configure_visuals(option)
    camera = mujoco.MjvCamera()
    configure_camera(model, camera, spec)
    # Offscreen rendering does not continuously update a tracking camera the
    # way the interactive viewer does, so use an explicitly centred free view.
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = min(spec.distance, 1.25)
    tracked_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, spec.tracked_body
    )

    poses = {
        "extended": spec.extended_angle,
        "neutral": 0.0,
        "flexed": spec.flexed_angle,
    }
    # The standalone donor models use MuJoCo's 640x480 default offscreen
    # framebuffer, so keep snapshots within that portable limit.
    renderer = mujoco.Renderer(model, height=480, width=640)
    try:
        for pose_name, angle in poses.items():
            reset_to_inspection_pose(model, data, model_name)
            set_flexion_coordinate(model, data, spec, angle)
            apply_display_activation(model, data, activation)
            mujoco.mj_forward(model, data)
            camera.lookat[:] = data.xpos[tracked_body_id]
            renderer.update_scene(data, camera=camera, scene_option=option)
            image = renderer.render()
            path = output_dir / f"{model_name}_{pose_name}.png"
            Image.fromarray(image).save(path)
            print(f"Wrote {path}")
    finally:
        renderer.close()


def launch_viewer(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    model_name: str,
    spec: ModelSpec,
    mode: str,
    cycle_seconds: float,
    activation: float,
    static_angle: float,
) -> None:
    # Importing here keeps non-interactive validation/imports independent of
    # the macOS GUI runtime requirement.
    import mujoco.viewer

    state = {"paused": mode == "static", "reset": False}

    def key_callback(keycode: int) -> None:
        if keycode == ord(" "):
            state["paused"] = not state["paused"]
        elif keycode in (ord("R"), ord("r")):
            state["reset"] = True

    reset_to_inspection_pose(model, data, model_name)
    set_flexion_coordinate(model, data, spec, static_angle)
    apply_display_activation(model, data, activation)
    mujoco.mj_forward(model, data)

    print(f"Model: {model_name} ({spec.path})")
    print(f"Control coordinate: {spec.coordinate}")
    print("Viewer controls: Space pauses/resumes; R returns to neutral; close the window to exit.")

    started_at = time.monotonic()
    with mujoco.viewer.launch_passive(
        model,
        data,
        key_callback=key_callback,
        show_left_ui=True,
        show_right_ui=True,
    ) as viewer:
        configure_visuals(viewer.opt)
        configure_camera(model, viewer.cam, spec)

        while viewer.is_running():
            frame_started = time.monotonic()
            if state["reset"]:
                set_flexion_coordinate(model, data, spec, 0.0)
                state["reset"] = False
                state["paused"] = True
            elif not state["paused"]:
                phase = 2.0 * np.pi * (frame_started - started_at) / cycle_seconds
                midpoint = 0.5 * (spec.extended_angle + spec.flexed_angle)
                amplitude = 0.5 * (spec.flexed_angle - spec.extended_angle)
                angle = midpoint + amplitude * np.sin(phase)
                set_flexion_coordinate(model, data, spec, float(angle))

            data.qvel[:] = 0.0
            apply_display_activation(model, data, activation)
            mujoco.mj_forward(model, data)
            viewer.sync()
            remaining = (1.0 / 60.0) - (time.monotonic() - frame_started)
            if remaining > 0:
                time.sleep(remaining)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=MODEL_SPECS, required=True)
    parser.add_argument("--mode", choices=("static", "animate"), default="static")
    parser.add_argument(
        "--cycle-seconds",
        type=float,
        default=12.0,
        help="Seconds per flexion-extension cycle in animate mode (default: 12).",
    )
    parser.add_argument(
        "--activation",
        type=float,
        default=0.10,
        help="Display activation applied to actuators, from 0 to 1 (default: 0.10).",
    )
    parser.add_argument(
        "--angle",
        type=float,
        default=0.0,
        help="Initial/static control-coordinate angle in radians (default: 0).",
    )
    parser.add_argument(
        "--capture-dir",
        type=Path,
        help="Render extended, neutral, and flexed PNGs to this directory, then exit.",
    )
    args = parser.parse_args()

    if args.cycle_seconds <= 0:
        parser.error("--cycle-seconds must be positive")
    if not 0.0 <= args.activation <= 1.0:
        parser.error("--activation must be between 0 and 1")

    spec = MODEL_SPECS[args.model]
    if not spec.path.exists():
        if args.model == "integrated":
            print(
                "Integrated model is missing. Run tools/build_lumbar_model.py first.",
                file=sys.stderr,
            )
        else:
            print(f"Model is missing: {spec.path}", file=sys.stderr)
        return 1

    model = mujoco.MjModel.from_xml_path(str(spec.path))
    data = mujoco.MjData(model)
    if args.capture_dir:
        render_snapshots(
            model,
            data,
            args.model,
            spec,
            args.capture_dir.resolve(),
            args.activation,
        )
        return 0

    try:
        launch_viewer(
            model,
            data,
            args.model,
            spec,
            args.mode,
            args.cycle_seconds,
            args.activation,
            args.angle,
        )
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
