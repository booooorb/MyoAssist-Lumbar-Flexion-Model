#!/usr/bin/env python3
"""Phase 4 physics checks for the healthy 28-muscle lumbar model."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import mujoco
import numpy as np

from lumbar_control import LUMBAR_ACTUATORS, set_signed_lumbar_control
from validate_lumbar_model import validate as validate_mechanics


REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_PATH = REPO_ROOT / "models/22muscle_2D/myoLeg22_2D_BASELINE.xml"
INTEGRATED_PATH = (
    REPO_ROOT / "models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml"
)
DEFAULT_REPORT = REPO_ROOT / "models/22muscle_2D_lumbar/PHASE4_PHYSICS.md"
LEG_JOINTS = (
    "pelvis_tx",
    "pelvis_ty",
    "pelvis_tilt",
    "hip_flexion_r",
    "knee_angle_r",
    "ankle_angle_r",
    "mtp_angle_r",
    "hip_flexion_l",
    "knee_angle_l",
    "ankle_angle_l",
    "mtp_angle_l",
)


@dataclass
class Phase4Results:
    extension_torque: float
    flexion_torque: float
    extension_displacement: float
    flexion_displacement: float
    maximum_passive_torque: float
    passive_to_active_ratio: float
    initial_contacts: int
    maximum_penetration: float
    neutral_lock_angle: float
    neutral_lock_speed: float
    static_leg_error: float
    dynamic_leg_position_error: float
    dynamic_leg_velocity_error: float
    baseline_height_after_1s: float
    integrated_height_after_1s: float
    passive_stand_available: bool
    inherited_keyframe_tolerances: tuple[str, ...]


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    identifier = mujoco.mj_name2id(model, kind, name)
    if identifier < 0:
        raise AssertionError(f"Missing {kind} named {name!r}")
    return identifier


def reset_stand(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    key = object_id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, key)


def joint_addresses(model: mujoco.MjModel, name: str) -> tuple[int, int]:
    joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])


def contact_rows(model: mujoco.MjModel, data: mujoco.MjData) -> list[tuple[str, str, float]]:
    rows = []
    for contact in data.contact[: data.ncon]:
        left = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        right = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        rows.append((left or "<unnamed>", right or "<unnamed>", float(contact.dist)))
    return sorted(rows)


def assert_foot_contacts_only(rows: list[tuple[str, str, float]]) -> None:
    surfaces = {"ground-plane", "terrain"}
    for left, right, _ in rows:
        pair = {left, right}
        surface = pair & surfaces
        other = right if left in surfaces else left
        if not surface or not (other.startswith("calcn_") or other.startswith("toes_")):
            raise AssertionError(f"Unexpected initial/sweep collision: {left} with {right}")


def assert_valid_stand_keyframe(
    model: mujoco.MjModel, data: mujoco.MjData
) -> tuple[str, ...]:
    if not np.all(np.isfinite(data.qpos)):
        raise AssertionError("Stand keyframe contains non-finite positions")
    inherited_tolerances = []
    for joint in range(model.njnt):
        if not model.jnt_limited[joint]:
            continue
        qpos_address = int(model.jnt_qposadr[joint])
        value = float(data.qpos[qpos_address])
        lower, upper = map(float, model.jnt_range[joint])
        violation = max(lower - value, value - upper, 0.0)
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if name == "lumbar_extension" and violation > 1e-6:
            raise AssertionError(
                f"Stand keyframe joint {name}={value} lies outside [{lower}, {upper}]"
            )
        if violation > 1e-6:
            inherited_tolerances.append(
                f"{name}: {value:.7f} vs [{lower:.7f}, {upper:.7f}] "
                f"({violation:.1e} coordinate units outside)"
            )
    return tuple(inherited_tolerances)


def command_torque(model: mujoco.MjModel, command: float) -> float:
    data = mujoco.MjData(model)
    reset_stand(model, data)
    set_signed_lumbar_control(
        model, data, command, immediate_activation=True
    )
    mujoco.mj_forward(model, data)
    _, dof = joint_addresses(model, "lumbar_extension")
    return float(data.qfrc_actuator[dof])


def command_displacement(model: mujoco.MjModel, command: float) -> float:
    """Return lumbar displacement after a short, fully activated 10 ms pulse."""
    data = mujoco.MjData(model)
    reset_stand(model, data)
    qpos_address, _ = joint_addresses(model, "lumbar_extension")
    initial_angle = float(data.qpos[qpos_address])
    set_signed_lumbar_control(model, data, command, immediate_activation=True)
    steps = max(1, int(round(0.010 / model.opt.timestep)))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    if not np.all(np.isfinite(data.qpos)):
        raise AssertionError("Non-finite state during signed-control pulse")
    return float(data.qpos[qpos_address] - initial_angle)


def passive_lumbar_torque_sweep(model: mujoco.MjModel) -> float:
    data = mujoco.MjData(model)
    joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, "lumbar_extension")
    qpos_address, dof_address = joint_addresses(model, "lumbar_extension")
    maximum = 0.0
    for angle in np.linspace(*model.jnt_range[joint], 201):
        mujoco.mj_resetData(model, data)
        data.qpos[qpos_address] = angle
        mujoco.mj_forward(model, data)
        torque = 0.0
        for name in LUMBAR_ACTUATORS:
            actuator = object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            tendon = int(model.actuator_trnid[actuator, 0])
            torque += data.ten_J[tendon, dof_address] * data.actuator_force[actuator]
        maximum = max(maximum, abs(float(torque)))
    return maximum


def validate_contacts_and_range(
    baseline: mujoco.MjModel,
    integrated: mujoco.MjModel,
) -> tuple[int, float, tuple[str, ...]]:
    baseline_data = mujoco.MjData(baseline)
    integrated_data = mujoco.MjData(integrated)
    reset_stand(baseline, baseline_data)
    reset_stand(integrated, integrated_data)
    mujoco.mj_forward(baseline, baseline_data)
    mujoco.mj_forward(integrated, integrated_data)
    baseline_tolerances = assert_valid_stand_keyframe(baseline, baseline_data)
    integrated_tolerances = assert_valid_stand_keyframe(integrated, integrated_data)
    if baseline_tolerances != integrated_tolerances:
        raise AssertionError("Inherited stand-keyframe tolerances changed")

    baseline_contacts = contact_rows(baseline, baseline_data)
    integrated_contacts = contact_rows(integrated, integrated_data)
    assert_foot_contacts_only(integrated_contacts)
    if [(a, b) for a, b, _ in baseline_contacts] != [
        (a, b) for a, b, _ in integrated_contacts
    ]:
        raise AssertionError("Initial contact pairs differ from the healthy baseline")
    if not np.allclose(
        [row[2] for row in baseline_contacts],
        [row[2] for row in integrated_contacts],
        rtol=0,
        atol=1e-12,
    ):
        raise AssertionError("Initial contact distances differ from the healthy baseline")

    joint = object_id(integrated, mujoco.mjtObj.mjOBJ_JOINT, "lumbar_extension")
    qpos_address, _ = joint_addresses(integrated, "lumbar_extension")
    maximum_penetration = 0.0
    for angle in np.linspace(*integrated.jnt_range[joint], 201):
        reset_stand(integrated, integrated_data)
        integrated_data.qpos[qpos_address] = angle
        mujoco.mj_forward(integrated, integrated_data)
        rows = contact_rows(integrated, integrated_data)
        assert_foot_contacts_only(rows)
        if rows:
            maximum_penetration = max(
                maximum_penetration, max(-distance for _, _, distance in rows)
            )

    wrap = object_id(integrated, mujoco.mjtObj.mjOBJ_GEOM, "pelvis_wrap")
    if integrated.geom_contype[wrap] or integrated.geom_conaffinity[wrap]:
        raise AssertionError("pelvis_wrap unexpectedly participates in collisions")
    if maximum_penetration > 0.01:
        raise AssertionError(
            f"Foot-ground penetration {maximum_penetration} m exceeds 0.01 m"
        )
    return len(integrated_contacts), maximum_penetration, integrated_tolerances


def validate_static_leg_equivalence(
    baseline: mujoco.MjModel,
    integrated: mujoco.MjModel,
) -> float:
    baseline_data = mujoco.MjData(baseline)
    integrated_data = mujoco.MjData(integrated)
    reset_stand(baseline, baseline_data)
    reset_stand(integrated, integrated_data)
    mujoco.mj_forward(baseline, baseline_data)
    mujoco.mj_forward(integrated, integrated_data)

    maximum_error = 0.0
    for actuator in range(baseline.nu):
        name = mujoco.mj_id2name(
            baseline, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator
        )
        integrated_actuator = object_id(
            integrated, mujoco.mjtObj.mjOBJ_ACTUATOR, name
        )
        baseline_tendon = int(baseline.actuator_trnid[actuator, 0])
        integrated_tendon = int(
            integrated.actuator_trnid[integrated_actuator, 0]
        )
        maximum_error = max(
            maximum_error,
            abs(
                float(
                    baseline_data.ten_length[baseline_tendon]
                    - integrated_data.ten_length[integrated_tendon]
                )
            ),
        )
        for joint_name in LEG_JOINTS:
            _, baseline_dof = joint_addresses(baseline, joint_name)
            _, integrated_dof = joint_addresses(integrated, joint_name)
            maximum_error = max(
                maximum_error,
                abs(
                    float(
                        baseline_data.ten_J[baseline_tendon, baseline_dof]
                        - integrated_data.ten_J[integrated_tendon, integrated_dof]
                    )
                ),
            )
    if maximum_error > 1e-12:
        raise AssertionError(f"Static leg tendon equivalence error: {maximum_error}")
    return maximum_error


def validate_neutral_dynamic_equivalence(
    baseline: mujoco.MjModel,
    integrated: mujoco.MjModel,
) -> tuple[float, float]:
    baseline_data = mujoco.MjData(baseline)
    integrated_data = mujoco.MjData(integrated)
    reset_stand(baseline, baseline_data)
    reset_stand(integrated, integrated_data)
    lock = object_id(
        integrated, mujoco.mjtObj.mjOBJ_EQUALITY, "lumbar_neutral_lock"
    )
    integrated_data.eq_active[lock] = 1

    maximum_position_error = 0.0
    maximum_velocity_error = 0.0
    for _ in range(100):
        mujoco.mj_step(baseline, baseline_data)
        mujoco.mj_step(integrated, integrated_data)
        for name in LEG_JOINTS:
            baseline_qpos, baseline_dof = joint_addresses(baseline, name)
            integrated_qpos, integrated_dof = joint_addresses(integrated, name)
            maximum_position_error = max(
                maximum_position_error,
                abs(
                    float(
                        baseline_data.qpos[baseline_qpos]
                        - integrated_data.qpos[integrated_qpos]
                    )
                ),
            )
            maximum_velocity_error = max(
                maximum_velocity_error,
                abs(
                    float(
                        baseline_data.qvel[baseline_dof]
                        - integrated_data.qvel[integrated_dof]
                    )
                ),
            )
    # The baseline torso is structurally rigid while the integrated torso uses
    # MuJoCo's soft equality solver for this test-only lock. Static leg geometry
    # must be exact; short-horizon dynamics are therefore checked to a tight,
    # explicitly reported numerical tolerance rather than bit-for-bit equality.
    if maximum_position_error > 0.0015 or maximum_velocity_error > 0.10:
        raise AssertionError(
            "Neutral-lumbar leg comparison exceeded tolerance: "
            f"q={maximum_position_error}, dq={maximum_velocity_error}"
        )
    return maximum_position_error, maximum_velocity_error


def validate_neutral_lock_stability(model: mujoco.MjModel) -> tuple[float, float]:
    data = mujoco.MjData(model)
    reset_stand(model, data)
    lock = object_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lumbar_neutral_lock")
    data.eq_active[lock] = 1
    qpos_address, dof_address = joint_addresses(model, "lumbar_extension")
    maximum_angle = 0.0
    maximum_speed = 0.0
    for _ in range(500):
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise AssertionError("Non-finite state during neutral-lock test")
        maximum_angle = max(maximum_angle, abs(float(data.qpos[qpos_address])))
        maximum_speed = max(maximum_speed, abs(float(data.qvel[dof_address])))
    if maximum_angle > 0.001 or maximum_speed > 0.1:
        raise AssertionError(
            f"Neutral lumbar is unstable: q={maximum_angle}, dq={maximum_speed}"
        )
    return maximum_angle, maximum_speed


def passive_stand_height(model: mujoco.MjModel) -> float:
    data = mujoco.MjData(model)
    reset_stand(model, data)
    pelvis = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    for _ in range(1000):
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)):
            raise AssertionError("Non-finite state during passive stand test")
    return float(data.xpos[pelvis, 2])


def run_phase4() -> Phase4Results:
    # Reuse the detailed XML, full-range tendon, moment-arm and torque checks.
    validate_mechanics()
    baseline = mujoco.MjModel.from_xml_path(str(BASELINE_PATH))
    integrated = mujoco.MjModel.from_xml_path(str(INTEGRATED_PATH))

    lock = object_id(
        integrated, mujoco.mjtObj.mjOBJ_EQUALITY, "lumbar_neutral_lock"
    )
    if integrated.eq_active0[lock]:
        raise AssertionError("lumbar_neutral_lock must be disabled by default")

    neutral_torque = command_torque(integrated, 0.0)
    extension_torque = command_torque(integrated, 1.0) - neutral_torque
    flexion_torque = command_torque(integrated, -1.0) - neutral_torque
    if extension_torque <= 0 or flexion_torque >= 0:
        raise AssertionError(
            f"Signed control directions are wrong: +={extension_torque}, -={flexion_torque}"
        )
    extension_displacement = command_displacement(integrated, 1.0)
    flexion_displacement = command_displacement(integrated, -1.0)
    if extension_displacement <= 0 or flexion_displacement >= 0:
        raise AssertionError(
            "Signed control does not move the lumbar joint in both directions: "
            f"+={extension_displacement}, -={flexion_displacement}"
        )

    passive_torque = passive_lumbar_torque_sweep(integrated)
    active_reference = max(abs(extension_torque), abs(flexion_torque))
    passive_ratio = passive_torque / active_reference
    if passive_ratio > 0.10:
        raise AssertionError(
            f"Passive lumbar torque is {passive_ratio:.1%} of active torque"
        )

    contacts, penetration, keyframe_tolerances = validate_contacts_and_range(
        baseline, integrated
    )
    static_error = validate_static_leg_equivalence(baseline, integrated)
    dynamic_q, dynamic_dq = validate_neutral_dynamic_equivalence(
        baseline, integrated
    )
    lock_q, lock_dq = validate_neutral_lock_stability(integrated)
    baseline_height = passive_stand_height(baseline)
    integrated_height = passive_stand_height(integrated)
    passive_stand = baseline_height > 0.75 and integrated_height > 0.75

    return Phase4Results(
        extension_torque=extension_torque,
        flexion_torque=flexion_torque,
        extension_displacement=extension_displacement,
        flexion_displacement=flexion_displacement,
        maximum_passive_torque=passive_torque,
        passive_to_active_ratio=passive_ratio,
        initial_contacts=contacts,
        maximum_penetration=penetration,
        neutral_lock_angle=lock_q,
        neutral_lock_speed=lock_dq,
        static_leg_error=static_error,
        dynamic_leg_position_error=dynamic_q,
        dynamic_leg_velocity_error=dynamic_dq,
        baseline_height_after_1s=baseline_height,
        integrated_height_after_1s=integrated_height,
        passive_stand_available=passive_stand,
        inherited_keyframe_tolerances=keyframe_tolerances,
    )


def build_report(results: Phase4Results) -> str:
    stand_status = "PASS" if results.passive_stand_available else "CONTROLLER REQUIRED"
    tolerance_note = (
        f"{len(results.inherited_keyframe_tolerances)} baseline-matched "
        "auxiliary coordinates (details below)"
        if results.inherited_keyframe_tolerances
        else "none"
    )
    tolerance_details = "\n".join(
        f"- `{item}`" for item in results.inherited_keyframe_tolerances
    )
    return f"""# Phase 4: physics validation without RL

Status: **MECHANICS PASS; UNACTUATED STANDING REQUIRES A CONTROLLER**

| Check | Result | Measurement |
|---|---|---|
| XML loads | PASS | Integrated model is `nq=54`, `nv=54`, `nu=28`, `na=28` |
| Full lumbar sweep | PASS | -0.8727 to 0.2618 rad; 201 poses |
| Positive signed control | PASS | {results.extension_torque:+.3f} N·m; {results.extension_displacement:+.4f} rad after 10 ms |
| Negative signed control | PASS | {results.flexion_torque:+.3f} N·m; {results.flexion_displacement:+.4f} rad after 10 ms |
| Tendons and moment arms | PASS | Finite, donor-matched, no sign reversals |
| Initial stand pose | PASS | {results.initial_contacts} baseline-matched foot contacts |
| Passive stand for 1 s | {stand_status} | Baseline pelvis {results.baseline_height_after_1s:.3f} m; integrated {results.integrated_height_after_1s:.3f} m |
| Passive lumbar torque | PASS | Maximum {results.maximum_passive_torque:.3f} N·m ({results.passive_to_active_ratio:.1%} of active reference) |
| Neutral-lumbar stability | PASS | Maximum |q|={results.neutral_lock_angle:.2e} rad, |dq|={results.neutral_lock_speed:.3f} rad/s |
| Collision sweep | PASS | Foot contacts only; maximum existing foot penetration {results.maximum_penetration:.4f} m |
| Static leg equivalence | PASS | Maximum tendon/leg moment-arm error {results.static_leg_error:.2e} |
| 100 ms leg dynamics | PASS | Maximum q error {results.dynamic_leg_position_error:.2e} rad (limit 1.5e-3); dq error {results.dynamic_leg_velocity_error:.3f} rad/s (limit 0.10) |
| Inherited keyframe tolerances | RECORDED | {tolerance_note} |

## Interpretation

- The six lumbar muscles are mechanically incorporated into the healthy walking model.
- `tools/lumbar_control.py` maps positive commands to the erector spinae and negative commands to both oblique pairs.
- The `lumbar_neutral_lock` equality is disabled by default. The validator enables it only to compare the legs fairly with the original rigid-torso model.
- Static leg muscle geometry is identical. Dynamic equivalence is tolerance-based because MuJoCo implements the test lock as a soft constraint, while the original torso is structurally rigid.
- Neither model stands passively with zero muscle control. This is expected because no standing/walking controller or RL weights are loaded; it is not evidence of a lumbar transplant failure.
- The recorded auxiliary-coordinate keyframe mismatches are present in both the source baseline and integrated model; the lumbar integration did not introduce them.
- A future controller must stabilize the original 22 leg muscles and command the six new lumbar muscles.

## Inherited keyframe details

These entries occur identically in the original healthy model and the integrated model. Most are auxiliary moving-point coordinates used by existing leg muscle paths, rather than the new lumbar joint.

{tolerance_details}
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--no-report", action="store_true")
    args = parser.parse_args()
    try:
        results = run_phase4()
        if not args.no_report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(build_report(results), encoding="utf-8")
            print(f"Wrote {args.report.resolve()}")
        print("PASS: Phase 4 mechanics; passive standing still requires a controller")
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
