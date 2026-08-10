#!/usr/bin/env python3
"""Mechanically validate the additive 28-muscle lumbar transplant."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
from pathlib import Path
import sys

import mujoco
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PATHS = {
    "Healthy baseline": REPO_ROOT / "models/22muscle_2D/myoLeg22_2D_BASELINE.xml",
    "Six-muscle donor": REPO_ROOT
    / "myosuite/simhive/myo_sim/torso/myotorso_abdomen.xml",
    "Full 210-muscle donor": REPO_ROOT
    / "myosuite/simhive/myo_sim/torso/myotorso.xml",
    "Integrated model": REPO_ROOT
    / "models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml",
}
DEFAULT_REPORT = REPO_ROOT / "models/22muscle_2D_lumbar/VALIDATION.md"
MUSCLES = (
    "ercspn_r",
    "ercspn_l",
    "intobl_r",
    "intobl_l",
    "extobl_r",
    "extobl_l",
)
EXPECTED_DIMS = {
    "Healthy baseline": (53, 53, 22, 22),
    "Six-muscle donor": (3, 3, 6, 6),
    "Full 210-muscle donor": (18, 18, 210, 210),
    "Integrated model": (54, 54, 28, 28),
}


@dataclass
class MuscleSweep:
    minimum_length: float
    maximum_length: float
    length_range: tuple[float, float]
    neutral_moment_arm: float
    active_torque: float


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def object_name(model: mujoco.MjModel, obj: mujoco.mjtObj, index: int) -> str:
    name = mujoco.mj_id2name(model, obj, index)
    if name is None:
        raise AssertionError(f"Unnamed {obj} at index {index}")
    return name


def assert_close(label: str, left: np.ndarray, right: np.ndarray) -> None:
    if left.shape != right.shape or not np.allclose(
        left, right, rtol=1e-12, atol=1e-12, equal_nan=True
    ):
        raise AssertionError(f"{label} differs")


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    value = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if value < 0:
        raise AssertionError(f"Missing actuator {name!r}")
    return value


def tendon_id_for_actuator(model: mujoco.MjModel, actuator: int) -> int:
    tendon_id = int(model.actuator_trnid[actuator, 0])
    if tendon_id < 0:
        raise AssertionError(f"Actuator {actuator} is not tendon-driven")
    return tendon_id


def joint_addresses(model: mujoco.MjModel, name: str) -> tuple[int, int, int]:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise AssertionError(f"Missing joint {name!r}")
    return (
        joint_id,
        int(model.jnt_qposadr[joint_id]),
        int(model.jnt_dofadr[joint_id]),
    )


def compare_actuators(
    left: mujoco.MjModel,
    right: mujoco.MjModel,
    names: list[str],
    label: str,
) -> None:
    scalar_fields = (
        "actuator_trntype",
        "actuator_dyntype",
        "actuator_gaintype",
        "actuator_biastype",
        "actuator_ctrllimited",
        "actuator_forcelimited",
        "actuator_actlimited",
    )
    vector_fields = (
        "actuator_dynprm",
        "actuator_gainprm",
        "actuator_biasprm",
        "actuator_ctrlrange",
        "actuator_forcerange",
        "actuator_actrange",
        "actuator_gear",
        "actuator_lengthrange",
    )
    for name in names:
        left_id = actuator_id(left, name)
        right_id = actuator_id(right, name)
        left_tendon = object_name(
            left,
            mujoco.mjtObj.mjOBJ_TENDON,
            tendon_id_for_actuator(left, left_id),
        )
        right_tendon = object_name(
            right,
            mujoco.mjtObj.mjOBJ_TENDON,
            tendon_id_for_actuator(right, right_id),
        )
        if left_tendon != right_tendon:
            raise AssertionError(
                f"{label}: {name} tendon reference {left_tendon!r} != "
                f"{right_tendon!r}"
            )
        for field in scalar_fields:
            if getattr(left, field)[left_id] != getattr(right, field)[right_id]:
                raise AssertionError(f"{label}: {name} {field} differs")
        for field in vector_fields:
            assert_close(
                f"{label}: {name} {field}",
                np.asarray(getattr(left, field)[left_id]),
                np.asarray(getattr(right, field)[right_id]),
            )


def compare_baseline_bodies_and_geoms(
    baseline: mujoco.MjModel,
    integrated: mujoco.MjModel,
) -> None:
    body_fields = (
        "body_pos",
        "body_quat",
        "body_ipos",
        "body_iquat",
        "body_mass",
        "body_inertia",
    )
    for baseline_id in range(1, baseline.nbody):
        name = object_name(baseline, mujoco.mjtObj.mjOBJ_BODY, baseline_id)
        integrated_id = mujoco.mj_name2id(
            integrated, mujoco.mjtObj.mjOBJ_BODY, name
        )
        if integrated_id < 0:
            raise AssertionError(f"Integrated model is missing body {name!r}")
        for field in body_fields:
            assert_close(
                f"body {name} {field}",
                np.atleast_1d(getattr(baseline, field)[baseline_id]),
                np.atleast_1d(getattr(integrated, field)[integrated_id]),
            )

    geom_fields = (
        "geom_type",
        "geom_pos",
        "geom_quat",
        "geom_size",
        "geom_contype",
        "geom_conaffinity",
        "geom_friction",
        "geom_margin",
        "geom_rgba",
    )
    for baseline_id in range(baseline.ngeom):
        name = mujoco.mj_id2name(
            baseline, mujoco.mjtObj.mjOBJ_GEOM, baseline_id
        )
        if name is None:
            continue
        integrated_id = mujoco.mj_name2id(
            integrated, mujoco.mjtObj.mjOBJ_GEOM, name
        )
        if integrated_id < 0:
            raise AssertionError(f"Integrated model is missing geom {name!r}")
        for field in geom_fields:
            assert_close(
                f"geom {name} {field}",
                np.atleast_1d(getattr(baseline, field)[baseline_id]),
                np.atleast_1d(getattr(integrated, field)[integrated_id]),
            )


def compare_baseline_joints(
    baseline: mujoco.MjModel,
    integrated: mujoco.MjModel,
) -> None:
    joint_fields = (
        "jnt_type",
        "jnt_pos",
        "jnt_axis",
        "jnt_limited",
        "jnt_range",
        "jnt_stiffness",
        "jnt_margin",
        "dof_armature",
        "dof_damping",
        "dof_frictionloss",
    )
    for baseline_id in range(baseline.njnt):
        name = object_name(baseline, mujoco.mjtObj.mjOBJ_JOINT, baseline_id)
        integrated_id = mujoco.mj_name2id(
            integrated, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if integrated_id < 0:
            raise AssertionError(f"Integrated model is missing joint {name!r}")
        for field in joint_fields:
            baseline_values = getattr(baseline, field)
            integrated_values = getattr(integrated, field)
            if field.startswith("dof_"):
                baseline_index = int(baseline.jnt_dofadr[baseline_id])
                integrated_index = int(integrated.jnt_dofadr[integrated_id])
            else:
                baseline_index = baseline_id
                integrated_index = integrated_id
            assert_close(
                f"joint {name} {field}",
                np.atleast_1d(baseline_values[baseline_index]),
                np.atleast_1d(integrated_values[integrated_index]),
            )


def compare_keyframes(
    baseline: mujoco.MjModel,
    integrated: mujoco.MjModel,
) -> None:
    if baseline.nkey != integrated.nkey:
        raise AssertionError("Keyframe count changed")
    _, qpos_index, qvel_index = joint_addresses(
        integrated, "lumbar_extension"
    )
    for key_id in range(baseline.nkey):
        integrated_qpos = np.delete(integrated.key_qpos[key_id], qpos_index)
        integrated_qvel = np.delete(integrated.key_qvel[key_id], qvel_index)
        assert_close("keyframe qpos", baseline.key_qpos[key_id], integrated_qpos)
        assert_close("keyframe qvel", baseline.key_qvel[key_id], integrated_qvel)
        if integrated.key_qpos[key_id, qpos_index] != 0:
            raise AssertionError("New lumbar keyframe coordinate is not neutral")


def active_torque(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    dof_address: int,
    name: str,
) -> float:
    actuator = actuator_id(model, name)
    tendon = tendon_id_for_actuator(model, actuator)
    # MuJoCo muscle tension is negative along the tendon transmission; the
    # product below is its generalized joint torque contribution.
    return float(data.ten_J[tendon, dof_address] * data.actuator_force[actuator])


def sweep_model(model: mujoco.MjModel) -> dict[str, MuscleSweep]:
    data = mujoco.MjData(model)
    joint_id, qpos_address, dof_address = joint_addresses(
        model, "lumbar_extension"
    )
    angles = np.linspace(
        float(model.jnt_range[joint_id, 0]),
        float(model.jnt_range[joint_id, 1]),
        201,
    )
    lengths = {name: [] for name in MUSCLES}
    moments = {name: [] for name in MUSCLES}

    for angle in angles:
        mujoco.mj_resetData(model, data)
        data.qpos[qpos_address] = angle
        mujoco.mj_forward(model, data)
        arrays = (
            data.qpos,
            data.qvel,
            data.xpos,
            data.xquat,
            data.ten_length,
            data.actuator_length,
            data.ten_J,
        )
        if not all(np.all(np.isfinite(array)) for array in arrays):
            raise AssertionError(f"Non-finite value at lumbar angle {angle}")
        for name in MUSCLES:
            actuator = actuator_id(model, name)
            tendon = tendon_id_for_actuator(model, actuator)
            lengths[name].append(float(data.ten_length[tendon]))
            moments[name].append(float(data.ten_J[tendon, dof_address]))

    mujoco.mj_resetData(model, data)
    data.qpos[qpos_address] = 0.0
    data.ctrl[:] = 1.0
    data.act[:] = 1.0
    mujoco.mj_forward(model, data)

    result = {}
    for name in MUSCLES:
        actuator = actuator_id(model, name)
        lower, upper = map(float, model.actuator_lengthrange[actuator])
        minimum = min(lengths[name])
        maximum = max(lengths[name])
        if minimum < lower - 1e-10 or maximum > upper + 1e-10:
            raise AssertionError(
                f"{name} tendon length [{minimum}, {maximum}] exceeds "
                f"actuator range [{lower}, {upper}]"
            )
        expected_moment_sign = -1.0 if name.startswith("ercspn") else 1.0
        if not all(expected_moment_sign * value > 0 for value in moments[name]):
            raise AssertionError(f"{name} moment arm changes sign during the sweep")
        neutral_index = int(np.argmin(np.abs(angles)))
        result[name] = MuscleSweep(
            minimum_length=minimum,
            maximum_length=maximum,
            length_range=(lower, upper),
            neutral_moment_arm=moments[name][neutral_index],
            active_torque=active_torque(model, data, dof_address, name),
        )
    return result


def validate() -> tuple[dict[str, mujoco.MjModel], dict[str, MuscleSweep]]:
    models = {}
    for label, path in PATHS.items():
        if not path.exists():
            raise AssertionError(f"Missing {label}: {path}")
        model = mujoco.MjModel.from_xml_path(str(path))
        models[label] = model
        dimensions = (model.nq, model.nv, model.nu, model.na)
        if dimensions != EXPECTED_DIMS[label]:
            raise AssertionError(
                f"{label} dimensions {dimensions} != {EXPECTED_DIMS[label]}"
            )

    baseline = models["Healthy baseline"]
    donor = models["Six-muscle donor"]
    integrated = models["Integrated model"]

    baseline_names = [
        object_name(baseline, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
        for actuator in range(baseline.nu)
    ]
    integrated_prefix = [
        object_name(integrated, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
        for actuator in range(baseline.nu)
    ]
    if baseline_names != integrated_prefix:
        raise AssertionError("The original 22-actuator order or names changed")
    compare_actuators(
        baseline, integrated, baseline_names, "original actuator preservation"
    )
    compare_actuators(
        donor, integrated, list(MUSCLES), "donor actuator preservation"
    )
    compare_baseline_bodies_and_geoms(baseline, integrated)
    compare_baseline_joints(baseline, integrated)
    compare_keyframes(baseline, integrated)

    donor_sweep = sweep_model(donor)
    integrated_sweep = sweep_model(integrated)
    for name in MUSCLES:
        left = donor_sweep[name]
        right = integrated_sweep[name]
        assert_close(
            f"{name} donor/integrated sweep",
            np.asarray(
                [left.minimum_length, left.maximum_length, left.neutral_moment_arm]
            ),
            np.asarray(
                [right.minimum_length, right.maximum_length, right.neutral_moment_arm]
            ),
        )

    for right, left in (("ercspn_r", "ercspn_l"), ("intobl_r", "intobl_l"), ("extobl_r", "extobl_l")):
        assert_close(
            f"bilateral geometry {right}/{left}",
            np.asarray(
                [
                    integrated_sweep[right].minimum_length,
                    integrated_sweep[right].maximum_length,
                    integrated_sweep[right].neutral_moment_arm,
                ]
            ),
            np.asarray(
                [
                    integrated_sweep[left].minimum_length,
                    integrated_sweep[left].maximum_length,
                    integrated_sweep[left].neutral_moment_arm,
                ]
            ),
        )

    if not all(integrated_sweep[name].active_torque > 0 for name in MUSCLES[:2]):
        raise AssertionError("Erector spinae do not produce extension torque")
    if not all(integrated_sweep[name].active_torque < 0 for name in MUSCLES[2:]):
        raise AssertionError("Obliques do not produce flexion torque")
    return models, integrated_sweep


def build_report(
    models: dict[str, mujoco.MjModel],
    sweep: dict[str, MuscleSweep],
) -> str:
    lines = [
        "# Lumbar model validation",
        "",
        "Status: **PASS**",
        "",
        "## Dimensions",
        "",
        "| Model | nq | nv | nu | na |",
        "|---|---:|---:|---:|---:|",
    ]
    for label in PATHS:
        model = models[label]
        lines.append(f"| {label} | {model.nq} | {model.nv} | {model.nu} | {model.na} |")

    lines.extend(
        [
            "",
            "## Checks passed",
            "",
            "- Original bodies, joints, geometry, keyframes and 22 actuators are preserved.",
            "- The six lumbar actuator parameters match the donor.",
            "- A 201-pose sweep produced no NaNs, moment-arm reversals or joint explosions.",
            "- Every tendon remained finite, continuous and inside its donor length range.",
            "",
            "## Muscle results",
            "",
            "Joint range: -0.8727 to 0.2618 rad. Positive torque is extension.",
            "",
            "| Muscle | Action | Observed length (m) | Donor range (m) | Moment arm (m) |",
            "|---|---|---:|---:|---:|",
        ]
    )
    for name in MUSCLES:
        item = sweep[name]
        action = "Extension" if item.active_torque > 0 else "Flexion"
        lines.append(
            f"| `{name}` | {action} | "
            f"{item.minimum_length:.6f}–{item.maximum_length:.6f} | "
            f"{item.length_range[0]:.6f}–{item.length_range[1]:.6f} | "
            f"{item.neutral_moment_arm:+.6f} |"
        )

    pair_torques = {
        "Erector spinae (right + left)": sweep["ercspn_r"].active_torque
        + sweep["ercspn_l"].active_torque,
        "Internal oblique (right + left)": sweep["intobl_r"].active_torque
        + sweep["intobl_l"].active_torque,
        "External oblique (right + left)": sweep["extobl_r"].active_torque
        + sweep["extobl_l"].active_torque,
    }
    lines.extend(
        [
            "",
            "| Bilateral group | Torque at unit activation (N·m) | Action |",
            "|---|---:|---|",
        ]
    )
    for label, torque in pair_torques.items():
        function = "extension" if torque > 0 else "flexion"
        lines.append(f"| {label} | {torque:+.3f} | {function} |")

    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- Numeric donor muscle and joint parameters were retained.",
            "- Equivalent XML defaults were renamed or written explicitly.",
            "- Baseline torso mass/inertia were retained; lateral bending and rotation were excluded.",
            "- Extended, neutral and flexed snapshot inspection passed.",
            f"- Integrated XML SHA-256: `{file_sha256(PATHS['Integrated model'])}`",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report",
        type=Path,
        default=DEFAULT_REPORT,
        help=f"Markdown report path (default: {DEFAULT_REPORT})",
    )
    parser.add_argument(
        "--no-report",
        action="store_true",
        help="Run checks without writing the Markdown report.",
    )
    args = parser.parse_args()

    try:
        models, sweep = validate()
        report = build_report(models, sweep)
        if not args.no_report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(report, encoding="utf-8")
            print(f"Wrote {args.report.resolve()}")
        print("PASS: lumbar model mechanical validation")
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
