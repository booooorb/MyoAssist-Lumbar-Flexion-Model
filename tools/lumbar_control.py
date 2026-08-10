#!/usr/bin/env python3
"""Small non-RL control helpers for the integrated lumbar muscle model."""

from __future__ import annotations

import mujoco
import numpy as np


EXTENSORS = ("ercspn_r", "ercspn_l")
FLEXORS = ("intobl_r", "intobl_l", "extobl_r", "extobl_l")
LUMBAR_ACTUATORS = EXTENSORS + FLEXORS


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    identifier = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
    )
    if identifier < 0:
        raise ValueError(f"Model does not contain lumbar actuator {name!r}")
    return identifier


def set_signed_lumbar_control(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    command: float,
    *,
    immediate_activation: bool = False,
) -> float:
    """Map a signed command to the six unidirectional muscle controls.

    Positive values activate the erector spinae for extension. Negative values
    activate both oblique pairs for flexion. The returned value is the clipped
    command in ``[-1, 1]``.

    ``immediate_activation`` is intended for static physics checks. Normal
    simulation should leave it false so MuJoCo's muscle activation dynamics
    remain active.
    """
    clipped = float(np.clip(command, -1.0, 1.0))
    for name in LUMBAR_ACTUATORS:
        identifier = actuator_id(model, name)
        data.ctrl[identifier] = 0.0
        if immediate_activation:
            address = int(model.actuator_actadr[identifier])
            count = int(model.actuator_actnum[identifier])
            if address >= 0 and count:
                data.act[address : address + count] = 0.0

    active_group = EXTENSORS if clipped >= 0 else FLEXORS
    magnitude = abs(clipped)
    for name in active_group:
        identifier = actuator_id(model, name)
        data.ctrl[identifier] = magnitude
        if immediate_activation:
            address = int(model.actuator_actadr[identifier])
            count = int(model.actuator_actnum[identifier])
            if address >= 0 and count:
                data.act[address : address + count] = magnitude
    return clipped
