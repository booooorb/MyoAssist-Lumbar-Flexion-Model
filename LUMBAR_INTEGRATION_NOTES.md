# Lumbar integration: changes and anatomical accuracy

## What was created

The new model is:

`models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml`

It is a separate copy of the healthy 22-muscle leg model with six lumbar
muscles added. The original healthy model and both MyoTorso donor models were
not overwritten.

The resulting actuator set is:

- 22 original lower-limb muscles;
- right and left erector spinae;
- right and left internal oblique;
- right and left external oblique.

No reinforcement-learning policy, reward, weakness configuration or walking
checkpoint was changed.

## Changes made to the model

### 1. Added one lumbar joint

A sagittal `lumbar_extension` hinge was added between the existing pelvis and
torso. Its properties came from the six-muscle `myotorso_abdomen` donor:

| Property | Value |
|---|---:|
| Range | -0.8727 to 0.2618 rad |
| Damping | 0.5 |
| Armature | 0.01 |
| Axis | Sagittal flexion–extension |

Under the donor convention, positive angle/torque represents extension and
negative angle/torque represents flexion.

### 2. Transplanted the muscle geometry

The donor pelvis attachment sites, torso attachment sites and cylindrical
pelvis wrapping geomeWeregin and the existing torso origin were
already identical, so no approximate rescaling or manual repositioning was
required.

The erector-spinae paths pass around the donor wrapping cylinder. The oblique
paths connect their pelvis and torso attachment sites directly.

### 3. Transplanted the tendon and actuator parameters

The six donor tendon paths, muscle length ranges, gain parameters, bias
parameters, activation dynamics and control limits were retained. XML class
names were changed where necessary to use the numerically equivalent defaults
already present in the leg model.

The original torso mass and inertia were deliberately retained. The donor
torso mass was not substituted.

### 4. Updated the existing poses

A neutral lumbar value was inserted into every existing keyframe. Removing
that added coordinate reproduces the original keyframe exactly.

### 5. Added inspection and validation tools

`tools/preview_lumbar_models.py` previews:

- the six-muscle abdomen donor;
- the full 210-muscle MyoTorso reference;
- the integrated 28-muscle model.

`tools/validate_lumbar_model.py` checks compilation, preserved model
parameters, tendon lengths, moment-arm signs, muscle torque directions and a
201-pose lumbar sweep.

## Validation results

The integrated model compiles with:

| Quantity | Value |
|---|---:|
| `nq` | 54 |
| `nv` | 54 |
| `nu` | 28 |
| `na` | 28 |

The checks confirmed that:

- all 22 original actuators and their parameters remain unchanged;
- existing bodies, inertias, geometry and joint parameters remain unchanged;
- all six lumbar tendon lengths remain inside the donor muscle ranges;
- right/left attachment geometry is symmetric;
- erector-spinae activation produces extension torque;
- internal- and external-oblique activation produces flexion torque;
- the tendon moment arms do not reverse during the permitted lumbar sweep;
- no NaNs, detached tendon segments or joint explosions appeared.

Detailed measurements are recorded in
`models/22muscle_2D_lumbar/VALIDATION.md`.

## Is it anatomically accurate?

### Accurate relative to the selected donor

The integration is mechanically faithful to the six-muscle MyoTorso abdomen
model. The effective attachment positions, tendon paths, wrapping geometry,
joint limits and compiled muscle parameters match that donor. The transplant
therefore preserves the anatomical assumptions already made by MyoSuite.

### Not a high-fidelity anatomical lumbar spine

The integrated model should be described as **anatomically inspired and
mechanically consistent**, not as a clinically complete lumbar model.

Its main limitations are:

- the lumbar spine is represented by one hinge and one rigid torso segment;
- individual vertebrae, discs, facet joints and spinal ligaments are absent;
- only sagittal flexion–extension is enabled;
- lateral bending and axial rotation are excluded;
- six grouped muscle paths represent much more complicated real muscle
  architecture;
- deep stabilizers such as multifidus and quadratus lumborum are not included;
- abdominal pressure and passive soft-tissue effects are not represented;
- the retained torso inertia belongs to the original leg model rather than the
  donor torso;
- muscle strengths have not been calibrated to a particular subject or
  validated against EMG, motion-capture or in-vivo spinal loading data.

The full 210-muscle MyoTorso model contains substantially more spinal and
muscle detail, but it depends on multiple vertebral bodies and cannot be
faithfully reduced to the current single-segment torso by copying its muscles
alone.

## Appropriate interpretation

This 28-muscle model is suitable for:

- confirming that active lumbar flexion and extension can be represented;
- visually inspecting simplified lumbar muscle paths;
- testing moment-arm and torque direction;
- preparing a later RL environment with six additional control outputs.

It is not yet sufficient for:

- clinical conclusions about lumbar loading or injury;
- predicting individual muscle recruitment in humans;
- comparing vertebra-by-vertebra motion;
- claiming validated human lumbar biomechanics.

Further anatomical validation should compare lumbar kinematics, moment arms,
maximum torque, muscle activation and spinal loads against experimental or
published human data before the model is used for biomechanical conclusions.
