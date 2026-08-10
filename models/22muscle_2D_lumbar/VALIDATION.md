# Lumbar model validation

Status: **PASS — automated mechanical checks**

The healthy 22-muscle XML and both donor XMLs were used as read-only inputs. The integrated model is a separate additive file.

## Compilation and dimensions

| Model | nq | nv | nu | na | Result |
|---|---:|---:|---:|---:|---|
| Healthy baseline | 53 | 53 | 22 | 22 | PASS |
| Six-muscle donor | 3 | 3 | 6 | 6 | PASS |
| Full 210-muscle donor | 18 | 18 | 210 | 210 | PASS |
| Integrated model | 54 | 54 | 28 | 28 | PASS |

## Preservation checks

- The first 22 actuator names, order, transmission references, limits, types, and numeric muscle parameters match the healthy baseline.
- Existing body poses, masses, inertias, and named geometry parameters match the healthy baseline.
- All 53 existing joint types, axes, limits, armatures, damping, and friction parameters match the healthy baseline.
- All existing keyframes match after removing the new neutral lumbar coordinate at index 33.
- The six compiled lumbar actuator parameters match the six-muscle donor.

## Lumbar sweep

The full donor joint interval, -0.8727 to 0.2618 rad, was sampled at 201 poses. Moment arm is `d(tendon length)/d(lumbar_extension)`. Under the donor convention, positive generalized torque is extension.

| Muscle | Function | Tendon length observed (m) | Donor muscle range (m) | Neutral moment arm (m) | Active torque sign |
|---|---|---:|---:|---:|---|
| `ercspn_r` | Right erector spinae — extension | 0.137731–0.186214 | 0.057206–0.194745 | -0.045939 | positive (extension) |
| `ercspn_l` | Left erector spinae — extension | 0.137731–0.186214 | 0.057206–0.194745 | -0.045939 | positive (extension) |
| `intobl_r` | Right internal oblique — flexion | 0.158893–0.212714 | 0.045401–0.306151 | +0.052822 | negative (flexion) |
| `intobl_l` | Left internal oblique — flexion | 0.158893–0.212714 | 0.045401–0.306151 | +0.052822 | negative (flexion) |
| `extobl_r` | Right external oblique — flexion | 0.201550–0.288516 | 0.044413–0.329629 | +0.062743 | negative (flexion) |
| `extobl_l` | Left external oblique — flexion | 0.201550–0.288516 | 0.044413–0.329629 | +0.062743 | negative (flexion) |

At neutral with unit activation, the bilateral torque checks are:

| Bilateral group | Generalized lumbar torque (N·m) | Result |
|---|---:|---|
| Erector spinae (right + left) | +222.633 | extension |
| Internal oblique (right + left) | -138.120 | flexion |
| External oblique (right + left) | -105.803 | flexion |

All swept tendon lengths were finite, remained inside their donor muscle length ranges, and matched the donor model. No NaNs or kinematic joint explosions occurred.

## Donor parameters represented differently

No numeric muscle, tendon-path, attachment-position, joint-range, damping, or armature parameter was changed. A few XML class references were resolved explicitly because the integrated file uses the baseline's default namespace:

- `myotorso_muscle` became the numerically identical baseline `muscle` class.
- Donor tendon width/colour, attachment-site size/group, and wrapping-geom collision/colour properties were written explicitly.
- The donor torso mass/inertia was intentionally not transplanted; the healthy baseline torso mass/inertia remains unchanged.
- Donor lateral-bending and axial-rotation joints were intentionally excluded.

## Visual inspection

Status: **PASS — three-pose snapshot review (2026-08-10)**

Integrated XML SHA-256: `3556432a54dbb3e8f9bc091009e4bdaa23f3bde2db922aed6e2b024cec28ba0c`

- The six donor paths and their integrated copies remain visually continuous at 0.15 rad extension, neutral, and -0.40 rad flexion.
- No tendon crossing reversal, detached segment, obvious lumbar body penetration, or joint explosion was visible in the integrated snapshots.
- The full 210-muscle donor also rendered in all three poses and remains reference-only.

This visual review supplements the numeric 201-pose sweep; it is not a contact-force or anatomical validation study.
