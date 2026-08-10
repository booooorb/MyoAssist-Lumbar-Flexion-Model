# Lumbar model validation

Status: **PASS**

## Dimensions

| Model | nq | nv | nu | na |
|---|---:|---:|---:|---:|
| Healthy baseline | 53 | 53 | 22 | 22 |
| Six-muscle donor | 3 | 3 | 6 | 6 |
| Full 210-muscle donor | 18 | 18 | 210 | 210 |
| Integrated model | 54 | 54 | 28 | 28 |

## Checks passed

- Original bodies, joints, geometry, keyframes and 22 actuators are preserved.
- The six lumbar actuator parameters match the donor.
- A 201-pose sweep produced no NaNs, moment-arm reversals or joint explosions.
- Every tendon remained finite, continuous and inside its donor length range.

## Muscle results

Joint range: -0.8727 to 0.2618 rad. Positive torque is extension.

| Muscle | Action | Observed length (m) | Donor range (m) | Moment arm (m) |
|---|---|---:|---:|---:|
| `ercspn_r` | Extension | 0.137731–0.186214 | 0.057206–0.194745 | -0.045939 |
| `ercspn_l` | Extension | 0.137731–0.186214 | 0.057206–0.194745 | -0.045939 |
| `intobl_r` | Flexion | 0.158893–0.212714 | 0.045401–0.306151 | +0.052822 |
| `intobl_l` | Flexion | 0.158893–0.212714 | 0.045401–0.306151 | +0.052822 |
| `extobl_r` | Flexion | 0.201550–0.288516 | 0.044413–0.329629 | +0.062743 |
| `extobl_l` | Flexion | 0.201550–0.288516 | 0.044413–0.329629 | +0.062743 |

| Bilateral group | Torque at unit activation (N·m) | Action |
|---|---:|---|
| Erector spinae (right + left) | +222.633 | extension |
| Internal oblique (right + left) | -138.120 | flexion |
| External oblique (right + left) | -105.803 | flexion |

## Notes

- Numeric donor muscle and joint parameters were retained.
- Equivalent XML defaults were renamed or written explicitly.
- Baseline torso mass/inertia were retained; lateral bending and rotation were excluded.
- Extended, neutral and flexed snapshot inspection passed.
- Integrated XML SHA-256: `3556432a54dbb3e8f9bc091009e4bdaa23f3bde2db922aed6e2b024cec28ba0c`
