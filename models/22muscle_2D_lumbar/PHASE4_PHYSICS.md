# Phase 4: physics validation without RL

Status: **MECHANICS PASS; UNACTUATED STANDING REQUIRES A CONTROLLER**

| Check | Result | Measurement |
|---|---|---|
| XML loads | PASS | Integrated model is `nq=54`, `nv=54`, `nu=28`, `na=28` |
| Full lumbar sweep | PASS | -0.8727 to 0.2618 rad; 201 poses |
| Positive signed control | PASS | +222.633 N·m; +0.0877 rad after 10 ms |
| Negative signed control | PASS | -243.201 N·m; -0.0455 rad after 10 ms |
| Tendons and moment arms | PASS | Finite, donor-matched, no sign reversals |
| Initial stand pose | PASS | 14 baseline-matched foot contacts |
| Passive stand for 1 s | CONTROLLER REQUIRED | Baseline pelvis 0.261 m; integrated 0.202 m |
| Passive lumbar torque | PASS | Maximum 13.016 N·m (5.4% of active reference) |
| Neutral-lumbar stability | PASS | Maximum |q|=4.21e-05 rad, |dq|=0.027 rad/s |
| Collision sweep | PASS | Foot contacts only; maximum existing foot penetration 0.0079 m |
| Static leg equivalence | PASS | Maximum tendon/leg moment-arm error 0.00e+00 |
| 100 ms leg dynamics | PASS | Maximum q error 9.41e-04 rad (limit 1.5e-3); dq error 0.070 rad/s (limit 0.10) |
| Inherited keyframe tolerances | RECORDED | 9 baseline-matched auxiliary coordinates (details below) |

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

- `knee_r_translation2: -0.3950000 vs [-0.4226000, -0.3953000] (3.0e-04 coordinate units outside)`
- `knee_l_translation2: -0.3950000 vs [-0.4226000, -0.3953000] (3.0e-04 coordinate units outside)`
- `gastroc_l_med_gas_l-P2_z: 0.0250000 vs [-0.0258000, -0.0235000] (4.9e-02 coordinate units outside)`
- `iliopsoas_r_psoas_r-P3_x: 0.0000000 vs [-0.0288000, -0.0238000] (2.4e-02 coordinate units outside)`
- `iliopsoas_r_psoas_r-P3_y: 0.0000000 vs [-0.0805000, -0.0570000] (5.7e-02 coordinate units outside)`
- `iliopsoas_r_psoas_r-P3_z: 0.0000000 vs [0.0759000, 0.0816000] (7.6e-02 coordinate units outside)`
- `iliopsoas_l_psoas_l-P3_x: 0.0000000 vs [-0.0288000, -0.0238000] (2.4e-02 coordinate units outside)`
- `iliopsoas_l_psoas_l-P3_y: 0.0000000 vs [-0.0805000, -0.0570000] (5.7e-02 coordinate units outside)`
- `iliopsoas_l_psoas_l-P3_z: 0.0000000 vs [-0.0816000, -0.0759000] (7.6e-02 coordinate units outside)`
