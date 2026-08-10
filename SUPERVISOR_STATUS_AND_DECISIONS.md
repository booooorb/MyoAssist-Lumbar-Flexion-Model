# Lumbar-flexion project: status and decisions requested

## Executive summary

I have created a new healthy 28-muscle, 2D walking model by retaining the
existing 22 leg muscles and adding six donor lumbar muscles. The model passes
mechanical, tendon-path, contact and control-direction checks without RL.

This establishes a **mechanically testable starting model**, not an
anatomically complete spine model and not an active-lumbar walking result.
The existing healthy walking policy has not been retrained and therefore does
not yet control the six lumbar muscles.

The immediate next experiment should be a corrected compatibility baseline:
run the old 22-output healthy policy on the 28-muscle body, with the lumbar
joint fixed at the same torso orientation used during healthy-policy training
and all six lumbar excitations at physical zero. This is required before
claiming that any later difference comes from active lumbar control.

## Completed work

| Area | Current state |
|---|---|
| Starting model | Original healthy 22-muscle 2D walking model retained unchanged |
| New model | Separate 28-muscle XML: original 22 leg muscles + 6 lumbar muscles |
| Lumbar muscles | Bilateral erector spinae, internal oblique and external oblique |
| Lumbar mechanics | One sagittal flexion-extension hinge between pelvis and torso |
| Muscle source | Existing MyoSuite `myotorso_abdomen` donor model |
| RL policy | No policy, reward, checkpoint or original training run overwritten |
| Physics validation | Completed without RL; see linked results below |

The added muscle paths retain the donor attachment, tendon and actuator
parameters. This was an integration step, not a subject-specific parameter
calibration.

## Physics validation summary

The integrated XML compiles with `nq=54`, `nv=54`, `nu=28`, and `na=28`.

| Check | Outcome |
|---|---|
| Full lumbar sweep | Passed across -0.8727 to +0.2618 rad |
| Signed lumbar control | Positive command causes extension; negative command causes flexion |
| Tendon paths/moment arms | Finite, continuous and without sign reversals in the sweep |
| Contacts | No new collision class; only pre-existing foot-ground contacts |
| Original leg geometry | Static tendon lengths and leg moment arms exactly match the 22-muscle baseline |
| Passive lumbar loading | Maximum passive lumbar torque: 13.0 N·m, 5.4% of tested active reference |
| Unactuated standing | Neither the original nor integrated model stands for 1 s with zero control; a controller is required |

Detailed measurements are in
[`models/22muscle_2D_lumbar/PHASE4_PHYSICS.md`](models/22muscle_2D_lumbar/PHASE4_PHYSICS.md).

## What is not yet established

- Healthy walking with active lumbar muscles.
- Whether lumbar control improves stability, tracking, recovery or any other
  functional outcome.
- Anatomical or clinical validity beyond the simplified six-muscle donor model.
- Appropriate lumbar activation patterns relative to EMG or experimental data.
- A lumbar reference trajectory or a validated definition of successful lumbar
  flexion during gait.

The model has one lumbar hinge and a rigid torso. It excludes vertebra-by-
vertebra motion, discs, ligaments, multifidus, quadratus lumborum and abdominal
pressure. It should therefore be described as a simplified sagittal lumbar
control model, not a detailed lumbar-spine model.

## Current compatibility issue

The old healthy checkpoint initially behaved poorly when run on the 28-muscle
model with its lumbar joint locked at 0 rad. Investigation showed that the
original healthy environment instead applies a torso orientation equivalent to
approximately **-0.25855 rad** at runtime. The policy therefore received leg
states from a body configuration different from its training configuration.

This does **not** mean the lumbar model must be retrained before the baseline
can be tested. The compatibility baseline should first lock the new lumbar
joint at -0.25855 rad and keep all six physical lumbar controls at zero.

## Proposed scope options

### Option A — smallest defensible study (recommended)

1. Correct and verify the locked, unpowered 28-muscle compatibility baseline.
2. Train/fine-tune a 28-output policy with active lumbar control.
3. Compare healthy gait with lumbar locked versus active.
4. Report kinematics, survival/tracking, effort and lumbar activation.

This retains the current 2D model and six-muscle representation. It answers
whether this simplified active lumbar module changes healthy gait behavior.

### Option B — recovery/weakness extension

Complete Option A first, then introduce unilateral weakness and compare locked
versus active lumbar control during gait recovery. This is substantially more
expensive because it adds training conditions and evaluation criteria.

### Option C — anatomical-model expansion

Move to a richer torso/spine model before RL work. This may be appropriate if
the research question requires vertebral motion or detailed trunk musculature,
but it invalidates direct compatibility with the current 22-output checkpoint
and substantially increases integration and training scope.

## Questions requiring supervisor direction

1. What is the primary research claim: a feasible lumbar-control extension,
   improved healthy gait, improved gait under weakness, or anatomical spinal
   fidelity?

2. Is the simplified six-muscle, one-hinge sagittal model acceptable for the
   first study, or is a multi-segment/greater-muscle-count trunk required from
   the outset?

3. Should the first active-lumbar objective imitate a measured lumbar motion,
   regulate a neutral/postural range, or allow lumbar motion freely while only
   penalizing implausible posture and effort?

4. Which outcomes define success? Suggested minimum set: gait survival,
   reference tracking error, pelvis/trunk stability, lumbar range/velocity,
   lumbar activation/effort and ground-reaction forces.

5. Is a healthy-gait-only demonstration sufficient before any weakness or
   recovery experiment?

6. Is transfer learning from the existing healthy 22-muscle checkpoint an
   acceptable methodology? The proposal is to preserve the original 22 action
   weights, add six lumbar outputs initialized near off, then fine-tune.

7. What training budget and compute resource are available? A short feasibility
   fine-tune can start around 0.5–2 million steps; a robust set of repetitions
   and reward-tuning experiments is likely several days to weeks on suitable
   CPU/GPU infrastructure.

8. What level of anatomical validation is expected before making claims? The
   present model can support mechanical plausibility checks, but not clinical
   or subject-specific conclusions.

## Recommended immediate decision

Approve or reject **Option A** as the first milestone:

> Demonstrate that a six-muscle, one-hinge lumbar extension can be added to
> the existing healthy 2D walking model; then test whether a fine-tuned
> 28-output policy can use those muscles without degrading healthy gait.

If approved, the next technical tasks are: correct the fixed-lumbar baseline,
define the observation/action expansion and reward safeguards, then run a
small transfer-learning feasibility training before committing to a long run.

## Files available for review

- [`models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml`](models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml)
- [`models/22muscle_2D_lumbar/PHASE4_PHYSICS.md`](models/22muscle_2D_lumbar/PHASE4_PHYSICS.md)
- [`LUMBAR_INTEGRATION_NOTES.md`](LUMBAR_INTEGRATION_NOTES.md)
- [`HEALTHY_28_UNWEIGHTED.md`](HEALTHY_28_UNWEIGHTED.md)
