# Healthy walking policy with six unweighted lumbar muscles

This is a compatibility baseline, not a newly trained policy.

- The physical model is the 28-muscle healthy model.
- The existing healthy checkpoint still controls the original 22 leg muscles.
- The six lumbar muscles receive exactly zero control.
- The six lumbar activations are hidden from the old policy observation.
- The lumbar joint is held at neutral so the old policy is not expected to
  solve an untrained torso-control problem.
- No original model, configuration or checkpoint is modified.

The lumbar muscles still exist physically and retain their passive muscle and
tendon parameters. "Unweighted" means that the neural policy has no outputs or
learned weights for them; it does not remove their anatomical parameters.
MyoSuite uses normalized actions, so the adapter sends `-1` for each lumbar
action; with a muscle control range of `[0, 1]`, this becomes physical control
`0`. A normalized action of `0` would incorrectly become 50% excitation.
The original 22 leg muscles still use the healthy checkpoint—without those
existing weights, this would not be a walking model.

Because the passive lumbar muscle paths and a soft neutral constraint are now
part of the physics, the trajectory need not be numerically identical to the
original rigid-torso simulation. This launcher is intended as a controlled
compatibility preview, not evidence that lumbar control has been learned.

## Run it

Paste this command without Markdown backticks:

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python .my_venv/bin/mjpython tools/run_healthy_28_unweighted.py
```

The camera follows the pelvis, muscle/tendon visualization is enabled, and the
same gait phase is replayed after termination or 1000 steps.

## Verify without opening the viewer

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python tools/run_healthy_28_unweighted.py --smoke-test
```

The smoke test confirms the 22-to-28 action adapter, the legacy observation
shape, the neutral lock, and zero lumbar controls.
