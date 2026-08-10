# Lumbar model preview guide

These viewers are kinematic inspections only. They do not load an RL policy,
train a policy, or change any model file. The animated sweep repeats until you
close the MuJoCo window.

## Launch commands

### 1. Six-muscle abdomen donor

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python .my_venv/bin/mjpython tools/preview_lumbar_models.py \
  --model abdomen \
  --mode animate
```

This is the source of the six transplanted muscles. It uses the
`lumbar_extension` coordinate.

### 2. Full 210-muscle MyoTorso reference

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python .my_venv/bin/mjpython tools/preview_lumbar_models.py \
  --model full \
  --mode animate
```

This model is preview-only and was not merged into the leg model. It uses the
`flex_extension` coordinate and its existing segmental equality mappings.

### 3. Integrated 28-muscle model

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python .my_venv/bin/mjpython tools/preview_lumbar_models.py \
  --model integrated \
  --mode animate
```

This opens the new model at
`models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml`. It contains the unchanged
22 leg muscles plus the six lumbar muscles and uses `lumbar_extension`.

## Viewer controls

- Drag in the window to rotate or reposition the camera; it tracks the torso.
- Press **Space** to pause or resume the slow repeating sweep.
- Press **R** to return to neutral and pause.
- Close the viewer window to exit cleanly.
- Change `--mode animate` to `--mode static` for a stationary inspection.
- Add `--angle VALUE` in static mode to select a pose in radians. The integrated
  and six-muscle models allow -0.8727 to 0.2618 rad. Positive is extension;
  negative is flexion under the donor convention.

Tendons, tendon wrapping geometry, and actuator-force colours are enabled by
the script. Dark or changing tendon colours are MuJoCo's actuator-force
visualization, not missing muscle geometry.

## Six-muscle index

| Actuator | Side | Donor muscle | Primary sagittal action |
|---|---|---|---|
| `ercspn_r` | Right | Erector spinae | Lumbar extension |
| `ercspn_l` | Left | Erector spinae | Lumbar extension |
| `intobl_r` | Right | Internal oblique | Lumbar flexion |
| `intobl_l` | Left | Internal oblique | Lumbar flexion |
| `extobl_r` | Right | External oblique | Lumbar flexion |
| `extobl_l` | Left | External oblique | Lumbar flexion |

The detailed mechanical results are in
`models/22muscle_2D_lumbar/VALIDATION.md`.
