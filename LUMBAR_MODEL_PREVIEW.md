# Running the lumbar models

These commands only preview movement. They do not train or modify a policy.

## Six-muscle donor

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python .my_venv/bin/mjpython tools/preview_lumbar_models.py \
  --model abdomen --mode animate
```

## Full 210-muscle MyoTorso reference

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python .my_venv/bin/mjpython tools/preview_lumbar_models.py \
  --model full --mode animate
```

This model is reference-only and was not merged into the leg model.

## Integrated 28-muscle model

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python .my_venv/bin/mjpython tools/preview_lumbar_models.py \
  --model integrated --mode animate
```

This is the original 22-muscle leg model plus six lumbar muscles.

## Run the non-RL physics checks

```bash
cd "/Users/borb/Documents/MyoSuite Lumbar Flexion/MyoAssist" && \
source .my_venv/bin/activate && \
python tools/validate_lumbar_physics.py
```

This checks the full lumbar range, signed flexion/extension control, tendons,
moment arms, contacts, passive forces, neutral stability and preservation of
the original leg mechanics. It does not load or train an RL policy.

## Controls

- **Space:** pause or resume
- **R:** return to neutral and pause
- **Mouse:** rotate or reposition the torso-following camera
- Use `--mode static` for stationary inspection
- Close the window to exit

For the donor and integrated model, positive angle is extension and negative
angle is flexion. Tendons and actuator-force colours are shown automatically.

## Added muscles

| Actuator | Muscle | Action |
|---|---|---|
| `ercspn_r` | Right erector spinae | Extension |
| `ercspn_l` | Left erector spinae | Extension |
| `intobl_r` | Right internal oblique | Flexion |
| `intobl_l` | Left internal oblique | Flexion |
| `extobl_r` | Right external oblique | Flexion |
| `extobl_l` | Left external oblique | Flexion |
