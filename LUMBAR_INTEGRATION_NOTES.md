# Lumbar integration summary

## Result

`models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml` is a new model containing:

- the original 22 lower-limb muscles;
- right and left erector spinae;
- right and left internal oblique;
- right and left external oblique.

The original 22-muscle model and MyoTorso donor files were not overwritten.
No RL policy, reward or checkpoint was changed.

## Work completed

The six muscles already existed in MyoSuite's `myotorso_abdomen` model. The
new work was integrating them into the walking model:

1. Added one `lumbar_extension` hinge between the pelvis and torso.
2. Copied the 12 donor attachment sites into the matching pelvis and torso
   frames.
3. Added the donor wrapping cylinder used by the erector-spinae paths.
4. Copied all six tendon paths and muscle actuator parameters.
5. Retained the walking model's original torso mass, inertia, limbs and 22
   lower-limb muscles.
6. Added a neutral lumbar coordinate to the existing keyframes.
7. Added preview, build and validation tools.
8. Added a disabled-by-default `lumbar_neutral_lock` used only by the physics
   validator to compare the legs against the original rigid-torso model.

The donor and target torso origins already matched, so the attachment sites
did not require manual scaling or approximate repositioning.

## Validation

The integrated model compiles as `nq=54`, `nv=54`, `nu=28`, `na=28`.
Automated checks confirmed that:

- the original bodies, joints, geometry, keyframes and 22 actuators are
  preserved;
- all six tendon lengths remain inside their donor muscle ranges;
- erector spinae produce extension torque;
- both oblique groups produce flexion torque;
- no moment-arm reversals, NaNs or joint explosions occur during a 201-pose
  lumbar sweep.

See `models/22muscle_2D_lumbar/VALIDATION.md` for measurements.

The expanded Phase 4 suite also verifies contacts, passive lumbar torque,
short-horizon neutral-lumbar dynamics and zero-control standing. Both the
original and integrated models fall without a controller, so this phase does
not claim autonomous standing or walking. See
`models/22muscle_2D_lumbar/PHASE4_PHYSICS.md`.

## Anatomical accuracy

The model is **mechanically faithful to the six-muscle MyoTorso donor**, but it
is only **anatomically inspired**, not a clinically complete lumbar spine.

Main limitations:

- one hinge and one rigid torso replace the individual vertebrae and discs;
- only sagittal flexion-extension is enabled;
- multifidus, quadratus lumborum, ligaments and abdominal pressure are absent;
- six grouped paths simplify real muscle architecture;
- strength and motion have not been validated against subject-specific EMG,
  motion capture or spinal-load measurements.

It is suitable for testing simplified active lumbar control and preparing
future RL integration. It should not yet be used for clinical conclusions or
vertebra-level biomechanical predictions.
