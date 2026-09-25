# Native trajectory expected-data converter

`trajectory.py` converts authenticated read-only native observations into the existing
`T25REF01` qualification binary. It does not supply production inputs, restart state,
coefficients, or a second physical clock. The public CLI deliberately admits only the
current v2 all-shell, single-interface/single-worker, 18-node/18-row, 1,000-interval
capture. It emits 1,001 force-base frames in native mm/tonne/s. No interpolation,
rounding, tolerance comparison or SI conversion occurs in this serializer.

`trajectory-inputs.json` binds raw startup/observation/sequence records, completion
summaries, declared source/decks, ABI manifests, original native sources, observer
modules and the unchanged TL serializer/reader contract by exact SHA-256 and extent.
The declared node order is joined through each actual ITAB; the observed NSV is
translated through Engine ITAB, not mistaken for an external node ID. Native row and
geometry cohort order remain unchanged. The observation supplied one classification
NSV, not per-cycle NSV snapshots; fixed single-interface row ownership remains the
explicit scope, with stable ITAB and complete row extents checked every cycle.

Activity means positive pre-FOR3 PENE at a force-base epoch in [0, 1000). Episodes
are zero-to-nonzero transitions of after-MAINF IRTLM(1), in secondary row order.
Frame 1000 is terminal motion and is excluded from both activity counters. TT is
checked against repeated original binary64 dt addition; DT1 and the initial half
kick DT12 are checked separately. Mid-cycle DT2 estimators are not accepted steps.
Native MS/IN are mapped from startup ITAB; all later observed MS values must match.

From the app checkout, with the workspace root supplied explicitly:

```sh
TYPE25_REFERENCE_WORKSPACE=/home/jsonzhou/Desktop/chrono-work python3 -B -m unittest benchmarks.native_contact_scene.reference.test_trajectory -v
python3 -B -m benchmarks.native_contact_scene.reference.trajectory --workspace /home/jsonzhou/Desktop/chrono-work --manifest benchmarks/native_contact_scene/reference/trajectory-inputs.json --output /home/jsonzhou/Desktop/chrono-work/crash-work/investigations/native-moving-contact-scene-reference-1
```

Run these under the workspace's bounded helper/author lock. Output creation is
exclusive; `--check` regenerates and compares both files without overwriting them.
The expected-data directory is passed to the existing C++ qualification using
`-DTYPE25_MOVING_NATIVE_REFERENCE_DIR=...`. Only that coupled native/GPU trajectory
gate establishes physical agreement; successful serialization is not a solver or
performance claim. Debugger capture timings are not benchmark measurements.
