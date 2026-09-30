# Accepted plastic source-part wall output

Wall material histories extend the existing accepted wall archive, mesh reader,
and Chrono/VSG replay. Elastic output retains its v1 schemas, strings, insertion
order, CSV columns and byte limits. Plastic output selects v2 configuration,
fields and manifest schemas and explicitly declares its selected material mode.
No new dynamics, rendering framework or restart claim is introduced.

`case/SourcePartPlasticFields.cpp` owns material/section serialization. It
records the original46-pair curve, source C/P/VP and the exact selected filter
settings. The curve's semantic SHA256 uses interleaved strain/stress binary64
values in big-endian order. The source-aware material mode retains C=8000/s,
P=8 and the resolved10000/s cutoff; alpha is tied to the actual fixed interval.
The rate-independent mode explicitly records inactive rates. Both current
plastic modes declare the native CZFINTN1 NPT3 stabilization correction.

Each frame adds94 source-parent rows in original source order. A row identifies
its source element/family slot, cumulative point-work diagnostic, per-interval
section diagnostics accepted reported thickness and all three point histories. Point records retain five
material stress components, accumulated equivalent plastic strain and filtered
rate. Actual section mean/minimum tangent and mean/last-point yield are kept
separately: the native stabilization consumes last-point yield. Force evaluation uses the evolving accepted thickness; original native mass is retained. Histories are
read only from the jointly accepted case state, checked against the captured
owner/epoch before and after readback, and associated with the existing accepted
interval attempt/time. Epoch zero has zero point histories and no fabricated
prepared contact candidate.

The reader validates the source curve, section weights, source/family mapping,
material policy/rate tuple, phase and owner/epoch/attempt associations, bounded
point domain and zero startup, reference-volume/native-total-mass association,
per-parent and aggregate summaries, and final-summary/frame equality. It reuses
all existing wall placement, native-mass, contact certificate, interval ledger,
mesh and failure-preservation checks. It does not infer stress or plasticity
from the rendered shape. The viewer labels the selected material policy and
uses the actual accepted geometry at physical scale.

The plastic replay colors each original shell parent by the maximum accumulated
equivalent plastic strain over its three accepted layers. It reads the actual
point histories and joins them to display triangles by source element ID; both
triangles of a Q4 receive one flat color. Colors are never inferred from motion.
The fixed scale is `0 .. max(0.001, accepted final maximum plastic strain)` for
the entire accepted prefix, including the all-zero initial frame. Zero is blue,
the midpoint is yellow and the upper endpoint is red; the visible legend uses
percent. Values at the upper endpoint saturate, and there is no frame autoscale.
The 0.001 floor gives an all-elastic prefix a meaningful 0.1% legend.

`chrono/ReplayParentScalarColors.h` owns the reusable immutable parent-to-face
map and uses Chrono's color interpolation. `AcceptedReplayScene` stages colors
and coordinates together, retaining the same mutable shape, mesh and face-color
indices for the entire replay. A bad parent ID, scalar or vertex preserves the
previous complete display. Its material-free shape selects the existing VSG
dynamic color-buffer path; scientific meshes, physical coordinates and the
placed wall are unchanged. The capture manifest records the quantity, palette,
fixed range and units. Existing scene checks cover Q4/T3 source association,
physical coordinates, stable handles, late failure and exact corrected retry.
No extra CMake target or Chrono modification is needed.

For an actual yielding archive, the existing scene check target can also compile
`viewer/accepted_plastic_replay_check.cpp` and link `robo_dyna_accepted_replay`.
Add this source only when that reader target exists, preserving standalone core
scene checks. Run with `ROBO_DYNA_PLASTIC_REPLAY_FIXTURE=/path/to/accepted-bundle`
and `--gtest_filter=AcceptedReplayScene.ActualPlasticBundle*`. This optional gate
reads recorded point PLA independently for every accepted frame, verifies the
reader's parent values and the actual Chrono face colors, and checks unchanged
physical coordinates, wall and visual handles. It requires nonzero recorded
yielding; without the environment variable the test skips. Root owns this
guarded execution and the separate VSG video check.

Plastic strain and native point-work diagnostics show material irreversibility.
They are not an additional total-energy term. Existing maximum chord change is
rigid-motion invariant; a settled permanent shape additionally requires review
of the post-contact/unloading interval. The writer does not label an arbitrary
still-contacting final frame as a completed rebound or relaxed shape.

The default inventory remains256MiB. Only explicit plastic wall v2 opts into a
1GiB aggregate cap; file size remains32MiB and frame count1000. Plastic fields
are limited to640KiB, meshes40KiB and OBJ16KiB. The existing v1 worst-token bound
is476443 bytes. Plastic adds at most94×1680+4096 bytes: each pretty-printed row
has12 scalar values and3×7 point scalars, all widened to26-character numeric
tokens with indentation and punctuation included. This fits the additional
160KiB field budget. A15ms coarse run at h=2^-24s with1000 saved frames forecasts
about893MiB, below1GiB. The unchanged CSV planner allows8×32MiB segments; longer
or finer ledgers must be planned explicitly rather than bypassing that limit.

Root integration adds `SourcePartPlasticFields.cpp` to the wall artifact target
and `AcceptedReplaySourcePartPlastic.cpp` to the accepted replay target. The
focused executable `source_part_plastic_output_check.cpp` links those targets,
the source material library and GTest; its arguments are the pinned readiness
and canonical wall paths. It checks an actual four-step accepted plastic prefix,
full layer records, startup/late-layer/source-curve/summary corruption, published
reader-state preservation, the actual serialized worst-token cap and explicit
inventory budget failure. Existing elastic output tests use the extracted
shared `test_support::WorstScalarWidth` helper unchanged.

The combined guarded build and 35 selected CTest groups passed in
`crash-work/reports/plastic-integration-tests-1.xml`, including actual four-step
archives for both material modes and corruption rejection. Sustained source-rate
impact and physical-scale video are the next integration evidence. Earlier
frozen elastic byte-regression evidence retains its original checkpoint scope.
