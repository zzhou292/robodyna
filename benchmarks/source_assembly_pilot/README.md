# Source assembly timestep pilots

This CPU comparison library and CLI report the response of the complete original
six-part component at common accepted position times. They reuse `AcceptedReplay`
for every archive/source/material/group/released-boundary/phase/ledger check,
then read fields through the retained hash-and-size inventory. No solver, CUDA
runtime, new force evaluation, state owner or energy-admission policy is linked.

The reference must have the finest step. A candidate step must be an exact
integer multiple of the reference's binary64 rational value, with multiplier
at most 2^20. Significand/exponent integer arithmetic rejects rounded false
matches such as `0.1 * 3`; checked integer epoch products identify common times.
The original accumulated timestamps are also checked using the owning reader's
rounding policy and are retained in the report. There is no interpolation.

Completed horizons and explicit accepted prefixes are both supported. Only the
intersection of saved epochs is compared. Each pair reports the shared accepted
horizon, last compared time, unmatched terminal flags, and whether any positive
common time exists. An initial-only match is explicitly incomplete response
coverage. Each run retains its own stop reason, actual terminal values and the
first positive contact/PLA row's original CSV base/endpoint timestamps.

The report has raw SI units, per-frame maximum/RMS differences, reference
magnitudes, worst source IDs, and maximum differences over the saved common
samples. Node position reference magnitudes are absolute world-coordinate
norms, **not displacement normalization**. Quaternion distance treats q and -q
as the same orientation and remains sensitive to small rotations. Layer flat
indices are `3*source_parent_index + bottom/middle/top layer`. Stress channels
compare corresponding native corotational XX/YY/XY/YZ/ZX components in each
run's current source-shell axes; they are not world-tensor stress differences.
Contact differences include nominal values and separation between the recorded
integration certificate intervals; their errors are not physical uncertainty.

Native cumulative shell work, cumulative plastic work and QEPH viscous work are
separate diagnostics. Carried midpoint v/omega/K, lagged-frame kinetic values,
interval work increments, filtered rates and surface power are excluded from
endpoint comparison. The optional force-stage archive is validated by the
reader and otherwise left separate. No reaction work is called dissipation,
no total energy is reconstructed, and no convergence verdict or numerical
acceptance threshold is emitted. Exit0 means the report was produced.

Only run/output IDs, timestep/step-count/cadence/forecast/ledger declarations,
resource ceilings and the optional diagnostic switch are removed for physical
configuration matching. Requested horizon, complete input/native M/J, source
tables, groups, released frontier, wall and deformation declarations must agree.
Original wall/placement/mesh bytes must also match. Archive files must remain
unchanged while read. The CLI requires an absent output path, writes only after
the full comparison succeeds, and limits its report to16MiB; shared `ArtifactIO`
create-only writers assume externally serialized filesystem ownership.

## Immediate pilot plan

1. Retain h=2^-26s,1024 intervals as the15.2587890625us reference. Run8h×128
   and4h×256 with the same source/wall/limits and save every2/4 intervals,
   respectively. Their65 physical endpoints match the reference's16h cadence.
2. Inspect all three pairwise response histories, first-contact and first-PLA
   records, yielded counts and source IDs of largest discrepancies. A closer4h
   response is useful refinement evidence, not an automatic convergence claim.
   Explicitly examine contact onset and native layer stresses where a small
   geometry change can cause an active-set or material-history difference.
3. If a run rejects, use its valid accepted prefix and exact parent/guard report;
   do not extend it by interpolation or relax its guards to fill the horizon.
   If response discrepancies remain unresolved, add2h at the same horizon before
   selecting a step for a longer deformation pilot. Longer-run admissibility
   remains a separate gate; these microseconds are not the full crash horizon.

Build the standalone CMake source in this directory with `Chrono_DIR`, explicit
`ROBO_DYNA_TL_ROOT`, and optionally `ROBO_DYNA_PILOT_REFERENCE` pointing to the
fixed1024/65frame actual reference. The owning include is
`SourceAssemblyPilot.cmake`; library `robo_dyna_source_assembly_pilot` and CLI
`robo_dyna_source_assembly_pilot_compare` can join an existing accepted-reader
build without another owner of reader objects.

```sh
robo_dyna_source_assembly_pilot_compare \
  crash-work/runs/source-assembly-wall-integration-1 \
  crash-work/runs/source-assembly-pilot-8h-short-1 \
  crash-work/runs/source-assembly-pilot-4h-short-1 \
  NEW_REPORT.json
```

Six host functions cover rational-time false matches/overflow/prefix joins,
quaternion and scalar arithmetic, physical configuration preservation, the
actual public65-frame self-comparison, source-layer attribution and deliberate
midpoint exclusion, and a late rehashed phase defect/unhashed truncation through
the real reader. Author compilation plus tests used one CPU/512MiB; all six
pass,3.986s runtime,357416KiB peak child RSS including compilation. The unchanged
qualified reader libraries were reused; this is not a fresh owning CMake build
or new dynamics evidence. XML: `/tmp/assembly-pilot-host-ilkg1gvl/tests.xml`.

The fresh owning CMake gate also passes all six functions
(`assembly-pilot-compare-tests-1`,3.67s). Its first build exposed a missing public
neutral-model include dependency; `SourceAssemblyPilot.cmake` now explicitly
links `robo_dyna_source_assembly`, without adding a solver/runtime dependency.
The failed build report is retained. The actual short three-run comparison and
the0.122ms4h/8h comparison are recorded in
`source-assembly-short-pilot-comparison-1.json` and
`source-assembly-medium-pilot-comparison-1.json` under workspace reports.

## Longer accepted-prefix response checkpoint

The 8h/4h runs request0.9765625ms with identical source physics and existing
numerical guards. Both stop at the total nodal orientation envelope: accepted
8h epoch4304/time0.5130767822ms,4h epoch8606/time0.5129575729ms. Each complete
archive contains35 accepted frames; neither completes its requested horizon.
The owning CLI reports34 exact common samples through0.5035400391ms in
`source-assembly-long-pilot-comparison-1.json`, with unmatched terminal flags.
Maximum x difference10.219um, q distance0.0014364rad, point PLA3.79225e-5,
and cumulative plastic-work difference0.001169586J are response diagnostics.
There is one saved node-active-set mismatch; matching earlier active sets did
not prove matching continuous contact events. No interpolation or convergence
admission is performed.

Independent source audit confirms nodal q is an endpoint field for ordinary and
rigid-member nodes: both drift by full h using the newly kicked midpoint spin
before the common accepted slab publication. Group principal axes alone retain
their separately declared lagged phase. The existing64-step native group owner
test independently checks this member-quaternion recurrence.

The limiting source node2181592 is ordinary and belongs to QEPH parents2214871
and2214872. Native geometric reconstruction indicates a localized drilling-like
spin; a1rad total quaternion angle is not itself a1rad geometrical bend. The
workspace native rotation qualification plan owns the next mechanics gate.
Do not widen guards, zero spin, or add guessed stiffness from this observation.
