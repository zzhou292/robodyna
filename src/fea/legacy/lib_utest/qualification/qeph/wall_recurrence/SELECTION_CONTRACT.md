# CW1 bounded boost comparison and six-job selection

This source increment is limited to pure numerical comparison/aggregation.
It performs no native map, Schur or Gram evaluation, dynamics admission, file
IO, source authentication or report publication. Its author has not compiled
or executed it. The physical experiment, source/native maps, full-state
metric, scalar chronology and numerical thresholds remain those already
declared in `ANALYSIS_CONTRACT.md` and `JOB_ANALYSIS.md`.

## Producer/reader boundary

Root authenticates the reviewed producer sources/executable and expected raw
and derived inventory hashes from the qualified run. The raw reader validates
exact inventories, model/tuple, shapes and source/hash chains. The future
derived reader must reconstruct named B/D operators and check their canonical
matrix descriptors, validate numeric field consistency and rederive local
per-step verdicts from the authenticated measurements. It must not copy
arbitrary saved `passed` flags into a purported validated result.

Recorded Schur residuals/eigenvalues and Gram spectra remain evidence of that
authenticated reviewed producer. Consuming this decision does not repeat all
108 full branch analyses or invoke a solver. A separately requested numerical
recompute audit is possible but is not part of this admission boundary.

`WallJobSummary` is a compact numerical DTO, not a self-authenticating receipt.
Its inputs must come from a freshly computed `WallJobAnalysis` or the owning
derived reader's validated records. `CompactWallJobAnalysis` stages this
projection, checking exact named raw identity and h/metric/window context,
complete observation extents and measurement consistency. It does not replace
the producer/reader validation of full spectra or local physical samples.
The summary omits the global job verdict entirely. Root's outer coordinator
must bind each summary and comparison to its exact expected artifacts.

## Small owning modules and memory lifetime

`WallBoostComparison` holds a private, initialize-once zero-boost reference:
one immutable named model, eighteen zero raw native matrices/baselines and
compact per-step context/sequence-gain summaries. No contact sample arrays,
dense derived operators, repeated scalar traces or histories are copied into
that reference. Preparation validates every partial matrix extent before
copying and stages all output; failed preparation preserves the old reference.
There is no mass averaging, normalization, altered dictionary or reduced map.

Process one fixture at a time, in order zero, -8, +8 m/s. After loading and
validating zero, prepare its reference and release the zero RawJob/full derived
tree. Read and compare one boost job at a time, retain its small comparison
receipt, then release it. Drop the reference before processing the other
fixture. The reference's maximum dense native payload is
18*194*194*8=5,419,584 B; its bounded baselines/context/gains keep the prospective
numeric retained payload below 8 MiB. Actual ABI/allocator/RSS measurements
remain root's responsibility. The existing one-CPU/1-GiB guard remains the
proposed execution envelope; no per-step mechanics allocations are introduced.

For each of six h and three amplitudes, `CompareWallBoost` independently reruns
`CheckWallMovingBaseline` on zero and the selected boost. Their two existing
absolute 5e-8 checks jointly control the baseline verdict. The actual baseline
difference minus the expected hV/V lift is retained as a diagnostic maximum
and controlling coordinate. It has no invented stricter pair threshold:
subtracting two individually allowed residuals could reach twice the original
budget. A nonfinite diagnostic is retained as incomplete evidence.

It compares raw native matrices with `CompareWallMatrices`, reconstructs both
complete A_shell*B operators transiently with the owning kick and compares
those matrices, then compares the eleven raw and weighted mean gains with
`CompareWallGains`. Full 109/194 coordinates, model, D, h and all nine windows
must agree. Raw gain differences are measured but cannot veto the weighted
gain decision. There is no fabricated partly-filled `WallBranchAnalysis` and
no change to existing comparison helpers. Missing or numerically failed points
retain available comparison results and do not silently become passing data.

Across both fixtures and both signed boosts there are 72 amplitude pairs,
216 raw/branch matrix comparisons and 1,584 scalar gain comparisons. These are
bounded reconstruction/comparison operations, not new native derivatives or
spectral solves. Ordinary unit tests use explicit synthetic identity matrices
and declared scalar summaries; they are not source-shell admission evidence.

## Exact tuple inventory and step selection

`WallScreenSelection` requires exactly six unique job tuples
(1,0),(1,-8),(1,+8),(2,0),(2,-8),(2,+8) and four unique comparison tuples
(1,-8),(1,+8),(2,-8),(2,+8), in any input order. It checks dimensions, six exact
h values and numerical comparison consistency. Both signs are required for
each fixture. It rejects duplicate/missing/contradictory tuples with no selected
step. Supplied comparison `passed` fields must agree with retained differences
and budgets; an arbitrary flag cannot override a failed comparison number.

At each h it records all six local job verdicts and four fresh boost verdicts,
ANDs those ten contributors, and calls the unchanged `SelectWallScreenStep`.
H0 is selected only when every point through 2H0 passes; otherwise H0/2 requires
every point through H0. Any finer required failure rejects. A 4H0 failure is
retained as a diagnostic and cannot veto either valid smaller selection.
No global `WallJobAnalysis::passed` precondition exists. Passing this screen
still admits no CW2 impact trajectory or other contact/mixed/T3 experiment.

## Derived report and budget handoff

The outer report coordinator must bind all six exact raw-index/provenance
hashes, all six derived-index/analysis-provenance hashes, the reviewed producer
and selector executable/source identities, and every input receipt byte total.
Matrix comparisons must identify both native amplitude payload hashes and
their reproducible B/D contexts, rather than duplicate dense matrices.
Comparison reports retain both baseline-check maxima, the lift diagnostic,
all matrix/gain differences and budgets, and every per-step contributor.
The final selection must reference those complete inventories, record the
frozen selector identity and retain all six aggregate points including 4H0.

There are 53,360,416 B of the shared 96-MiB report budget remaining after the
root-retained raw set. Proposed reservation: at most 1 MiB collectively for
two fixture comparison payloads, selection/provenance and their inventories,
leaving 52,311,840 B for derived reports. This reservation is a proposal for
root's writer integration, not a cap bypass or runtime size claim. The existing
32-MiB/file ceiling and exact aggregate byte checks remain authoritative.
Create-only publication, final-index presence and hash/tuple validation stay
in the existing report/reader layer; no new generic runner is introduced here.

## Focused source gate

Seven new host functions are staged: four in `WallBoostComparisonTest.cpp`
and three in `WallScreenSelectionTest.cpp`. They use the real immutable model,
known normalized +/-8 lift and explicit synthetic matrices/scalar gain inputs.
Coverage includes both dimensions/signs, native plus both branch comparisons,
late context failure/output preservation/retry, wrong units/model/h/metric/
matrix/gain, harmless raw gain differences, compact partial evidence, exact
tuple permutation/duplicates/missing signs, contradictory comparison flags,
factor-two step selection and a 4H0 diagnostic failure that does not veto a
smaller valid selection. They run no
native Jacobian grid or full branch analysis.

Root registers the two new numerical .cpp files and two test translation units
against the existing wall model/job-analysis/Eigen/GTest targets. Existing
C++17 and `-fno-fast-math -ffp-contract=off` remain unchanged. No shared build,
source-map, existing qualification helper or native source is edited here.
