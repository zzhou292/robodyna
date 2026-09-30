# CW1 one-job derived analysis: prospective contract

This increment consumes one `RawJob` and produces pure derived numerical
evidence. The caller must first authenticate the durable raw bytes, model,
source provenance and exact fixture/boost identity with the owning raw reader.
It does not call native force routines, collect replacement samples, run a
mechanics owner, serialize reports, select a timestep or compare separate
boost jobs. Its author has not built or executed these files. Existing
`ANALYSIS_CONTRACT.md` and the frozen CW1 planning contract retain authority
over all numerical thresholds and full-state semantics.

## Modules and recomputed evidence

`WallJobAnalysis.h` declares one result tree and a synchronous progress
callback. `WallJobAnalysis.cpp` validates one immutable one-/two-cell model,
the selected {-8,0,+8} m/s boost (negative zero is rejected), six exact h values
and three exact amplitudes. It retains all 109/194 coordinates. Every available
complete native matrix receives its moving-baseline check and complete
`AnalyzeWallBranches` evaluation, including both constant branches and nine
switching windows. A failed baseline does not suppress the available failed
spectral, identity or Gram evidence. No legacy `FeedbackIndices` or reduced
free-shell policy is used.

Each adjacent amplitude pair receives both `CompareWallMatrices` on its raw
shell matrices and `CompareWallAnalyses` on its complete derived operators and
gains. The latter checks both full branch matrices and eleven sequence gains.
Only weighted mean-gain differences control gain consistency; raw differences
remain explicit diagnostics. The stored raw collection flag and saved
numerical pass bits cannot turn missing operands into a passing analysis.
Collection flags for individual matrices/baselines and sample counts describe
which observations actually exist; they are checked with dimensions and finite
values, rather than treated as scientific verdicts.

`WallJobContactChecks.cpp` checks each inactive/active branch against the
finest retained native shell matrix at its h. It rebuilds the exact owning
kick and full A_shell*B operator, rebuilds every frozen n+7 sign-cone direction,
checks the complete recorded physical sample identities/certificates/masks,
and recomputes both one-sided directional quotients from the recorded
baseline and three sample states. Every baseline is at touching and therefore
has the owning active diagnostic mask, even for an inactive directional
branch. Perturbed masks must match the selected cone. The moving baseline is
checked independently against the complete normalized expected state.

The small `ContactDerivativeChecks.h` extraction shares the retained probe's
quotient grouping and directed comparison body verbatim. The native collection
path in `ContactBranchProbe.cpp` now calls that same helper; its sample order,
native operations and numerical budgets are unchanged. Extraction was released
only after the immutable `qeph-wall-raw-1` checkpoint / TL `842469b` passed
7 new raw functions and 31 prerequisite regressions. Both quotients and each
residual/budget remain visible on a valid failed comparison. Saved quotients,
saved budgets and saved pass flags are never used to grant the new verdict.
Physical sample validation is not another evaluation of the mechanics law.

The raw `MovingMatrixProbe` contains its baseline and centered derivative
columns, not the two native states used to form each column. Consequently
this increment recomputes native baseline, matrix-amplitude, full identity,
spectral and gain checks, but cannot redo centered differentiation from absent
states. The contact-native directional probes do retain actual states, so
their derivatives are recomputed. It does not claim to authenticate an
arbitrary in-memory matrix: raw source/byte authentication remains required.

## Completion, failure and progress

Each record distinguishes evaluable/complete from numerically passed. Expected
numerical rejection returns its available intermediates and a diagnostic;
later independent amplitude, branch and step checks continue. A global
model/grid mismatch emits only `Finished` and an invalid result. An incomplete
job retains all available component evidence and cannot pass. No input record
is modified and no existing output object is incrementally overwritten.

The whole-job `complete` and `passed` fields aggregate all six h values,
including the diagnostic 4H0 point. They are reporting summaries, not an
admission precondition for selecting H0 or H0/2. A later six-job coordinator
must aggregate the individual step verdicts and cross-boost comparisons and
then call `SelectWallScreenStep`; it must not first require every `job.passed`.
A retained 4H0 failure can coexist with a valid smaller-step selection.

For each h the callback order is three `Amplitude` events, two `Contact`
events, then one `Step`; one final `Finished` event follows. Thus a fully
traversed job emits 37 events, including slots with unavailable observations.
All result grid identities are initialized before the first callback. The
callback receives a borrowed const result valid for that call; it may persist
the just-completed component and throw to interrupt further analysis. No
exception is swallowed and no later callback occurs after interruption.
The orchestration does not retain caller pointers or provide automatic resume.

## Fixed numerical and resource bounds

Thresholds remain those of the owning helpers: normalized identities and
directional/matrix comparisons 5e-8, Schur/Gram decomposition checks 1e-10,
weighted mean gain <=64, and weighted gain consistency
.005*max(gains)+1e-10. Directed comparison budgets/residuals keep their original
downward/upward arithmetic. The extracted long-double quotient expression is
unchanged from raw collection; it is not a new portable interval certificate.
No timestep, epsilon, boost, metric or threshold is fitted to observed data.

One complete job has 18 branch analyses: 72 explicit RealSchur decompositions,
72 EigenSolver extractions and 396 endpoint-inclusive Gram SelfAdjoint
eigensolves, each dimension at most 194. Each sequence has at most 32768
ordinary transitions represented through bounded binary PowerGram composition,
not a full native trajectory. There are twelve contact rechecks, with 11 or
13 directions and two quotients each. This slice performs zero native cell
intervals and zero GPU operations.

The prospective retained numeric payload is below 32 MiB per job: 72 full or
weighted 194-square matrices consume 21,678,336 B, scalar traces have at most
193,554 entries (24-byte host layout would consume 4,645,296 B), and spectra,
controlling directions, baselines and quotients fit within a further 3 MiB.
Container/string overhead, Eigen workspace, raw input storage, allocator peak
and actual host ABI are not claimed measured by this source forecast. Root's
proposed first one-job guard is one CPU, 1 GiB RSS and 120 seconds; runtime is
unmeasured. A resource failure requires retained evidence and explicit review,
not a numerical tolerance change.

The existing **96 MiB output budget covers all six raw plus derived jobs**.
This in-memory payload forecast is not permission to serialize all matrices
repeatedly across 108 analyses. A later derived writer must bind exact raw
artifact hashes and explicit reproducible B/D recipes plus derived matrix
hashes, while retaining full spectra, controlling Gram directions, metrics,
schedules and failure evidence. That later serialization must share the
existing <=32 MiB/file and aggregate 96 MiB checks. No report or reader is
introduced here.

## Focused gate and registration handoff

Eight new host functions are staged: four in `WallJobContactCheckTest.cpp`
and four in `WallJobAnalysisTest.cpp`. They collect no native Jacobian grid.
The contact tests use actual qualified host point-law records with an explicit
synthetic linear full-state oracle, for both dimensions and branches. They
test fresh derivative checks despite false saved flags, tampered samples behind
forged passing checks, late sign/identity/direction/operator/nonfinite failures,
clean retry, and cancellation/nonfinite quotient evidence. The independent
linear derivative assertion uses 2e-12; it does not replace any screen budget.

Coordinator tests check all 37 event slots for missing data, exact frozen grid
and native observation boundaries, three synthetic mutually identical operators
that remain failed under actual observer/cache checks, fresh local comparisons
and contact verdicts, retained spectra/Gram directions after a bad baseline,
and interruption after the first fully analyzed amplitude. The complete
synthetic identity is deliberately not a native shell derivative; no test
relabels its failed observer check as physical success. There are four full
109-coordinate `AnalyzeWallBranches` calls across these orchestration tests,
not eighteen native matrices per synthetic fixture.

Root should register `WallJobAnalysis.cpp` and `WallJobContactChecks.cpp` with
the existing wall analysis/model targets, and the two new test translation
units with GTest. The new shared derivative header is a dependency of the
existing `ContactBranchProbe.cpp` target. Existing C++17 / Eigen and
`-fno-fast-math -ffp-contract=off` flags remain unchanged. Root owns the source
map, build, focused test execution and all affected model/raw/analysis
regressions. Actual boost comparisons and aggregated step selection remain a
separate subsequent evidence stage.
