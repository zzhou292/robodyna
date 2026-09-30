# CW1 one-job derived evidence

Source preparation only. Four focused host report functions are staged and
unexecuted. The owning raw reader and job analysis are separate dependencies;
this writer changes no model, numerical threshold, source map or build file.

`WallJobReportWriter` borrows an immutable authenticated `RawReadResult` and
`RawReadBinding`. The result must outlive it. The constructor checks the exact
fixture/boost, closed receipt, raw index/provenance hashes, a new directory,
bounded exact analysis-provenance JSON and an explicit remaining set allowance.
The launcher separately authenticates source/build/executable provenance. A
self-provided hash or receipt cannot establish that trust by itself.

The writer consumes `WallJobProgress` callbacks from `AnalyzeWallRawJob`:

- `provenance.json` retains the exact analysis provenance bytes once.
- `context-hN.json` retains the full fixed metric and scalar schedule/trace once
  per h. The first actual input-complete amplitude supplies it. Earlier missing
  input keeps context absence explicit; later actual contexts must match the
  exact stored context-data hash, and cannot replace that file.
- `amplitude-hN-aN.json` retains moving-baseline, identity and complete raw/weighted
  Schur and Gram diagnostics, every eigenvalue and controlling direction, flags,
  budgets and failures. It binds exact raw native and context file hashes.
- `contact-hN-bN.json` retains independently recomputed quotients, errors/budgets
  and failures, binding the exact raw branch and finest native file hashes.
- `step-hN.json` retains both local amplitude comparisons and hashes of the prior
  amplitude/contact derived records. Step verdicts remain visible individually.
- A new `progress-NNN.json` follows every context or ordinary callback payload;
  `index.json` closes the ordered inventory, possibly with failed/incomplete
  analysis. A job with all six contexts and all callbacks writes 87 files.

Every payload binds the external raw index and raw provenance, raw model, and
exact analysis provenance. Each inventory lists all earlier files' names/bytes/
hashes, including earlier inventories, excluding itself. Returned receipts also
include the current inventory. The final aggregate `analysis_passed` includes
the diagnostic 4H0 point and is not a six-job timestep-selection verdict.

Dense derived operators are not duplicated on disk. Each descriptor retains
dimensions, presence/finiteness and SHA256 over the exact compact JSON from
`EncodeRawJson(raw_detail::Matrix(matrix,true))`. The owning reconstruction is
`BuildContactBranch(model,h,branch,retained_native_shell)` followed by
`ApplyWallStateMetric(full,D)`. The report states the exact coefficient precision
and arithmetic order; the raw model retains k/m and dictionary scales, and the
context retains D. Reconstructed descriptors must match before downstream use.
No mode is deleted. The full spectra and Gram eigenvalues/directions remain
measured evidence under the reviewed-producer trust boundary, without implying
a repeated eigensolve, thermodynamic energy or physical CFL theorem.

Failed/partial nonfinite values use the existing explicit marker kind plus
binary64 bits. Completed components require finite values and correct dimensions.
Later serialization failures retain earlier files and never create a completed
index. Publication is create-only per file, not atomic across a callback or
multiple files: context/payload writing can succeed before inventory writing
fails. The caller must stop after such an exception. There is no IO retry/resume
promise. The receipt lists successfully closed files; a failing partial file or
constructor failure may require a launcher directory inventory for byte accounting.

All six raw jobs measured 47,302,880 bytes, leaving 53,360,416 bytes within the
shared 96 MiB raw+derived cap before any derived or final selection report. This
is a shared allowance, not a per-writer entitlement. Root supplies a decreasing
remaining budget while reserving space for final selection, and debits every
actual retained file, including failed partial jobs. Every file remains bounded
by 32 MiB. Numerical or resource failure cannot loosen any scientific gate.

Proposed root targets: `qeph_wall_job_report` from `WallJobReport.cpp`,
`WallJobReportWriter.cpp`, `WallJobReportInventory.cpp`, `WallJobJsonNumerics.cpp`,
`WallJobJsonContext.cpp`, `WallJobJsonAmplitude.cpp`, `WallJobJsonChecks.cpp`;
link the owning job-analysis target and `qeph_wall_raw_read`. The four tests in
`WallJobReportTest.cpp` use `WallJobReportTestFixture.h`, GTest main and the report
library as `qeph_wall_job_report_check`. They exercise delayed/shared context,
exact matrix/provenance links, finite/failed diagnostics, late serialization and
IO/budget failures, mismatched identities and false-final rejection. Protocol
operators are synthetic and explicitly unqualified; no native interval maps or
eigensolves are run by these tests. Preserve strict CXX and single-thread Eigen
flags; proposed bound is one CPU, 1 GiB, 60 seconds, no CUDA, serialized by root.

The future derived reader must authenticate inventories, reconstruct matrix
hashes, validate exact context/model identities and rederive pass predicates from
the retained measurements. Cross-boost comparisons and the frozen six-job
selector remain separate. No startup, nonlinear impact, vehicle or rendering
admission follows from this writer.
