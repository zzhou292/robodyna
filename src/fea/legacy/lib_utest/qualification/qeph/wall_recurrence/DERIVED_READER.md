# CW1 derived evidence reader

Source is prepared for its first execution. The five test functions below are
synthetic protocol tests, not measurements of the native recurrence or impact.
No numerical policy, writer, raw-reader helper or physical owner is changed.

`ReadWallJobSummary` takes one derived directory, an already authenticated
`RawReadResult`, external expected analysis index/provenance SHA256 values,
and the remaining shared byte allowance. Its complete output is staged.
A malformed archive returns false without changing the caller's result.
A valid closed partial archive returns its authenticated receipt with
`summary_available=false`. Its files retain the partial/nonfinite evidence.
When every named point/context/payload is present, failed numerical points
remain explicit in the compact summary. A failed diagnostic 4H0 does not
become a global job veto. No timestep is selected by this reader.

## Authentication and measured evidence

The caller must bind the raw result through `ReadRawJob` and external hashes,
and authenticate the reviewed analysis producer/source/executable separately.
Self-provided provenance or a matching index cannot establish source trust.
The reader authenticates every exact file, ordered progress inventory, raw
model/native/contact link, analysis provenance link and final receipt.
Missing, extra, renamed and symlinked artifacts are rejected. Each payload is
bounded by 32 MiB; actual bytes must fit the supplied remainder and the saved
producer budget within the existing 96 MiB raw-plus-derived set cap.
The caller debits all six jobs and final selection artifacts from one budget.
The six actual raw/derived jobs currently consume 90,089,309 bytes, leaving
10,573,987 bytes before selection; these observed totals do not replace
per-read byte verification.

The current owning model builder and serializer bind the exact model, native
reference mass/inertia, penalty tuple and dictionary. Present contexts match
the owning fixed D and scalar schedule, including all chronological windows.
Unavailable early-return context stages remain unavailable. Full and weighted
operators are reconstructed with the existing B and D helpers and checked
against their canonical descriptor hashes. Dense operators are kept for one
h point's three amplitudes and released after its local comparisons.

Moving baseline, full-state identities, physical contact derivative checks,
and local matrix/gain comparisons use the existing owning functions. No
native recurrence, Schur decomposition, power composition or Gram eigensolve
is run. The saved eigenvalues, decomposition residuals, Gram norm,
antisymmetry and controlling vectors are authenticated producer measurements.
The reader checks their shapes/finiteness and rederives the original scalar
predicates, spectrum extrema/counts, Gram extrema/mean gain, chronological
state counts and controlling-coordinate argmax. It does not add a vector-norm
or other numerical gate. Explicit nonfinite marker bits are accepted only in
incomplete fields supported by the producer's staged record.

Raw Gram gain remains diagnostic. Complete sequences require both measured
Grams; passing sequences use the weighted gain limit only. The original raw
and weighted spectrum, identity, matrix comparison and weighted gain
comparison predicates remain intact. Saved completion/pass bits must agree
with the rederived values. `CompactWallJobAnalysis` stages the final compatible
`WallJobSummary`; the summary contains no aggregate job-passed field.

## Source and first qualification target

Proposed library `qeph_wall_derived_read` has exactly six translation units:

- `WallDerivedRead.cpp`: bounded file/inventory driver and publication.
- `WallDerivedReadContext.cpp`: header/raw links and exact model context.
- `WallDerivedReadInventory.cpp`: ordered prefix and captured status checks.
- `WallDerivedReadNumerics.cpp`: retained Schur/Gram scalar predicates.
- `WallDerivedReadAmplitude.cpp`: operator reconstruction and component checks.
- `WallDerivedReadChecks.cpp`: contact/local comparisons and dense release.

Public `WallDerivedRead.h` and private `WallDerivedReadFields.h` accompany them.
Link the existing `qeph_wall_job_report`, `qeph_wall_screen_selection` (owning
compact summary implementation), and their
transitive raw-reader/analysis/ArtifactIO dependencies. No CLI is included.
Keep the existing strict floating-point and single-thread Eigen definitions.

Proposed `qeph_wall_derived_read_check` uses `WallDerivedReadTest.cpp` and
`WallDerivedReadTestFixture.cpp`, the reader library and GTest main. Five
functions cover:

1. Full summary, raw gain 80, and failed 4H0 with earlier points available.
2. Closed partial nonfinite evidence and false completed-field rejection.
3. External hashes, byte limits, missing/extra/symlink files and exact retry.
4. Rehashed model/context/operator/window/native-link/prefix corruption.
5. Rehashed spectrum/Gram/contact/comparison corruption.

The explicit free-particle/cache raw operator and synthetic measured spectra
and Grams are labelled test data. They test the reviewed-producer protocol;
they do not claim native shell spectra or physical admission. Corruption
clones rebind dependent hashes so they reach semantic checks. Failure tests
compare complete semantic output snapshots, including all compact fields and
receipt links, rather than object padding.

Root owns CMake registration, source maps and execution. Suggested first run:
one CPU, no CUDA, 1 GiB RSS cap, 60 s limit. Repeated eigensolves/native probes
are deliberately absent from this target.
