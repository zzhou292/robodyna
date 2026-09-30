# CW1 six-job selection command and retained evidence

Source preparation only; four report tests are staged and unexecuted by their
author. This adds command/report composition around the qualified raw reader,
derived reader, boost comparisons and selector. It adds no native, Schur or
Gram evaluation, mechanics owner, timestep formula or physical admission.

```
qeph_wall_select JOB_SET_CONFIG_JSON EXPECTED_CONFIG_SHA256 NEW_DIRECTORY REMAINING_SET_BYTES
```

The exact configuration bytes (at most 64 KiB) must match the externally supplied
lowercase SHA256. The shared command preflight verifies the mandatory
`executable:{path,sha256}` against the actual running `/proc/self/exe` image.
The root launcher authenticates the opaque source/build/producer bindings;
self-provided hashes do not establish that trust. Exact configuration bytes
become `provenance.json`, without rewriting their fields or paths.

`jobs` contains exactly six objects, each with exactly `raw` and `derived`:

- `raw` is the existing analysis command's six-field object: `directory`,
  `cells`, `normal_velocity_m_s`, `index_sha256`, `provenance_sha256`, `bytes`.
  Cells are unsigned JSON integers 1/2; velocity is explicitly the JSON double
  -8.0, 0.0 or 8.0, rejecting integer representations and negative zero.
- `derived` has exactly `directory`, `index_sha256`, `provenance_sha256`,
  `bytes`. Its two hashes bind the analysis index and analysis provenance.
- Every byte count is a positive bounded JSON unsigned integer, later required
  to equal its reader's actual complete receipt. All hashes are lowercase
  64-digit hexadecimal. Directories must be real directories, not symlinks.

All six unique tuples, metadata, paths and the declared aggregate byte count
are checked before creating output. Input order is arbitrary; execution order
is (1,0), (1,-8), (1,+8), then (2,0), (2,-8), (2,+8). Output cannot be nested in
an input archive, because doing so would alter its closed inventory before read.

Only one raw/derived job tree is read at a time. The zero job prepares the
existing immutable eighteen-matrix reference, then its raw tree is released.
One signed boost is compared and released at a time. The zero reference is
destroyed before starting the second fixture. Retained data are six compact
summaries/receipts and four small numerical comparison results; no dense
derived operator is duplicated in memory or the new output.

The report writes exact provenance and an initial inventory, one progress
inventory after every authenticated job, and four named boost payloads with
an inventory after each. Each boost payload retains both complete moving
baseline vectors, the diagnostic lift, all three matrix differences/budgets,
and all eleven raw plus eleven weighted gain differences/budgets at every
h/amplitude. Exact zero/boost native, derived-amplitude and context hashes bind
the operands and owning B/D reconstruction. Failed/nonfinite partial evidence
uses the existing explicit marker encoding. Complete/pass parent flags require
their child evidence; raw gain passing remains diagnostic.

`selection.json` records all six job and four boost contributors at each h and
the unchanged factor-two-margin result. `index.json` closes the inventory and
binds all twelve raw/derived indices, provenance hashes and exact byte totals.
A full report contains 18 files, including prior progress inventories. A valid
closed partial input retains its receipt, unavailable comparisons and all known
contributors, but cannot produce a selected step. Complete failed numerical
points, including diagnostic 4H0, retain the selector's existing meaning.

The hard report cap is **2 MiB**, further limited by the caller's remaining
allowance and 96 MiB minus all actual input bytes. The root measured remaining
set space is 10,573,987 bytes before selection; the new reservation replaces the
earlier prospective 1 MiB estimate. Every output, including provenance and all
inventories, is debited. No file may exceed the existing 32 MiB ceiling.
IO is externally serialized and create-only per file, not atomic across files
or callbacks. Callers stop on exceptions and inventory any partially written
file not present in the successful receipt. Earlier evidence survives.

Exit 0 means a valid complete selection decision, including scientific rejection
with selected_h=0. Exit 2 means a selectable summary/comparison set is unavailable.
Exit 1 means protocol, image binding or IO failure. No output admits CW2 dynamics,
a vehicle run, or rendering. The derived reader validates reviewed-producer
measurements and cheap predicates; this command does not repeat eigensolves.

Proposed owning targets: `qeph_wall_selection_report` from
`WallSelectionReport.cpp` and `WallSelectionJson.cpp`, linked to the existing
job JSON/boost/selector libraries; `qeph_wall_select` additionally links the
derived reader. `WallSelectionReportTest.cpp` and its small fixture supply four
GTest functions for full retention/hash/byte accounting, scientific rejection,
partial evidence, malformed bindings, child-status failures, retry before write,
and create-only/budget failures. Inputs are explicit synthetic protocol records,
not accepted scientific archives. Root owns registration, source maps and all
executions. Reuse the one-CPU/1-GiB/60-second host envelope and strict C++ flags;
rerun the affected raw/analysis CLI protocol checks after RawInput extraction.
