# CW1 incremental raw native capture

Source preparation only. Seven focused host tests are staged; this raw capture
slice has not been compiled or executed. The retained model/support gates do
not establish a completed CW1 screen, startup admission or impact response.
The prospective policy remains in `planning/QEPH_WALL_RECURRENCE_SCREEN.md`.

`CollectRawJob(cells, normal_velocity, callback)` accepts one or two cells and
physical world-X velocity -8, 0 or +8 m/s. It collects all six fixed steps and
three centered-difference amplitudes before contact work. Each inactive/active
sign-cone branch is then collected once per step using the finest native matrix;
the branch probe itself retains all three physical amplitudes. The complete
counts are 4,350 native cell intervals per one-cell job and 14,964 per two-cell
job, or 57,942 across the six fixture/boost jobs. Startup construction is outside
these interval counts. The full 109/194-coordinate dictionary is retained.

`RawJob` owns six `RawStep` records. Its `collection_complete` flag means all
native baselines/columns and all physical directional samples were collected.
It does not require local derivative checks to pass, and makes no spectral or
trajectory decision. Expected numerical failures retain earlier operands and
allow independent probes to continue; contact at a step is skipped if its
finest native matrix is incomplete. A callback exception stops collection.

`RawJobWriter` consumes typed Model, NativeMatrix, ContactBranch and Finished
callbacks. It requires a new directory, bounded exact provenance JSON and an
explicit remaining byte allowance. Its append-only layout is:

- `provenance.json`: the exact supplied bytes, once.
- `model.json`: frozen geometry, native reference, physical/added/total inertia,
  dictionary/scales, penalty bits and directed chain, contact weights and
  certificates, actual synthetic finite wall and coverage query metadata.
- `native-hN-aN.json`: one complete or partial matrix, moving baseline and flags.
- `contact-hN-bN.json`: full operator, baseline, all available actual host-law
  records/native states, directional quotients and local check diagnostics.
- `progress-NNN.json`: a new inventory after each payload, starting at 000.
- `index.json`: final inventory, which may explicitly describe an incomplete job.

Each payload is written once. It binds exact provenance and model hashes;
contact payloads also bind the retained finest native matrix file/hash. An
inventory lists every earlier file's bytes/hash, including earlier inventories,
and excludes itself. The returned receipt includes the current inventory too.
A completed job has 65 files. This is externally serialized create-only IO,
matching ArtifactIO; it is not an atomic/exclusive multi-file transaction.
Guard interruption preserves completed payloads and inventories, while the
current in-progress probe/file may be absent or incomplete. There is no resume
claim. Readers must authenticate bytes against the inventory and reject missing
or invalid operands before any decision; this slice implements no decision reader.

Finite doubles use full-precision JSON round trips. Failed/partial nonfinite
fields are explicit objects with a `nonfinite` kind and exact `binary64_bits`.
Completed matrix columns, published physical samples and completed quotient
checks must remain finite. No invalid JSON NaN token or silent zero substitution
is used. Native EINT0/EINT1/EVIS remain source work, not elastic potential.

Each file is bounded by 32 MiB; exact provenance is bounded by 64 KiB. The shared
96 MiB cap includes all six jobs plus derived decision/report artifacts, including
provenance, inventories and final indices. Root supplies each serialized job's
remaining allowance after reserving room for derived reports. `RawSetBytes`
checks six unique tuple receipts and adds explicit derived bytes; it checks
accounting, not external-file authenticity. No per-job full-cap default is offered.

Proposed owning targets for root wiring: `qeph_wall_raw_capture` from
`WallRawCapture.cpp`, linked to `qeph_wall_recurrence_model`; `qeph_wall_raw_report`
from `WallRawJson/Model/Contact/Probes/Report/Writer/Inventory.cpp`, linked to the
capture library and existing `qeph_recurrence_report` (which owns the reused
compact JSON encoder and links `qeph_recurrence_report_io`). Keep numerical
capture independent of JSON/OpenSSL and reuse strict floating-point flags and
single-thread Eigen behavior. No CLI or spectral orchestration is added here.

`qeph_wall_raw_check` combines `WallRawCaptureTest.cpp` (2 functions),
`WallRawReportTest.cpp` (4) and `WallRawContactReportTest.cpp` (1). Synthetic report
matrices are explicitly unqualified. Tests cover interruption after model,
collection-versus-check flags, partial nonfinite preservation, exact hashes and
precision, stale/duplicate input, create-only/budget failure and all used fields
from four actual contact/native samples. Root runs with one CPU, no CUDA, 1 GiB
and 60 seconds. Full raw jobs remain separate guarded executions under the
prospective six-job resource contract; no numerical gate or load is widened.
