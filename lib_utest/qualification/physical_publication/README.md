# Physical common publication qualification

The explicit physical path in `ShellBatchPublication` has one owner commit and
the existing six participant cache publications. This target qualifies their
transaction; it adds no material or force equations and makes no full-vehicle
connectivity claim. Kinetic energy is explicitly unavailable until a current
CIN/rigid-primary observer is qualified.

The tiny fixture has 15 physical nodes, two QEPH layers, one T3, one QBAT, two
TYPE25 connections, one TYPE13 beam and one each Solid18/Solid24/Solid6z. It also
contains one PART, one two-member physical plain group and a CIN dependent.
Three actual coincident Q/Q/B parents witness that CIN patch. All internal
force/couple and STI/STIR contributions come from the real participants; the
only external load is a declared 10 N nodal load. Activity is read from the
accepted family caches authenticated by the publisher. No synthetic substitute
stiffness, preset activity or extra publisher peer is used.

Five host functions cover the complete fixture, mandatory contributor presence,
host budget boundaries/alias rejection/retry, and complete typed diagnostics.
The added host function freezes old status values, receipt non-aggregate
construction, and the exact separate one/two-issuer host forecast and cap.
Seven CUDA functions cover late attach rejection before any claim, duplicate
claims, stale activity source and alias rejection, capture rejection, all named
accepted histories/activity/forces/nodal/rigid/CIN fields across rollback,
last-contributor numerical failure after the other five candidates, changed last
solid diagnostic rejection, plus self-only and wall+self fixed-roster
participation. The latter uses the actual owner/token/assembly/prepared fixture
to reject missing, duplicate, stale/replayed, foreign source/owner/token/stream,
candidate-without-assembly, duplicate seal and late structural/validation
failure before successful half-kick and ordinary retries.
Snapshots compare named double bits, not aggregate padding.

Run the owning gate on a CUDA workstation:

```sh
cmake -S lib_utest/qualification/physical_publication -B <fresh-build> \
  -DCMAKE_BUILD_TYPE=Release -DPHYSICAL_PUBLICATION_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <fresh-build> --parallel 1
ctest --test-dir <fresh-build> --output-on-failure
```

Without `PHYSICAL_PUBLICATION_CUDA`, CMake builds only the five host functions
and executes focused source/public host-syntax proofs. Run this author gate
under `crash-work/reports/connection-author.lock`, one CPU and a 512 MiB
address-space cap. It invokes no NVCC, GPU, CUDA runtime operation, or Fortran.
The Bazel host target is
`//lib_utest/qualification/physical_publication:physical_publication_host_check`;
the focused executable source-proof target is
`//lib_utest/qualification/physical_publication:scratch_participation_source_proof`.
the complete actual CUDA gate remains owned by the CMake target above.

The publisher has zero CUDA allocations. `ForecastPhysical` reports only its
new host payload plus the maximum transient initial-proof phase. All already
owned participant, immutable source and owner bytes remain in the caller's
whole-composition budget. No 20 GB workstation or vehicle budget is inferred
from this tiny transaction test.

On the current incoming ABI the physical publisher's roster-absent owned payload
remains 6,880 bytes; the tiny startup reservation remains 8,704 bytes. The same
checked proof layout at the current original
372,435 nodes / 11,165 attachments gives 44,876,960 bytes of additional startup
reservation. This is an arithmetic forecast, not an original-owner execution or
a second charge for already-owned source/participant backings.

The frozen LP64 host ABI is: issuer 96 bytes, typed receipt 264 bytes, public
two-slot roster 32 bytes, and publication-owned configured state 112 bytes.
Accordingly the exact complete protocol totals are 208 bytes for self-contact
only and 304 bytes for mapped-wall plus self-contact. The optional state reuses
the internal two-word CIN-count header after one-time configuration, so the
roster-absent 6,880/8,704-byte physical forecasts above do not change.

Native reference comparisons remain in the separately owned Q/T/QBAT, TYPE25,
TYPE13 and solid resident gates. Author qualification is host/syntax only;
actual CUDA and affected legacy publication regressions are root-owned.

## Runtime integration

The opt-in profile embeds one
`ShellPhysicalScratchParticipation` privately in each concrete contact module.
After `InitializePhysical` and contact startup, but before interval 1, compose
the fixed roster and call `ConfigurePhysicalScratchParticipation` with the same
actual physical binding, participant pointers, publication identity and owner.
Use the mapped wall's immutable wall binding ID and the self-contact binding's
immutable source/profile ID unchanged as their nonzero source IDs. A wall-only
legacy case that does not configure a roster follows the byte/output-compatible
old path.

After each contact's successful accepted force/STI assembly, its private issuer
calls `RecordAcceptedAssembly`. After structural `PreparePhysical` and that
contact's successful candidate/activity/crossing checks, it calls
`SealCandidate` and returns the typed receipt to the app. The app passes the
fixed receipt slots to `SealPhysicalScratchParticipation`, then calls the
existing `CommitPhysical` with its independent case-validation receipt. Common
discard revokes both issuers; retry begins from the fresh owner token. No
contact operation, receipt publication, allocation, or callback occurs after
owner success.
