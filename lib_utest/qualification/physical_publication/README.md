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

Four host functions cover the complete fixture, mandatory contributor presence,
host budget boundaries/alias rejection/retry, and complete typed diagnostics.
Three CUDA functions cover late attach rejection before any claim, duplicate
claims, stale activity source and alias rejection, capture rejection, all named
accepted histories/activity/forces/nodal/rigid/CIN fields across rollback,
last-contributor numerical failure after the other five candidates, changed last
solid diagnostic rejection, and successful first/second physical intervals.
Snapshots compare named double bits, not aggregate padding.

Run the owning gate on a CUDA workstation:

```sh
cmake -S lib_utest/qualification/physical_publication -B <fresh-build> \
  -DCMAKE_BUILD_TYPE=Release -DPHYSICAL_PUBLICATION_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <fresh-build> --parallel 1
ctest --test-dir <fresh-build> --output-on-failure
```

Without `PHYSICAL_PUBLICATION_CUDA`, CMake builds only the four host functions.
The Bazel host target is
`//lib_utest/qualification/physical_publication:physical_publication_host_check`;
the complete actual CUDA gate remains owned by the CMake target above.

The publisher has zero CUDA allocations. `ForecastPhysical` reports only its
new host payload plus the maximum transient initial-proof phase. All already
owned participant, immutable source and owner bytes remain in the caller's
whole-composition budget. No 20 GB workstation or vehicle budget is inferred
from this tiny transaction test.

On the author ABI the added owned payload is 6,112 bytes; the tiny startup
reservation is 7,936 bytes. The same checked proof layout at the current original
372,435 nodes / 11,165 attachments gives 44,876,960 bytes of additional startup
reservation. This is an arithmetic forecast, not an original-owner execution or
a second charge for already-owned source/participant backings.

Native reference comparisons remain in the separately owned Q/T/QBAT, TYPE25,
TYPE13 and solid resident gates. Author qualification is host/syntax only;
actual CUDA and affected legacy publication regressions are root-owned.
