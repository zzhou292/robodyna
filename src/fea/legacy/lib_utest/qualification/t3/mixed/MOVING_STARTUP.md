# Shared uniform-translation startup

This bounded extension admits explicit uniform translation for standalone T3
and joined QEPH/T3 batches. The default reference-rest behavior remains intact.
Moving startup requires CoupledForces usage, exact declared common velocity
bits, reference positions, zero spin, identity orientation and free native
mass/inertia on every physical node. Initial material history and force caches
remain zero; no zero-duration force operation or fabricated interval runs.

`ShellBatchStartup.h` owns the common declaration, validation and ordered
binary64 kinetic helper. Existing QEPH names remain aliases. T3 adds the same
declaration and live-owner assembly overload. Initial moving assembly requires
source authentication before validation or kinetic publication.

Joined family kinetic stays unavailable and zero. The publication coordinator
checks identical startup declarations and both source identities, obtains fresh
accepted owner fields through CopyAccepted, and measures common K0 once in
native node order. Expired assembly views are never dereferenced. Initial
`base_kinetic` remains zero because epoch zero has no completed interval.
All fallible startup work precedes publication-scope claims.

Six new functions pass: four mixed startup tests and two standalone T3 tests.
They cover native initial/flight fields and the
first half-kick, kinetic accounting, rejected first-trial retry, foreign/raw
source rejection, startup mismatch, late union-node motion/mass faults and
metadata/kinetic overflow. The existing QEPH test now accepts coupled joined
startup while continuing to reject prescribed moving startup.

All 30 mixed/T3 collection functions, 20 affected QEPH/contact functions and
three original-source pulse/native functions pass. The three owning Bazel batch
and publication targets also pass. Evidence is retained under
`crash-work/reports/mixed-moving-startup-*`, `moving-startup-qeph-*`,
`moving-startup-geometry-*` and `moving-startup-bazel-1.*`.

The first integration build rejected a change to a pinned family BUILD file.
Both family files are preserved; the shared declaration is exported through
the existing neutral `shell_batch_fields` dependency. The source validators,
second build and runtime gates pass without changing any physics tolerance.

T3's configuration adds one 32-byte startup record. This startup contract does
not qualify a deforming mesh-wall trajectory or its timestep; actual-source
uniform flight and contact coupling have separate application gates.
