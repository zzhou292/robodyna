# Resident joined TYPE25 contributor

`Batch` owns one bounded device arena with exact active property/connection/node
arrays, two complete `Evaluation` slabs and candidate status/control scratch.
It retains immutable `Model` and `NodalMassBinding` handles. Counts are limited
to 64 properties, 1024 connections and 2048 nodes; default device/host caps are
2/8 MiB, with explicit maxima of 16/16 MiB. Startup charges retained immutable
payloads in full plus temporary arena and permanent readback staging. There is
no per-step allocation, independent owner, clock or standalone commit.

InitializeJoined requires a fresh extended staggered owner and either reference
rest or declared uniform translation with zero spin. The first live assembly
authenticates complete owner sources, checks original x/v/w/q, combined native
M/J and free component masks, and queries the owner's actual immutable rigid
membership for every connector endpoint. The supported subset rejects endpoints
inside rigid groups; it does not infer membership from a source descriptor.
Native X0 is established at original geometry with a known zero-stress cache,
complete reference frames and a measured property-coefficient step bound. The
first drifted Evaluate is the subsequent native recurrence interval, not TT=0.

The accepted endpoint force/couple cache is already the nodal RHS. Assembly uses
+1 scatter, one serialized writer in source connection then endpoint order. This
preserves repeated-endpoint additions and both native shear-arm couples. A failed
contribution marks the common assembly sticky. Independent candidate elements
use a 64-thread grid, followed on the same stream by a serial first-failure scan
and ordered diagnostics. Native scalar work channels remain signed and distinct
from accepted-cache kick/drift work. No kinetic sum, physical energy closure,
damping interpretation or timestep acceptance policy is supplied by this batch.
The reported native minimum conservatively includes inactive tracked elements.

Only ShellBatchPublication can claim and publish a batch. Its private preflight
requires the exact retained combined binding, startup/configuration/qualification,
live owner, common token, pending view and diagnostics. After the owner's sole
commit, Publish only selects the trial slab and host identity; it cannot allocate,
launch CUDA or fail. Discard clears pending authority without changing accepted
history/cache. Readback validates all supplied ranges, copies to owned staging,
and publishes complete typed outputs only after successful CUDA completion.
Detected device errors poison the batch; no accepted device-read guarantee is
made after a context failure.

`Type25Batch.cmake` owns `tl_type25_batch`; Bazel owns `:type25_batch`. Both reuse
the existing startup, bounded arena, native coefficient authentication, nodal
scatter and common view identity utilities. Native donor math/hash qualification
is separate under `lib_utest/qualification/type25`; runtime batch/publication
qualification must run on CUDA before claiming connected dynamics. Source audit
and equations remain in SOURCE_CONTRACT.md without any reference repinning.
