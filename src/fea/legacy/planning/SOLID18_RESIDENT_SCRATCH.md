# Solid18 resident scratch allocation

The first complete original vehicle startup stopped its GPU growth guard before
publication. Explicit participant arenas forecast about 2.012 GB, while sampled
NVML total usage rose from about 2.009 GB to 14.153 GB. The root report is
`vehicle-runtime-original-owner-tests-1`. No guard increase or successful initial
vehicle startup is claimed by this correction.

`cuobjdump --dump-resource-usage` on the owning startup executable identified
`solids::batch_detail::Initialize<Traits18>` with 39,728 bytes of stack per thread,
and `Evaluate<Traits18>` with 36,224 bytes. Nested automatic ForceTrial, History
and geometry packets explain a large device-wide automatic stack reservation.
Explicit arenas alone did not account for that CUDA runtime allocation.

## Bounded storage change

The existing solid18 arithmetic now has one internal `CalculateForceStaged`
implementation. Public `EvaluateForce` and constructor `InitializeForce` keep
staged value publication. The resident uses the same implementation with private
unpublished scratch. `HistoryWriter` retains the same validation and field writes;
its caller supplies disjoint unpublished history storage. Public prescribed
history still stages a complete history before output publication.

`Scratch18` owns the large force packet, next point histories, current startup
geometry, constructor history/interval and staged force cache. On the author
binary64 ABI it occupies 22,144 bytes. Arena admission reserves
`min(solid18_parent_count, candidate_blocks * candidate_threads)` slots, with the
existing 64 blocks by 64 threads. Each candidate worker owns one slot for its
entire grid-stride sequence. Initialization uses its original one block of 64
threads and the first slots of the same allocation. Different parents are never
allowed to use a slot concurrently. No workspace is an accepted history owner.

A failed geometry, material point, history or cache check may leave private
scratch partially written. Accepted/output state stays unchanged until the whole
packet succeeds; retries overwrite every live field. The existing two state slabs,
accepted selector, force signs, point visitation, reductions and owner clock are
unchanged. HEPH24 and S6Z retain their prior value adapters.

The arena layout, upload construction/rebasing and public Batch forecast include
the new region. The 908/1309/195 original family counts and eight-point source
curve give 20,106,752 bytes of scratch, a complete solid device arena of 33,644,608
bytes and the unchanged 5,405,280-byte readback staging. Host upload scratch grows
by the same explicitly charged device region. The existing 128 MiB device cap is
retained; a count ceiling does not promise every mixture fits that cap. No scratch
is reserved when the solid18 family is absent.

## Qualification boundary

Author checks under one CPU and 512 MiB:

- Four new host functions: constructor and 400-step exact typed values, eighth
  point overflow and stale-phase rollback/retry with an independent workspace,
  original-count exact device cap and one-byte-short preservation, and complete
  launch-slot coverage at empty, boundary and maximum parent counts.
- Four existing solid18 force host functions and four existing three-family
  constructor host functions.
- Changed host arena/upload/forecast source syntax and unchanged original
  adhesive geometry/material identity checks.

Root must run the existing owning native/CUDA gates before integration is
qualified. Use `lib_utest/qualification/solid_resident` with
`SOLID_RESIDENT_CUDA=ON`, `SOLID_RESIDENT_ORIGINAL=ON`; targets
`solid_resident_host` and `solid_resident_cuda`, CTest selectors
`solid_resident_host|solid_resident_cuda|solid_resident_.*identity`.
This includes original 908/1309/195 constructor/native comparisons and the real
owner recurrence, accepted/trial readback, late material failure and retry.
Also run `solid18_force_host|solid18_force_native_test|solid18_force_sourcenative_test|solid18_force_cuda|solid18_force_source_identity`
in the existing solid18 force gate, and the constructor gate's
`solid_force_startup_*` tests. Donor wrappers, equations and tolerances are not
modified.

Owning Bazel targets are `//lib_src/elements/solid18:force`,
`//lib_src/elements/solids/resident:values` and
`//lib_src/elements/solids/resident:batch`.

Before the full original startup retry, inspect the newly linked executable with
`cuobjdump --dump-resource-usage` and record Initialize/Evaluate solid18 stack
bytes. A passing small value test does not establish that device-wide stack
reservation has been reduced. The full startup report must retain the original
failed resource evidence and use the actual updated public arena forecast.
