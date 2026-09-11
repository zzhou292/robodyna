# Prepared TT0 physical rigid-main summary

This gate qualifies the explicit `PhysicalAggregateV1` query, not a native PART
RBYM main-field implementation. It uses the actual owner's initialized body mass
and minimum principal inertia, accepted pre-kick center/member positions, and
current post-CIN scalar stiffness. Source kind, group ID and node-set ID remain
separate; no main node is added to the physical domain. The joint damping
constructor's initial mean principal inertia is a distinct quantity.

The query requires the current actual token after successful CIN advancement at
base epoch zero. It returns every rigid group and the actual owner/fixed-step/CIN
receipt. It does not supply an independent permission to advance or commit, and
does not assert that optional structural screening was enabled. Accepted joint
history, rather than a later query, must retain the first accepted automatic K.

No device or retained host allocation is added. Seven doubles per group reuse
the dead CIN entry-inertia/acceleration tail after motion recovery; the two
stiffness arrays and separate requested force-stage acceleration capture are
preserved. Actual 773 roots require 43,288 bytes within existing scratch. Output
shape and aliases into retained source, owner and staging ranges are rejected
before writes. Caller rows and receipt publish only after all rows validate.

The three host functions cover actual aggregate versus zero member M/J, ordered
scalar reductions at nonzero centers, late invalid geometry/stiffness and exact
retry, and exact scratch/staging boundaries. One independent native function
compares both groups' scalar sums to authenticated RGBODFP member expressions.
The two CUDA functions use the existing mixed PART/plain/ordinary/CIN owner;
they cover complete identities, repeat reads, prepared state and acceleration
preservation, capacity/source/output alias failures, discard/stale retry and
post-first-acceptance rejection without another allocation.

## Donor scope

OpenRadioss pin `a62b27e6baa555d222a580d6218867d0be4d70b5`. The gate reuses the
existing independent `cin_physical_timestep/native` manifest, exact source
stager and wrapper. RGBODFP lines 119–121 provide each member's `DD`, `F1` and
`F2`; source IDs, actual body M/minimum J and aggregation are the explicit
physical policy. No copied or hand-mirrored native implementation is added.
The complete INIRBY, DTNODA, RMATFORP and RMATPON source receipt remains in that
manifest. In particular, the missing native PART RBYM stiffness producer and
DTNODA indexing defects are not presented as working native reference behavior.

## Root qualification

Author checks: three host functions pass under 1 CPU/512 MiB; query, owner test
and native test pass C++ syntax. Source identity staging passes. Native runtime,
NVCC/CUDA and owning Bazel qualification are root-only and pending at freeze.

Configure this directory with the normal root GNU Fortran/CUDA environment:

```sh
cmake -S lib_utest/qualification/cin_physical_main_summary -B <build> \
  -DTL_CIN_PHYSICAL_MAIN_NATIVE=ON -DTL_CIN_PHYSICAL_MAIN_CUDA=ON \
  -DTL_CIN_TIMESTEP_SOURCE_CACHE=/home/jsonzhou/Desktop/chrono-work/crash-work/deps \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> --parallel 1
ctest --test-dir <build> --output-on-failure
```

CTest names are `cin_physical_main_host`, `cin_physical_main_native`,
`cin_physical_main_cuda`, and `cin_physical_main_source_identity`: six numerical
functions plus identity. The owning Bazel targets are
`//lib_src/solvers:cin_physical_main_values`,
`//lib_src/solvers:explicit_nodal_state`,
`//lib_utest/qualification/cin_physical_main_summary:cin_physical_main_host` and
`//lib_utest/qualification/cin_physical_main_summary:cin_physical_main_cuda`.

Affected gates are the previous `cin_physical_timestep_{host,matrix,native,cuda}`
and source identity, plus existing `rigid_assembly_owner_cuda` and
`tied_cin_runtime_cuda` (prepared/readback/commit behavior). No existing force,
time integration, donor expression or tolerance changes in this increment.
