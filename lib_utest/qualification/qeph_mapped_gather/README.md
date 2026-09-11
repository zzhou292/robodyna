# Deterministic mapped QEPH assembly

The mapped QEPH assembly at TL `52ca602` launched one CUDA thread for the
complete family. Its candidate constitutive loop was already parallel. This
increment changes only mapped accepted force/stiffness assembly, with five
stream-ordered kernels:

1. Check the authenticated assembly destinations and reset diagnostic scratch.
2. Independently validate each parent and prepare its existing native stiffness
   packet, including the virgin versus accepted and rigid-skin branches.
3. One thread per node starts from its actual existing force/couple/STI/STIR and
   gathers that node's parents in the old ascending parent/local-slot order.
4. Reduce the earliest error using only an integer atomic minimum and record the
   same first parent/node error identity.
5. Publish staged nodal values only if all checks succeed.

The addition uses the unchanged `AccumulateNodalForces<1>` and
`ShellNodalStiffness::Add<1>` helpers. This preserves unary force negation,
rounding, existing producer contributions and signed zero. There are no floating
atomics, reordered sums, material/history updates, extra owner clocks, or
per-attempt allocations. A failed attempt now preserves the complete incoming
nodal contribution arrays; the old private serial kernel could leave earlier
parents' contributions, but already required discarding the failed attempt.

The flat incidence uses the same parent/local traversal as
`cpu_utils::BuildNodeIncidence`; it directly constructs bounded contiguous
storage instead of an Eigen matrix plus one vector per node. The offsets double
as startup insertion cursors, avoiding an additional full-node cursor array.
The existing family arena owns the optional tail; old initializers allocate no
such tail. The private header gains five pointers (40 bytes on the qualified
64-bit target); this is in-process storage, not an archive format.

For 324,094 QEPH parents and 372,435 owner nodes, the optional device tail is
56,825,344 bytes, and the complete mapped family base arena is 1,183,491,056
bytes. These values exclude the unchanged material sidecar. Startup host
forecast includes the complete new staging arena, and the full app device
forecast includes the tail. The 6 GiB device-growth guard remains unchanged.

The five host functions cover incidence ordering, late invalid connectivity,
counts-before-input, exact layout caps/rebase, all eight masked physical-parent
combinations with a separate rigid skin, initial/accepted packets, all eight
nodal channels versus serial scatter, a cancellation case that rejects
reassociation, signed zeros, nonzero incoming producer values, overflow rollback
and retry, and node-versus-parent validation priority.

Two actual-CUDA functions compare the new kernels to the test-only frozen
`52ca602` serial caller: initial and two accepted stamps/all masks; late skin
and node failures; earlier overflow versus later validation; same-parent error
priority; complete destination preservation; and fresh retry. The tests supply
valid force packets to isolate assembly. Complete constitutive trajectories,
source-specific placements/failure policies and the real sole-owner transaction
remain covered by the existing `qt_mapped`, `physical_publication` and full
vehicle loaded gates. No tolerance change or native equation change is made.

Author evidence: host values, C++ syntax and temporary CUDA-to-host syntax only.
Actual NVCC/native/GPU execution and performance measurement belong to root.

Owning gates:

```
cmake -S lib_utest/qualification/qeph_mapped_gather -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DQEPH_GATHER_CUDA=ON
cmake --build BUILD -j2
ctest --test-dir BUILD --output-on-failure
bazel test //lib_utest/qualification/qeph_mapped_gather:host
bazel build //lib_src/elements/qeph:batch
```

Also rerun the unchanged mapped Q/T host/native/CUDA tests and complete loaded
vehicle startup, accepted steps and discard/retry. Compare whole-process device
growth against the existing 6 GiB guard. Measure initial and later QEPH assembly
separately; do not report constructor or first-only TYPE45 initialization time as
steady-state trial cost. App timers surround existing synchronized calls, so no
additional GPU synchronization is introduced here.
