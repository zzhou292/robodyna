# Vehicle-sized immutable host shell binding

`ShellBatchBinding::Initialize(input, ShellHostBindingLimits::Vehicle())` explicitly
admits up to 524,288 total parents and 524,288 global nodes, with independently
bounded 1 GiB immutable payload and 64 MiB startup scratch. These are host binding
bounds only. Default collection admission remains 128 parents/nodes with inline
storage; the pair has its version6,52-word inventory and 4..7-node contract.
`ShellHostBindingLimits{}` retains the earlier 1,024-parent/2,048-node/4-MiB scope.
Its previous out-of-domain status behavior is retained.

The old shared `MaxShellHostParents/Nodes` are unchanged. In particular, the
existing plasticity catalog rejects `Vehicle()` before inspecting its borrowed
records or indexing fixed arrays. No catalog/material, resident batch, contact,
owner, integrator or numerical formulation is expanded by this patch.

`ShellBindingIdentityIndex.h` owns private startup-only identity indexes. Each
entry stores the complete uint64 source ID and original occurrence. Sorting this
scratch does not reorder source parents, local connectivity, native startup or
mass/inertia reduction. Binary lookup reports the first original occurrence;
the original validation loops therefore find the earliest second occurrence,
even when it has a larger ID than a later duplicate.

The phases remain connectivity/parent identity, all QEPH then T3 native startup,
source-node identity/coordinate bits, coverage, QEPH then T3 ordered native M/J,
and one immutable publication. Signed zero and all source-ID bits remain part
of inventory and shared-coordinate identity. No material or total-inertia value
is reconstructed. Failures preserve the destination object's bytes.

Index/seen storage uses `BoundedStartupArray`; small startup remains allocation
free and copies/moves share immutable backing without allocation. Vehicle
startup preflights actual array extents and reserved shared-control bytes before
borrowed reads. The scratch budget includes both index objects and arrays plus
seen flags; allocator bookkeeping/call-stack overhead is outside this payload
budget and remains covered by the independent process resource guard. All
dynamic scratch is released when initialization returns.

Include `VehicleShellHost.cmake` in a host CMake project with GTest available.
It owns `vehicle_shell_host_check` and two separately schedulable tests:

- `vehicle_shell_host_small`: five functions for malformed/count/byte/scratch
  admission before poisoned input, source-order duplicate failures, shared
  signed zero and late failures, all 12 dynamic allocation failure points,
  allocation-free immutable copies, and existing catalog rejection.
- `vehicle_shell_host_source_size`: two source-sized **synthetic** functions at
  328,344 Q4 + 21,301 T3 / 359,785 nodes. They check complete source IDs and native
  inputs, every global M/J, exact ordered native totals, independent flat-area
  mass, immutable lifetime, last-node failure and exact clean retry. They are not
  an authenticated Yaris startup, source-material qualification or dynamics run.

The author ran only the small gate and 37 existing host binding/catalog/pair
regressions under one CPU/512 MiB. The parent owns the source-sized gate under
the workstation lock with a 2 GiB process bound. No GPU is needed for either.
