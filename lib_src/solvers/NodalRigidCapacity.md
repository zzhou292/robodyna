# Explicit rigid-group capacity

`NodalStateConfig::rigid_limits` defaults to 64 groups and 16,384 members,
preserving the previous effective 64 × 256 limit for custom legacy models.
The immutable host model retains its own default of 64 groups / 4,096 members.
Both `NodalRigidOwnerLimits::Vehicle()` and `NodalRigidGroupLimits::Vehicle()`
explicitly select 1,024 groups / 8,192 members. Every group still requires 2–256
complete members, with no duplicated or omitted physical nodes.

The owner also needs explicit `max_nodes` and `max_device_bytes` appropriate to
the selected node inventory. The separate hard device cap remains 256 MiB and
the separate hard node cap remains 524,288. No contributor or source-mechanics
admission is implied by this owner capacity.

## Storage and admission

`RigidStorageLayout` forecasts the existing compact immutable device arena and
the rigid-only retained host payload before reading borrowed nodal arrays.
The latter includes the `RigidStorage` object, source properties and members,
compact ranges and metrics, membership mask and staged snapshots. Its cap is
8 MiB. Actual vector capacities are checked again before publishing the owner.
Allocator bookkeeping, the caller's immutable model and common nodal staging
are separate; no startup-only rigid array or per-step allocation is introduced.

`StateLayout` then includes all seven existing device allocations, both
18-double/group state tails and optional A/AR capture. Count and device-byte
failures now precede malformed borrowed nodal values. After that preflight,
the original ordered per-node and exact source/native-M/J checks are unchanged.
Rejected initialization publishes no owner; stepping still has one accepted
slab, trial slab, clock and commit. The group loop, arithmetic, failure order,
readback and observation semantics are unchanged.

For the current 64-bit build (`sizeof(RigidStorage)=288`, `sizeof(Control)=136`):

| Nodes / groups / members | Rigid host payload | Immutable device metadata | Whole device owner | With A/AR capture |
|---|---:|---:|---:|---:|
| 359,785 / 673 / 6,170 | 1,442,337 B | 534,785 B | 148,600,380 B | 165,902,364 B |
| 359,785 / 759 / 7,539 | 1,636,369 B | 571,081 B | 148,661,444 B | 165,967,556 B |
| 524,288 / 1,024 / 8,192 | 2,056,480 B | 761,856 B | 216,539,272 B | 241,754,248 B |

The 759-group case additionally retains 54,796,616 B of common nodal staging
and 1,079,355 B of constraint staging. These are payload forecasts, not process
RSS estimates. Executable evidence is the workspace report
`crash-work/reports/vehicle-rigid-capacity-forecast-1.json`.

## Qualification scope

`nodal_rigid_vehicle_host_check` uses exact source group IDs and count shapes
(673 internal / 6,170 members, or 759 / 7,539) from the full-shell report, with
explicitly synthetic positive M/J, geometry and source-node IDs. The source
report SHA-256 is
`da85bfffc01f96f962c5e8658a66912e915838f3b376afc3a39d3cf71ae32f1b`.
It preserves four two-member declarations, source traversal and the last global
node 359,784. It tests old default rejection, both profiles, exact bytes,
overflow, late invalid identity and clean retry.

With `TL_NODAL_RIGID_OWNER_CHECKS=ON`, `nodal_rigid_vehicle_owner_check` covers
the complete physical-node extent, 759 groups, the explicit 1,024-group maximum,
capture on/off state parity, complete source-bearing readback, observation,
poisoned borrowed inputs under invalid caps, last-group failure, discard and
exact retry. CUDA execution is a separate required gate from host compilation.
Capacity does not establish actual source mass/load-path closure, admissibility
of crossing groups or performance of the currently serial group advance.
