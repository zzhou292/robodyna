# Vehicle-sized resident shell collection

This qualification admits explicitly requested active storage for one complete QEPH family and one complete T3 family. It does not admit the original vehicle's unsupported materials, deletion, tied interfaces, beam welds, or omitted masses. Native force/history arithmetic and the common owner/publication transaction are unchanged.

`ShellResidentLimits::Vehicle()` admits up to 524,288 parents and global nodes. Each family also requires an explicit `max_device_bytes` (at most 2 GiB); its conservative host startup payload is bounded by 4 GiB. `ShellPublicationLimits::Vehicle()` admits 524,288 nodes with 32 MiB device and 128 MiB host limits. Legacy defaults still admit 128 parents/nodes, retain the old hard ceilings, and reject an oversized number without the new profile tag. Appending that immutable tag enlarges each family's model header by 8 bytes; legacy numerical state, array extents and allocation counts are unchanged.

Startup indexes preserve original parent traversal, local-node accumulation order, source-ID bits and first-failure precedence. Only valid vehicle startup takes the indexed path. The T3 repeated-node invalid-input fallback deliberately retains the old subset scan; it is not part of a valid full-vehicle startup. There is no per-step index work or allocation.

The per-family host budget is an inclusive startup bound: Impl, native model/results staging, compact ID/seen/index scratch, native reference scratch, optional plastic state staging and retained immutable scope. It is not additive across families. For the Vehicle profile, the embedded binding object is counted through Impl, and catalog inventory backing is discounted only after pointer, word extent and byte extent prove the same allocation. Independently allocated equal inventories are counted separately. The optional combined nodal-mass producer keeps its existing opaque inclusive budget and admission limits.

## Exact layout forecast

The source counts come from tire-free scope report 6: 328,344 Q4 + 21,301 T3 parents and 359,785 nodes (source SHA-256 `67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301`). These are storage counts, not an assertion that this synthetic fixture is original source geometry.

| Payload | Bytes |
|---|---:|
| QEPH model and two native result slabs | 1,132,577,992 |
| T3 model and two native result slabs | 72,889,780 |
| QEPH optional three-point parameters/history, 1,024 curve slots | 210,156,584 |
| T3 optional three-point parameters/history, 1,024 curve slots | 13,649,064 |
| Common four-array native metric | 11,513,280 |
| Ordinary rotational nodal owner | 147,871,771 |
| Total module device payload | 1,588,658,471 |

The two-point fixture uses 1,588,625,767 device bytes. Each maximum-count QEPH native + optional plastic arena needs 2,141,209,176 bytes, below the explicit 2 GiB family ceiling. A maximum-count six-array publication layout needs 25,165,984 bytes. The CUDA fixture prints actual module allocations, unique retained binding/catalog bytes, one complete result-buffer payload and one complete 19-double/node snapshot. Its multiple expected/accepted/retry buffers are test-owned and separately bounded by the workstation guard. CUDA runtime/driver and allocator bookkeeping are not included in module payload numbers.

## Owning gates

Small host-only gate (CUDA headers, no GPU execution):

```sh
cmake -S lib_utest/qualification/vehicle_shell_resident -B /tmp/vehicle-shell-resident-host-1 -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/vehicle-shell-resident-host-1 --parallel 1 --target vehicle_shell_resident_host_check
/tmp/vehicle-shell-resident-host-1/vehicle_shell_resident_host_check
```

Seven host functions cover exact byte layouts/overflow, explicit profile admission, shared versus equal-independent scope accounting, indexed native startup equivalence, complete indices beyond the old cap, and malformed-input first failure. Author qualification passed under one affinity CPU and 512 MiB; the CUDA fixture has only host syntax evidence before the parent runs its owning gate.

The parent's serialized CUDA gate enables `-DTL_VEHICLE_SHELL_RESIDENT_CUDA=ON`, supplies the workstation CUDA compiler/architecture flags, and builds `vehicle_shell_resident_check`. Its four functions cover full-count elastic and tabulated/analytic LAW44 history, independent host tail packet checks, complete-node common kinetic scope, insufficient readback capacity before poisoned pointers, explicit budget/default rejection, immutable original inputs, joined-only publication, late final-Q/final-T failures and exact retry with stable allocations. Source-shaped synthetic geometry/materials exercise storage and recurrence; the existing native Fortran suites remain the independent formulation oracle. Run this gate within the parent's shared workstation lock and 8 GiB process guard; it must not overlap another heavy/GPU job.
