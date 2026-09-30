# Vehicle-sized native mesh-wall contact storage

`NodalWallDeviceLimits::Vehicle()` explicitly admits up to 524,288 parents, compact incident nodes and global nodes. Callers separately supply `max_device_bytes` and `max_host_bytes`, each capped at 2 GiB. Legacy defaults remain 128/128/128 with 512 KiB device and 1 MiB host; their explicit larger profile remains bounded at 1,024 parents, 2,048 nodes, 8 MiB device and 16 MiB host. Raising an old numeric limit alone cannot select Vehicle.

This adds storage/startup capacity to the existing fixed finite mesh wall and reference-area penalty law. It changes no node/parent force arithmetic, directed reductions, CUDA launches, temporal or mass admission, motion coverage, or transaction. It does not establish full-vehicle self-contact, source closure or new material/connection support. The rigid-group mass metric remains a local native-coefficient diagnostic with the existing scope restrictions.

Vehicle startup replaces the old node-times-parent incidence search with a dense global-node-to-compact index. Native global indices are already bounded identities, so a source-ID sort is unnecessary. Counts become prefix offsets; the same offsets serve temporarily as write cursors during the original parent/local traversal and are then restored. Subsequent rate evaluation follows the same ascending compact node order and identical per-node share order as legacy. Point evaluation is shared by both paths. The index is released before startup returns; there is no new per-step allocation/work.

## Payload and ownership

| Payload | Tire-free source counts | Hard maximum counts |
|---|---:|---:|
| Parents | 349,645 (328,344 Q4 + 21,301 T3) | 524,288 |
| Compact/global nodes | 359,785 | 524,288 |
| Complete contact device arena | 1,078,530,400 B | 1,602,831,752 B |
| Contact host preparation admission | 1,080,493,924 B | 1,605,453,288 B |

The arena retains the existing pointer-header model, four shares/parent, ordered incidence, compact statuses, six global scatter arrays, error scratch and complete base/candidate result arrays. The immutable profile adds eight bytes to its model header; fixed `NodalWallDeviceResults` remains exactly 79,248 bytes. Its overload still rejects large scopes before reading caller output/identity. Active readback requires the exact profile/counts and disjoint caller-owned ranges; all transfers complete into retained staging before publication.

Host admission counts the full retained arena, the existing 512 KiB reserve for host shadows/Impl/finite wall preparation, and the exact startup index payload (`4*global_nodes + 96` bytes including the bounded-array wrapper/control reserve). Caller-owned source weights/fields, prior output instances during pure host replacement tests, result buffers, allocator bookkeeping and CUDA driver/runtime memory are outside this module payload. No second physical mass/state owner is introduced.

Together with the qualified resident Q/T, optional LAW44 history, ordinary nodal owner and common publication forecast, the source-count total is **2,667,188,871 B of module device payload** (using the conservative 1,024-point curve pool). This is not a measurement of total GPU utilization. Runtime/driver memory and other modules remain subject to the parent's serialized 4 GB GPU-growth and 20 GB workstation guards.

## Qualification

The small owning host target uses CUDA headers/runtime linking but invokes no GPU work:

```sh
cmake -S lib_utest/qualification/vehicle_wall_device -B /tmp/vehicle-wall-device-host-1 -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/vehicle-wall-device-host-1 --parallel 1 --target vehicle_wall_device_host_check
/tmp/vehicle-wall-device-host-1/vehicle_wall_device_host_check
```

Five host functions cover exact source/max arena bytes, default/profile/count/byte rejection before borrowed physical reads, exact-capacity/disjoint output ranges, mixed legacy/indexed incidence/mass/position/rate bit parity, malformed physical-input first-failure order, immutable prepared-state rejection/retry, and a native 2,122-parent/4,097-node fixture beyond the old caps. They passed under one affinity CPU and 512 MiB. The first rate-overflow negative used huge stiffness alone, which correctly remained finite; the preserved initial failure was corrected by supplying a finite inverse mass that makes the intended rate overflow. No production tolerance changed.

For the parent-owned actual gate, enable `-DTL_VEHICLE_WALL_DEVICE_CUDA=ON`, use the workstation CUDA compiler/architecture settings and build `vehicle_wall_device_check`. Its four functions include full-count host incidence coverage/late mass failure, full-count actual CUDA base/candidate forces and two accepted intervals, last global node scatter-overflow atomicity/exact retry, and large active readback/legacy/stale/partial-transfer rejection. The failure fixture loads only the final node so the complete global resultant stays finite while the destination addition provably overflows. Fixed native square/triangle patches supply genuine native mass/J, but their repeated geometry and distinct IDs are synthetic capacity fixtures, not an original Yaris crash.

The author performed host syntax checks of the CUDA fixture; the failure kernel's temporary syntax surrogate strips only launch qualifiers in `/tmp`. This is not CUDA execution evidence. The complete-count host/GPU gate must run through the parent's shared workstation lock with an 8 GiB process budget; no author GPU or overlapping heavy job is authorized.
