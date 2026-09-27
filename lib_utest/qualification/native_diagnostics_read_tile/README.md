# Native diagnostic read tile experiment

Source-only branch from impact-qualified TLad610. No compile, CUDA correctness,
resource or performance result exists for this experiment yet. Delivered data
and current qualified binaries stay unchanged.

Production changes are confined to diagnostic read staging and the64-lane launch.
`runtime/diagnostics/Tile.h` owns a2048B trivial shared SoA tile. `Read.h` checks
nonzero positive flags before dereferencing canonical slots/responses. All
inactive and tail lanes remain unread. The real runtime arena keeps these input
regions disjoint from Control, and ResponseRows/PackForces precede Diagnostics
on the same stream. No new public pointer/alias API is imposed.

Only lane0 loads incoming totals/count, folds every positive occurrence in the
original canonical order, applies the existing three unit conversions and
publishes exactly the same four Control fields. The original terminal integer
failure minimum remains unchanged, including prior failures, NaNs and overflow.
The leader never replaces individual terms with tile sums or adds inactive zero
terms. Two block-uniform barriers protect each tile; bounded remaining/count
iteration handles partial tails without final index wrap. No retained allocation,
clock, response/history/arena layout or physics admission changes.

## Qualification prepared, not executed

CMake option TL_NATIVE_READ_TILE_CUDA=ON selects one literal old/current kernel
comparison target. The reference translation-unit source is frozen atad610.
The existing diagnostic Rig is reused by exact generation, changing only its
include binding and current launch1→64; reference remains<<<1,1>>>. The existing
eight diagnostic test bodies and full-Control/operand comparisons are unchanged.
Both old/new cases share one CUDA translation unit to avoid duplicate private
oracle kernel definitions.

Expected tests: four host (two existing arena/disjointness plus two field-read
cases),12 CUDA (eight existing plus four boundary cases), one source proof;
three CTests. New cases cover63/64/65 and127/128/129 tails, nonboolean flags,
canonical permutations and cancellation across tiles, inactive poisoned/null
operands, cross-tile overflow/NaN/prior failure, wrap and same-device repair.
The original tests retain empty counts, signed-zero seeds, unit conversion and
unit-overflow checks. Full Control means all eight fields, including the three
nondiagnostic counters and the prior integer failure key; operands stay exact.

The source proof authenticates13 unchanged fixture/runtime files, the frozen
whole kernel source, two new read headers and normal CMake/Bazel registration.
It admits only the reviewed diagnostic replacement/include/launch; the original
terminal conversion/stores/Fail suffix is exact. The older diagnostic source
proof remains frozen and is not weakened to accept this later scheduling change.

Before promotion, inspect actual production registers, stack, shared/local
memory and device footprint; then affected owner/native-contact gates, exact
101-step outputs and paired uninstrumented timing. The prior742ms/202-call
observation pools interfaces and belongs to a serialized diagnostic trace; it
is neither a promised saving nor a throughput or OpenRadioss CPU comparison.
