# Shared fixed integer primitive qualification

Source-only until root qualification. This does not dispatch native contact work
to the GPU and adds no solver, owner, device error context or runtime selector.

`lib_src/math/FixedInteger.h` reuses the eleven existing integer-only helper
bodies from `fixed_triangle_features/ExactInteger.h`, including complete zero
initialization, significant-limb loops, signed magnitude and sticky overflow.
The feature wrapper retains its binary64 decode/alignment, domain selection,
public Sign type and 8/144-limb policy. Arithmetic aliases inherit the same core.
ExtractionManifest records exact old body hashes. The only body spelling changes
for device compilation replace unsigned std::min/max with equivalent scalar
helpers; raw fixed arrays avoid std::array device-library/ODR dependencies.
The shared math annotation follows existing TL host/device helpers and includes
no CUDA runtime. Integer types do not change across host/device compilation.

New operations are FromU64, checked magnitude ShiftLeft and validity-bearing
signed Compare. Amounts at/beyond capacity and discarded high bits latch overflow.
Zero keeps its canonical positive sign, including negation and multiplication.
Invalid old arithmetic payloads are not required to normalize discarded carries;
callers must respect overflow/CheckedSign.valid before interpreting a value.
This API assumes canonical internally produced operands, not hostile raw limbs.

The uint128 multiply/carry loops are retained for CUDA 13.2/Linux; no intrinsics
rewrite or allocation is introduced. No floating arithmetic is performed.
Boost is only the host unbounded mathematical test oracle. A native future port
still needs its authenticated full-pair domain and explicit per-pair failure
propagation; these primitives do not establish that higher-level contract.

Six host groups exercise 1,257 bounded cases, unsigned interpolation weights,
carry/borrow/cancellation, signed comparison, zero padding, shift boundaries,
overflow propagation and the retained Integer3 helpers. Two CUDA groups compare
every limb/used/sign/overflow/result field with host and the unbounded oracle,
including different block sizes/order and repeated evaluation. Both device
buffers are bounded by 2,048 POD rows; each launch uses an explicit nondefault stream.
No simulation, GTest-per-pair setup, relaxed arithmetic or performance claim.

Configure this directory with CMake. Set FIXED_INTEGER_CUDA=ON and explicitly
set CMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc for the optional actual GPU tests.
Bazel host/cuda targets own the same corpus. Its source_proof wrapper executes
the same extraction check as CMake using explicit runfiles and no shell command. Root must additionally rerun the
owning fixed_triangle_features 57 GTests, source/shape, and 314-record complete
adaptive/wide discovery parity because those production predicates reuse this
header. Standalone primitive parity does not replace those regressions.
