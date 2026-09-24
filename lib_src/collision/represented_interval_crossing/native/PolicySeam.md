# Exact integer policy seam — source-only first stage

This branch starts from qualified `ec51b6b7`, plus the shared fixed-integer
extraction/CUDA primitive dependencies. The deferred canonical-witness experiment
is not included. Public CPU execution still uses the original witness traversal
and checked Boost512/16384 storage selection. No GPU executor, batch, public backend
selector or transaction wiring exists in this checkpoint.

## Ownership and failure contract

Each pair call creates a lexical `ArithmeticContext` and a noncopyable kernel
instance. The context is passed by reference into its thin integer policy. It is
not encoded in integers, source keys or thread-local/global storage. Only the pair
driver is allowed to clear failure through the private friend `BeginPair` method.
Every fixed operation latches overflow; sign/zero tests latch their validity
before returning a boolean or sign. A later zero assignment, cancellation or
successful operation cannot clear a prior failure.

`Arithmetic`, `Geometry` and `CellKernel` share one implementation across policies.
Numeric methods become instance methods; normal scratch explicitly receives that
instance. The cached normal becomes ready only while the context remains healthy.
The original kernel traversal is inside one policy `Protect` call. The Boost
policy preserves its checked operators and catch-all failure result; the fixed
policy executes without exceptions and checks its sticky context. There is no
wide retry after fixed arithmetic has executed.

Failure fences occur immediately after exact common-translation evaluation and
after every cell evaluation, before accepting a result, continuing, scheduling
children or incrementing another cell's work. A failed static common-translation
sample reports `ExactArithmeticRange` with work **zero**, matching the original
catch boundary. A failed ordinary sampled cell retains its current admitted work.
The outer fixed protection also rejects a latched error at return. The default
Boost policy's `Healthy` checks compile to true; exceptions retain their original
first-error behavior. No arithmetic precision, source validation, witness,
classification, work/depth cap or publication format is relaxed.

The fixed policy calls the already shared `tl::math::fixed_integer::Arithmetic<8>`
for every integer operation. Its `Integer` stays the same POD used by the qualified
CUDA primitives. There is no second bigint implementation. `Sign` uses the core's
validity-bearing Result; shifts and scalar scaling preserve sticky overflow.

## First-stage qualification boundary

`CompareFixedIntegerPolicy` is private host qualification. It performs native
path/identity checks first, derives `NativeStorageDomain` from the actual pair,
and selects fixed8 only at the existing B<=125 boundary. All other inputs execute
the full wide CPU route before fixed work. Both results pass the native fieldwise
StoreResult publication representation. The default public owner remains Boost.
Its retained wide scratch, DFS and worker stack forecasts are unchanged; kernel
instances/context add only scoped execution state. Compiler stack qualification
still must be refreshed for the changed call graph before production promotion.

Nine host groups compare complete stored results across VF/EE, nonuniform motion,
degeneracy, bounds, extreme scales, canonical permutations, DAZ/FTZ and malformed
inputs. Separate private numerical tests deliberately run out-of-domain data through
both checked512 implementations to prove work0/work1 rejection, cache safety and
retry. Those tests do not grant fixed-backend admission for such data. A sticky
error test shows that Sign and then assigning zero cannot erase an earlier fault.
Existing native owner/report/cap/publication/frozen-fixture gates remain mandatory.

The original69-body manifest remains immutable. The source checker reverses only
explicit enumerated policy-call and instance-context spellings and reconstructs
the original try/catch placement before comparing those baseline body hashes.
It separately pins the context gates and default Boost operator implementation.
This is a documented adapter transform, not a claim that all raw source tokens
are unchanged. Geometry and traversal formulas have one source.

No build, test or GPU execution has been performed by this author for the seam.
Owning host/source/Bazel checks, affected downstream gates and the compiler stack
proof must pass before any qualification claim. A controlled benchmark must verify
that the CPU-compatible default has not regressed.

## Subsequent standalone GPU stage

Only after this host seam passes, port the shared pure kernel to host/device
compilation. Device code must not instantiate Boost or exceptions. Reuse exact
source-key comparison and bit decoding with no fabricated/compressed identities.
A private GPU facade will reuse native host roster authentication, canonical
pair sorting/deduplication, work-cap folding and failure-atomic publication.
Actual B<=125 proof precedes GPU execution; wide/unsupported rows run original
wide CPU arithmetic. No in-domain device overflow or CUDA failure is hidden by a
retry. One retained scratch/DFS area per bounded GPU worker and one writer per
canonical result replace unbounded per-thread/per-pair allocations. All device,
transfer and compiler-local memory must be forecast and measured; no global CUDA
stack-limit change is authorized. The transaction integration remains later work.
