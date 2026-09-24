# Bounded exact integer storage

Source-only optional performance slice, based on selected TL `b9957e84`. No
qualification or speedup is claimed at authoring. The active vehicle source is
separate. Promotion requires owning tests, full-record discovery parity, and a
controlled measurement after contact-substage evidence establishes its relevance.

`ExactPredicates.h` and the predicate results remain unchanged. The former
`ExactPredicates.cpp` integer and predicate bodies are reused in private
`ExactInteger.h` and `ExactPredicateKernel.h`. Arithmetic is instantiated at eight
and 144 uint64 limbs. Production wrappers always select the adaptive entry; there
is no public mode, global/thread-local switch, caller validity flag, cached input
certificate or retry after a narrow result. Each predicate derives its domain
from exactly the finite binary64 coordinates it consumes. Qualification's separate
wide adapter is linked only into a different test executable.

The domain traversal extends the original `MinimumExponent` loop: signed zeros
are ignored, all-zero input retains exponent zero, and every other significand is
aligned to the minimum stored exponent. Let B = 53 + (maximum - minimum) for
nonzero inputs, or B = 0 for all-zero input. Every aligned signed integer has
magnitude strictly less than 2^B. Finite binary64 gives B <= 2098, including
subnormals. `Decode` itself is unchanged; exponent 972 identifies a nonfinite
input, which selects the original 144-limb arithmetic with its original common
scale and behavior. There is no new nonfinite rejection in this optimization.

With D = B+1, coordinate differences have magnitude <2^D. Sufficient bit bounds
for every intermediate (not only the final sign) are:

| Operation | Bits |
|---|---:|
| Difference of coordinates | B+1 |
| Product of two differences | 2B+2 |
| Two-product minor / Orient2D | 2B+3 |
| Three-component dot of differences | 2B+4 |
| Difference of those dot products | 2B+5 |
| Orient3D determinant | 3B+6 |
| DirectedTriangle determinant | 3B+5 |
| ClosestStratum quartic determinant | 4B+9 |

The directed predicate includes its direction coordinates in the same domain;
its direction is not a coordinate difference. The closest-region predicates
multiply two degree-two dots, subtract two such products, and do not multiply
`at_b` or `at_c` differences. Therefore the single conservative B<=125 selector
fits every predicate in 512 bits (4*125+9=509). The full finite-input bound is
8401 bits, below 144*64=9216. No dyadic time-depth factor is appropriate here:
these functions consume already represented binary64 points, not sampled paths.

All arrays remain completely zero-initialized. Significant-limb loops,
normalization, sign rules, overflow flags, null-output early returns and Ericson
region/tie order are unchanged. `Negate` still receives its own value copy. The
unsigned128 multiply accumulator fits because
(2^64-1)^2 + 2*(2^64-1) = 2^128-1. The existing normalized `used` lengths and the
product bound prevent an out-of-range high-limb product or carry. Partial unsigned
products cannot exceed their complete product. Add carry and subtraction borrow
semantics are unchanged; no hidden heap allocator or new arithmetic backend is
introduced. Wide inputs retain the existing implementation exactly.

The possible benefit is smaller complete zeroing, value copies and temporary
working storage: the limb array is 64 bytes rather than 1152. Arithmetic loops
already use only significant limbs, so this is not an eighteen-fold speed claim.
Compiler inlining, stack-slot reuse, dispatch cost and the measured fraction of
runtime in these predicates determine any actual gain. Worker stacks, retained
arenas, resource caps and discovery scheduling remain unchanged.

Qualification reuses the existing unbounded rational conversion/vector helpers
and Voronoi oracle structure. It compares forced-wide and adaptive signs and
strata, all closed-region ties, input exponent thresholds, cancellation/carries,
zeros/subnormals, uniform scales, full exponent mixtures, nonfinite fallback and
null-output behavior. Separate binaries link the same discovery implementation
against each adapter and compare complete fieldwise reports/publications across
success, failures, previous-publication retention, masks and worker counts. These
are qualification-only entry points; a successful test does not grant physical
contact or vehicle acceptance authority.

## Source checkpoint and next qualification

At the 2026-09-24 source freeze, independent review found no arithmetic or
qualification-harness blocker. The 14 integer helper bodies and four predicate
arithmetic/tie bodies were checked against selected `b9957e84`; seven primitive
oracle/differential groups and the 314-record complete discovery corpus are
written but unbuilt and unexecuted. No speedup or vehicle acceptance is claimed.

The owning CMake module remains `lib_utest/qualification/fixed_triangle_features`.
Its existing host/source/shape checks gain the primitive groups; the separate
CTest `fixed_triangle_exact_storage_discovery_parity` compares the adaptive and
wide drivers. Bazel exposes `host`, `source_proof`, and
`exact_storage_discovery_parity` in the same qualification package. Discovery
comparison runs are sequential, bounded and retained in fresh directories;
Bazel prefers its undeclared-output artifact directory. No production flag enables
the forced-wide adapter.

Regression qualification must also cover current regularity's DirectedTriangle
consumer and continuous-contact Orient3D callers, plus existing transaction and
frozen geometry/publication replays. Any performance decision waits for real
contact-substage evidence. Measure a warmed persistent discovery cohort against
actual selected `b9957e84`, with matching flags, inputs and CPU affinity, timing
only discovery work. JSON serialization, test startup and the new forced-wide
qualification adapter are not performance baselines. No heavy work is authorized
by this document; the active run and workstation lane remain controlled by root.
