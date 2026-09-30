# Immutable mixed shell startup binding

**Source contract, frozen before first execution; unexecuted.** This gate adds
only a host startup producer for one QEPH Q4 and one native T3. It does not join
resident batches, grant a publication receipt, initialize a nodal owner, or
qualify mixed dynamics. The later shared-edge transaction remains subject to
[the resident design](../../../../../planning/T3_RESIDENT_BATCH_DESIGN.md).

`lib_src/elements/ShellBatchBinding.{h,cpp}` owns the complete immutable
reference inventory and nodal mass ledger. The two typed `ReferenceInput`
records pass through the existing qualified `qeph::InitializeReference` and
`t3::InitializeReference`. There is no second derived-reference validator and
no copied donor equation. The owning startup provenance remains in
[QEPH's source manifest](../../qeph/source-manifest.json) and
[T3's source manifest](../source-manifest.json), at their recorded OpenRadioss
pin `a62b27e6baa555d222a580d6218867d0be4d70b5`.

The reusable domain is exactly one Q4 plus one T3, with 4–7 dense global nodes
and 0–3 shared indices. Every declared node must occur; native rows contain no
duplicate index. Shared indices require equal source IDs and all three exact
binary64 coordinate representations, including signed zero. Different global
indices cannot use the same source ID. Distinct IDs at coincident coordinates
are permitted: no geometric overlap, attachment, tie or contact policy is
inferred. QEPH's uint32 source IDs are widened; T3's uint64 IDs never pass
through a floating-point or narrower integer representation.

Mass, native total isotropic J, physical thickness inertia and area-added
inertia are accumulated independently, QEPH first then T3, each in its native
connectivity order. The total J uses the producer's original expression;
diagnostic partitions are not recombined into it. T3's angle weights are not
renormalized. The four aggregate totals use the same seven-contribution order,
not a reordered sum over global nodes. Every positive contribution and every
intermediate total must be finite and positive; an unrepresentable union fails
even if both individual startup producers succeed. No reciprocal mass,
constraint, energy, load, state, clock or device storage is owned here.

Initialization is one-shot. All references, source associations, coverage and
sums are prepared in local storage; publication is a final assignment with no
fallible operation afterward. Every rejection preserves the caller binding
and inputs, including an already-prepared binding. Const getters expose the
owned results; assignment is disabled and copy construction preserves the
immutable value. Default getters carry no prepared authority.

The 52-word inventory contains an internal discriminator and node count, then
each family and arity, seven ordered `(global index, source ID, x, y, z)`
records, and density/thickness/E/nu plus placement for each family. Binary64 values use their
entire object representation. Equality compares every word. This is an
in-process identity encoding, not a persisted schema, reduced hash, owner ID,
or proof that two independently constructed experiments are physically equal.
Future joined participants must bind their actual references and sole owner
to this inventory before their publication coordinator can be implemented.

## Frozen fixture and checks

The shared-edge fixture has metres `(0,0,0)`, `(1,0,0)`, `(1,1,0)`,
`(0,1,0)`, `(1.75,.25,0)`. Q4 connectivity is `{0,1,2,3}`, T3 is `{1,4,2}`;
the areas are 1 and .375 m² and the T3 is scalene. Both sections use
rho=1024 kg/m³, t=1/32 m, E=2e6 Pa and nu=.3. Q4 source IDs are 100–103;
the T3-only node has ID `2^54+103`. No velocity, h, force or experiment energy
scale is required by this startup-only gate.

Eight host functions qualify:

1. Shared-edge native contribution order, separate physical/added/total J,
   nonuniform angle mass and the full-width T3 ID.
2. Dense disjoint/single-/two-/three-shared unions, coincident distinct physical
   nodes and a permutation of the global numbering.
3. Independent density, thickness and geometry scaling plus an exact cyclic
   coordinate permutation and rigid translation.
4. Complete inventory equality/difference for both families' four material
   fields, all five global source IDs, all 15 represented coordinate components,
   signed-zero identity, global numbering and native cyclic ordering.
5. Declared count, out-of-range/repeated connectivity, dense holes, shared ID
   mismatch, uint32-truncation collision and one ID mapped to distinct nodes.
6. Exact-position mismatch, nonfinite/material/source-ID failure and a late
   collinear T3 after valid QEPH startup, with native diagnostic forwarding.
7. Both producers valid at rho=`.75*DBL_MAX`, t=1 but their union mass not
   representable; complete rejected output remains unchanged and retry works.
8. Owned input lifetime, copy construction, const access and refusal to replace
   an already-prepared binding.

The independent oracle uses the existing test-only long-double world-edge
cross/`atan2` T3 oracle and two world-triangle areas for the declared flat Q4
fixtures. Its startup dimensional budget remains `2e-12*(dimension+|truth|)`,
where each positive mass/inertia truth supplies its own dimension. Scaling
comparisons use the same budget. Exact scalar native reduction order, identity,
and failed-output checks use **no tolerance**. Small coordinate variations used
only to probe identity may warp the Q4; they are checked against the actual
startup values, without claiming physical-area equality for a warped QEPH.
No native Fortran function or production force evaluation is invoked by these
host tests; the existing independent oracle is reused as a header only.

## Owning registration requested

Root owns registration and all execution. No existing build, source-manifest,
QEPH/T3 batch or native-reference file is changed by this source handoff.

* Production host library `tl_shell_batch_binding`:
  `lib_src/elements/ShellBatchBinding.cpp`, public header alongside; depends
  only on existing QEPH/T3 startup headers and their fixed math dependencies.
* Host target/test `shell_batch_binding_check`:
  `ShellBatchBindingTest.cpp` and its local `ShellBatchBindingFixture.h`,
  `GTest::gtest_main`, TL-root includes and the production library. The reused
  independent oracle header lives in `qualification/native/t3`; no native
  library or CUDA runtime link is required.
* C++17, `-fno-fast-math -ffp-contract=off`, no fast/reassociated summation.
  Register a bounded single-worker host test. No standalone project, driver,
  GPU target or new allocator is needed.

First execution and any source revision must be retained before promotion.
The next gate is joining typed resident participants and preflighting both
before one owner commit; this startup producer alone establishes none of those
transaction or numerical claims.
