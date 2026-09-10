# Bounded native shell reference collections

This extension prepares immutable typed reference collections. All eight new
collection functions and eight unchanged single-pair functions pass in the
owning host target; their numerical budgets remain unchanged. No resident batch, nodal owner,
contact, clock or force/history capacity changes are part of this patch.

`elements/ShellCollectionLimits.h` declares 128 physical nodes and 128 **total**
Q4/T3 parents. These are collection bounds, not separate allowances of 128 per
family in one input. Both typed storage arrays have fixed capacity so no heap,
allocator or variant framework is needed. Actual host layout remains unmeasured
until the owning root probe/build; this contract makes no device allocation
claim. The existing `MaxShellBindingNodes=7` remains the old pair/scratch bound.

`ShellBatchCollectionInput` borrows typed pointer/count ranges for the duration
of `Initialize`. Zero count requires null pointer. At least one family is
present; one-family preparation is allowed independently of later joined
participant policy. Each typed record carries its original native input,
ordered global connectivity and a nonzero source-parent ID unique across both
families. Counts are bounded before range access or count addition. Callers own
the stated valid input ranges; the builder copies every accepted field.

All connectivity is checked before native startup. Fresh qualified QEPH and T3
startup producers prepare every reference before shared node registration and
complete dense coverage checks. Source node IDs and represented coordinate bits
must agree at shared global indices, including signed zero. One source ID may
not name two global nodes. QEPH IDs retain their native uint32 type and widen for
comparison; T3 uint64 IDs are never truncated. No overlap, attachment, connected
graph or wall-projection policy is inferred here.

Mass and inertia reductions execute QEPH parents first, then T3 parents, each in
input and local-node order. Native total inertia is accumulated directly,
separately from physical thickness and added-area partitions. The native T3
angle-weighted masses are not normalized or replaced by equal thirds. Invalid
native input, identity, coverage or an unrepresentable sum preserves the entire
unprepared destination. Successful preparation is immutable and owns its data;
a failed call can be retried with valid input.

The indexed const accessors expose counts, prepared reference, connectivity and
source-parent ID for each family. Consumers must validate their exact counts
and indices. Invalid indices return empty unprepared values/source ID zero;
the old zero-argument accessors return empty unless there is exactly one parent
of each family. They never silently select the first parent of a collection.

The old `ShellBatchBindingInput` overload forwards to this same builder with
absent parent IDs and its original 4..7-node restriction. Its complete 49-word
inventory values and ordering remain exact. New collection inventories use
encoding discriminator 2, node/Q4/T3 counts and ordered per-parent words:
family, arity, source-parent ID, then global index/source-node ID/three binary64
coordinate words per local node, and the four original material scalar words.
`inventory().words()` is now a read-only pointer/active-length view, not a
49-element array reference. `WordCount=49` names the legacy length only;
consumers use the active size or full inventory equality, never a 49-word
prefix. Equality includes active length and the complete fixed storage (unused
words are zero). This is an in-process identity encoding, not a file schema,
hash, owner token or publication receipt.

Eight added host functions freeze the following checks before execution:

- Every original pair inventory word and native scalar sum is unchanged.
- A synthetic connected 117-node/94-parent fixture contains 88 nonuniform flat
  rectangular Q4s and six scalene T3s. Every node is checked using independent
  long-double world-triangle areas and world-edge atan2 angles, plus fresh native
  startup scalar sums in the exact declared order. Node 116 has a wide source ID.
- All 128 nodes and all 128 parents (each family separately) are exercised.
- Last-family parent IDs, coordinates, materials and order affect the inventory;
  active lengths differ when parents are omitted.
- Oversized/null/count inputs, zero/duplicate parent IDs, late/high-node ID and
  position mismatches, uncovered nodes, late native startup failure and finite
  union overflow preserve output and permit a clean retry.
- Prepared data owns its input, rejects replacement and fails closed on invalid
  indexed and zero-argument collection access.

The 117/94 mesh is explicitly synthetic. The authenticated original source
fixture belongs to robo-dyna and is not a dependency of this TL host target;
no source authenticity or original material-card equivalence is claimed.
Fixture material values vary by parent. Independent dimensional tolerances are
unchanged from the startup/pair gate: `2e-12*(dimension+abs(truth))`, with
`dimension=abs(truth)` for these positive mass/inertia quantities. Exact native
reduction and identity checks have no added tolerance.

Owning registration: add `ShellBatchCollectionTest.cpp` to the existing
`shell_batch_binding_check` executable and export the new limits header from
the existing binding library/target. There is no new runner or native solver
runtime. All eight original plus eight new host functions should run together;
resident consumers are a separately owned implementation stage.

Root execution evidence: `crash-work/reports/shell-collection-{build,tests}-1.*`
and the tests-1 XML directory. Build elapsed 18.393 s with 704,290,816 B sampled
peak RSS; the sixteen-function host run passes in 0.268 s under one CPU/1 GiB.
The authenticated original-source adapter also passes two new functions and
nine affected source regressions under `source-shell-collection-tests-1/`.
It preserves all 117 original nodes/88 Q4/six T3 and independently checks native
structural mass 0.25650893888187326 kg and total J 5.1937904054349167e-6 kg*m^2.
That is host startup only; resident/contact capacity and part dynamics are
separate integration gates. No original MAT024 or attachment equivalence claim.
