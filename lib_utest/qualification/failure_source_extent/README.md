# Immutable failure-source prefix extent

Separate source-only candidate from TL92dd532e. FailureHostStorage::CheckActivitySources
keeps its shape-first checks and replaces N parent-null probes with the existing
last-parent probe for a nonempty prefix. Empty prefixes keep success without
reading a binding. No public API, field, allocation, source/law cache, numerical
history validation or readback changes.

The immutable binding owns complete contiguous family-index maps. Its parent
getter returns null only for no data, an unsupported family or an out-of-range
family index; no row policy/content is checked by that getter. Source proof pins
that implementation, immutable shared ownership and the exact initialization
that fills every map. It proves only this predicate changed in the entire
FailureStorage translation unit. No equivalent endpoint shortcut is claimed for
per-parent law/role validation.

Four host tests compare literal old/current method bodies (signature adapted
only, no body changes) on real initialized mixed and non-QBAT bindings, all
prefix boundaries, invalid families, empty input, shape failures, copy/move/source
lifetime and rejected initialization/retry. These supplement actual storage and
owner coverage; they are not a runtime owner proof.

Both actual-storage CUDA tests are required qualification: they call the real
FailureHostStorage Initialize/CheckActivitySources/CheckReadSources for QEPH/T3,
both slabs, shorter/full valid prefixes, lost original source lifetime, invalid
family/layout/missing extent, and repair on the same object. They fail without
a real CUDA device. No GPU/build has run for this authoring checkpoint.

Root gates: source proof; four host tests; both actual-storage CUDA tests; unchanged
qeph_mapped_activity and relevant compact T3 activity/PhysicalActivePrefix owner
checks; exact resource forecast; then matched uninstrumented vehicle cadence and
accepted output comparison. Keep this candidate separate from QBAT/solid tiles
until independently measured. Fewer getter calls is not yet a wall-time claim.
