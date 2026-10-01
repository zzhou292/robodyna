# Genuine renamed-type archive probe

`write_legacy_probe` compiles only `LegacyTypes` against the unmodified factory
and archive implementation. The root operator generated and froze its three
archives before enabling the new naming support; `fixtures/manifest.json`
records that producer and the relevant unchanged production inputs.

The reader proof uses `robodyna::serialization_compat::RbRenameProbe` and
`RbRenameProbeBase`, with the same physical layout as the legacy types. It does
not link or include the old definitions and does not use type aliases. The
abstract serializable base is the second base, so a correct factory result must
adjust its pointer and retain the original shared ownership control block.

Compatibility requires:

- One canonical factory tag, `ChRenameProbe`, for the renamed derived type.
- The old `ChRenameProbe` / `ChRenameProbeBase` conversion graph identifiers.
- The registered derived version 5 and the unregistered base version 3.
- The captured base version identity `N6chrono17ChRenameProbeBaseE`, independent
  of the new namespace and RTTI spelling.
- Exactly matching old JSON, XML and binary bytes for the same object graph.

The named version helper is intended for previously unregistered bases or
template specializations. A registered type already derives its version key from
its canonical factory tag. Neither case requires a second registration or an
additional global naming registry.

Only the named probe is qualified by these fixtures. Every real type family
still needs its own old archive samples and affected subsystem tests before an
implementation rename. Existing binary portability limits are retained.
