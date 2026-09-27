# Source-declared solid controls and native packet identity

`Selection` owns complete semantic source rows and a full solid packet roster.
It joins each source EID to the existing coefficient parent family/local index;
no PID table, distinct-node heuristic, CUDA block size or manufactured packet
partition is used. Source file, binary and export authentication belongs to the
app evidence, not this TL module. Success validates the supplied complete roster;
it does not establish actual native vehicle provenance by itself.

The default `LegacyNoStructuralIcontrol` profile has no optional arrays or
per-parent allocation. `SourceDeclared` requires source-instance equality, an
explicit SI or mm/Mg/s working-unit system, NVSIZ and compiled MVSIZ, all model
parent source PID/SID/MID/native-property/ICONTROL rows, all native processor
partitions (including empty solid partitions), all solid packets including IC0,
and every ordered source EID exactly once. Native property/MID identities are
resolved external identities; the app resolves raw native internal indices.

Partitions have contiguous zero-based PROC IDs. Within each partition, NFT
starts at zero and successive NEL extents cover its complete solid roster.
Packet NG IDs preserve strictly increasing source order; gaps in NG IDs can
represent other element types, but solid NFT gaps are rejected. Member arrays
preserve native packet order and retain source EID plus TL family/local/combined
indices. Packet family, MID, native property and ICONTROL must agree with every
member. Unsupported enabled families, missing/duplicate elements, unexplained
offsets, vector overruns and incomplete partitions reject before publication.
A native HEPH raw8 packet cannot silently join a model that projected its cells
to S6; the source/model family discrepancy must be resolved explicitly.

`ModelInput.controls` is optional. Model identity includes all selected rows,
units, native vector extents, partition/packet boundaries and member order.
The owned arena and startup identity-index/seen scratch are included in exact
payload forecasts. The new fixed Selection handle is charged through the
model's Impl sizeof even for legacy models; unchanged legacy byte size is not
claimed. Allocator bookkeeping stays outside the existing payload convention.
Inputs are borrowed during initialization only; published rows are owned.

The current resident planner explicitly rejects enabled controls until its
controlled family/history/owner implementation is qualified. This module changes
no force, accepted/trial history, owner clock or integration semantics.
The standalone host gate covers complete joins, source lifetime, omissions and
duplicates, empty partitions, exact/one-byte-short budgets, model identity,
legacy behavior and the real resident planner rejection. Actual exported native
packet membership remains a separate app/source gate.
