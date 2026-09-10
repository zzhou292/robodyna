# Full-shell static bundle design

The immutable canonical/source mapping and mapping-only record slice is
implemented, followed by the separate bounded source-copy/key-chunk layer.
See README.md and SOURCE_BUNDLE.md for qualified static-only scope.

Mapping base: `118e1d7`; source-copy base: frozen mapping `576861d`.
Separate source-copy worktree: `crash-work/worktrees/full-shell-source-copy`.
This is topology/source I/O, not mechanics admission or a completed simulation publisher.

## Ownership

New `output/full_shell/static_bundle` modules: `Types`, `CanonicalCatalog`,
`Scope`, `Topology`, `SourceChunks`, `Records`, and small descriptor helpers/tests.
One local CMakeLists, added by the existing full_shell CMakeLists. Reuse
BoundedArrayIO, ArtifactIO, FullShellIdentityFields and immutable frame Context.
No solver, viewer, Chrono scene, source keyword/material parser or runtime owner edits.

## Static authority and immutable preparation

1. Authenticate caller-supplied canonical manifest and scope-report bytes/SHA.
   Preserve their original bytes, complete tables and all 17 canonical arrays.
   Parse only canonical descriptor/geometry/ID tables and explicit scope metadata.
   Source-card interpretation remains the existing Python compiler's authority.
2. Validate the arrays against their descriptors and the canonical NID/connectivity
   association. Check positive unique IDs, node range, part MID/SECID references,
   raw source ELFORM and original three/four-node repetition pattern. Do not
   convert or omit beam/solid arrays; they are source evidence, not active mass.
3. The caller supplies actual global-node canonical indices, ordered parent
   canonical indices, opaque native-family/local indices and native point/applicability
   declarations. Check complete selected/excluded PID coverage, every retained
   original shell exactly once and exactly the incident nodes. Preserve caller
   global/parent order and authenticate it; do not assume canonical order equals
   a future native batch order.
4. Build immutable display topology using the existing source-surface policy:
   Q4 [a,b,c] + [a,c,d], T3 [a,b,c], exact source IDs and full physical coordinates.
   No vertex welding, geometry transformation or new tessellation heuristics.
5. Bind ordered mapping layouts/counts/content hashes with a versioned digest;
   exclude filenames and run/owner IDs. Use that digest for the existing
   Identity.source_mapping_sha256. A prepared mapping exposes the digest and
   constructs the existing frame Context once the caller supplies actual
   owner/run/configuration/qualification identities and fixed dt. The frame
   point-layout digest and native-family/application authority remain unchanged.

## Typed topology once

Eight little-endian arrays, all using existing canonical dtypes:

- source node IDs: <u8>[N,1]; canonical node indices: <u4>[N,1].
- source parent EID/PID/MID/SECID: <u8>[P,4].
- canonical parent/native family/family-local index: <u4>[P,3].
- source ELFORM/native points/PLA applicability: <u4>[P,3].
- original local shell connectivity: <u4>[P,4], including repeated T3 node.
- display connectivity: <u4>[T,3]; triangle-to-parent index: <u4>[T,1].

For no-tire N=359785/P=349645/T=677989 this totals 40,339,684 bytes.
Native-family values and point applicability are supplied, never inferred from
material IDs or source ELFORM. Resolved rigid/nonplastic records may have zero
points. Missing unavailable PLA is distinct from zero plastic strain.

## Source chunks and accounting

The authenticated original vehicle member is 42,846,753 bytes. Chunk it as
33,554,432 + 9,292,321 bytes, with exact ordered offsets, per-chunk SHA and the
complete member SHA. Use ArtifactIO read/hash/create-only write operations;
no archive extraction library or new digest implementation. Caller supplies
an already extracted bounded source file/bytes. Tests can use the existing
Python zipfile path to prepare the exact pinned member as a fixture.

Known static inputs: 45,415,582 canonical array bytes, 42,846,753 source bytes,
13,175,122 scope bytes, 2,632,394 canonical-manifest bytes, and ~40.3 MB mappings.
Pinned reference/README, static descriptor, configuration/index/manifest,
per-frame metadata and other declarations must each be measured or explicitly
reserved too. The 192 MiB reserve is a cap, not proof of exhaustive listing.
Pass every actual static file extent + declared remaining reservations into
PlanArchive before any output creation; retain the real dt/interval count and
selected frame cadence. Total remains at most 2 GiB and each file at most 32 MiB.

## Publication and readback

Prepare authenticates/stages values without creating outputs. Writing preflights
all destinations and the total plan, copies bounded inputs, rechecks source bytes
at copy, then returns a static descriptor only after every file succeeds.
Failure leaves create-only incomplete evidence and no success descriptor; it
cannot imply a completed run. Reader accepts a caller-authenticated descriptor
identity, stages source/array/topology checks and publishes an immutable value
only after the final check. It does not create an accepted nodal state or certify
source-to-native material equivalence. Full run manifest and replay dispatch
remain separate integration tasks.

## Qualification

- Small independent source geometry and reordered runtime-index mapping controls.
- Wrong source/units/PID/MID/SECID, duplicate/omitted/late parent/node and source
  incidence errors; rejected/unknown native point applicability and digest change.
- All 17 actual canonical arrays roundtrip byte-identically, with explicit
  original no-tire coverage and original source IDs/PID palette provenance.
- Exact 32 MiB source split, whole-hash mismatch, reversed/gapped chunk,
  truncation/rehashed semantic tail, late destination and partial write failure.
- Previously visible read result unchanged; no successful descriptor after
  failure; exact measured static budget and one-byte overflow before files.
- Host-only under one CPU/512 MiB; synthetic native mapping is explicitly
  formatting evidence, not actual full-shell material/runtime admission.
