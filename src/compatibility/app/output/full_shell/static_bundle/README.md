# Full-shell source mapping foundation

This layer authenticates canonical source data and writes/reads an immutable
topology mapping. The layered `SourceBundle` also carries the original source
key, all 17 arrays and original metadata in a bounded static package. It does
not publish a completed simulation, assign native formulations, own nodal state,
or add replay dispatch. See [SOURCE_BUNDLE.md](SOURCE_BUNDLE.md) for the separate
static-only schema and exact measured payload.

`CanonicalSource::Read` requires explicit expected byte counts/SHA256 for the
canonical manifest, source-scope report and original extracted source member,
plus caller-approved units and tire policy. It authenticates before parsing
once, retains original metadata bytes and all 17 original canonical arrays,
and checks their exact existing dtype/shape/field conventions. It verifies
original node/connectivity associations, unique IDs, canonical versus scope
PID/MID/SECID tables, source section/ELFORM, and complete retained/omitted shell
and incident-node coverage. The original report supplies explicit tire PIDs;
this layer does not classify parts by titles or parse source keyword cards.
Beam/solid arrays remain complete source evidence, without invented zero mass
or runtime admission.

`PreparedSourceMapping::Prepare` takes the actual caller's ordered canonical
node and parent indices plus opaque native-family/local indices and native
point/applicability declarations. It requires every retained shell and exactly
its nodes once, validates bounded native indices/point declarations, and owns
immutable values. Caller ordering may differ from canonical ordering. The
native family is never inferred from source ELFORM or material ID. Its meaning
and correspondence to the actual native batch remain caller obligations.

The eight little-endian arrays preserve original IDs and original connectivity:

| Name | Representation |
| --- | --- |
| node_source_ids | `<u8>[N,1]`, original NID |
| node_canonical_indices | `<u4>[N,1]` |
| parent_source_ids | `<u8>[P,4]`, original EID/PID/MID/SECID |
| parent_reference | `<u4>[P,3]`, canonical index/native family/family-local index |
| parent_points | `<u4>[P,3]`, source ELFORM/native count/PLA applicability |
| parent_nodes | `<u4>[P,4]`, original local connectivity including T3 repetition |
| triangles | `<u4>[T,3]` |
| triangle_parents | `<u4>[T,1]` |

The display split is the existing source-surface policy: Q4 `[a,b,c]` and
`[a,c,d]`; T3 `[a,b,c]`. Source T3 must use its original repeated n3/n4. Geometry
is neither transformed nor welded. Physical coordinates remain in the retained
original SI canonical position array, selected by the exact node mapping.

`MappingDigest` is the single identity authority. It hashes a length-prefixed
domain `robo_dyna.full_shell_source_mapping.digest.v1`, the eight ordered array
names, dtype, rows, columns, field count/names, payload byte count and content
SHA256. Each count/string byte length is unsigned64 little-endian. Paths and
run IDs are excluded; native C++ structure bytes are never hashed. A standalone
Python packing/hash oracle freezes the encoding in the value test.

`MakeFrameContext` fills source-scope and mapping identities after the caller
supplies actual owner/run/topology/source-instance/configuration/qualification
IDs and fixed dt. Conflicting supplied source identities reject. It reuses the
existing frame Context and unchanged point-layout digest. Unavailable or
inapplicable fields remain distinct from zero PLA; zero native points are legal
for those declarations. Constructing a context is not a live acceptance token.

`PlanMappingRecord` returns exact metadata bytes and all nine file reservations.
`WriteMappingRecord` checks the whole reservation and every destination before
writing, and returns a descriptor only after success. Partial I/O leaves
create-only evidence and no successful descriptor; the same paths cannot be
reused. Calls/filesystem access are externally serialized, as with ArtifactIO.
`ReadMappingRecord` requires an already authenticated CanonicalSource and an
external expected mapping digest. It stages every array, regenerates semantic
mapping through the same source authority and returns only after equality.
Thus even a rehashed wrong MID or display association cannot replace source
semantics. This mapping descriptor is not a run completion manifest.

For the original no-tire report, 359,785 nodes, 349,645 parents and 677,989 display
triangles use exactly **40,339,684 mapping payload bytes**. All 17 original arrays
use 45,415,582 bytes. Arrays, original 42,846,753-byte member, 13,175,122-byte scope
and 2,632,394-byte canonical manifest plus mapping payload total 144,409,535
bytes before mapping metadata and other static obligations. The final publisher
must measure every file/reservation against the 192 MiB static budget and shared
2 GiB full-run forecast. A reserve is not proof that source/configuration/wall
and completion metadata are exhaustive. The source-copy layer writes the key as
33,554,432 plus 9,292,321 bytes and measures both mapping and bundle descriptors.
The exact qualified static bundle is 144,417,279 bytes across 31 files.

The host budget preflights source/array/metadata payloads and conservative
working copies. Independent author checks are restricted to one CPU and a
512 MiB process address-space limit. Six new GTest functions cover an independent
digest oracle, resource rejection before missing input reads, all actual
canonical arrays roundtripping byte-identically, complete reversed source
ordering, V1 scope units, late input rejection/retry, rehashed MID corruption,
staged read preservation, truncation, exact metadata-size admission and a real
partial file write. The actual-source tests use an explicitly opaque fixture
family with all native fields unavailable: source/output coverage evidence,
not a full-shell material, accepted dynamics or crash-physics qualification.

```sh
cmake -S output/full_shell -B <build> -DChrono_DIR=<install>/lib/cmake/Chrono \
  -DROBO_DYNA_SOURCE_MAPPING_CANONICAL_FIXTURE=<canonical-assets> \
  -DROBO_DYNA_SOURCE_MAPPING_SCOPE_FIXTURE=<yaris-full-shell-scope-6.json>
cmake --build <build> --parallel 1 --target robo_dyna_source_mapping_check
ctest --test-dir <build> -R full_shell_source_mapping --parallel 1 --output-on-failure
```

The tiny fixture driver only extracts and hashes the pinned original ZIP member.
It is test setup, with no geometry/material parser or runtime logic.
