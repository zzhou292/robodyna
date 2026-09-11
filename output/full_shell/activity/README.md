# Accepted parent activity records

This optional record is separate from the unchanged position/native-PLA V1
record. Its schema is `robo_dyna.full_shell_parent_activity.v1`; the one-time
declaration sidecar uses `robo_dyna.full_shell_parent_activity_declaration.v1`.
Absence of this capability supplies no activity information. Zero stress, PLA,
point failure, and contact-surface eligibility cannot supply a parent flag.

`ActivityRecord::Create(context, input, expected_stamp)` accepts an exact-length
`uint8_t` input with values only 0 (inactive) or 1 (active). Native pending OFF0.8
is not an accepted state. Complete `Identity`, `point_layout_sha256`, and the
entire `FrameStamp` must match the caller's expected Context/accepted phase before
any input value is read. The existing point-layout digest already binds original
parent IDs/order, family, source ELFORM and point applicability; it is reused
without inventing a second mapping digest.

The immutable record retains the shared Context and packed values. Copy/move
construction preserves lifetime, including the moved-from handle; assignment is
deleted. No accepted owner, producer, live-state authentication, activity
recurrence, mechanical clock, or complete archive publisher is introduced here.
The caller remains responsible for supplying actual common-commit data and an
authenticated expected frame association. A fully rehashed, consistently
redeclared valid bit pattern is new content, not evidence of live acceptance.

## Encoding and I/O

The existing bounded codec has UInt64 but no UInt8. Activity is packed into
`ceil(parents/64)` little-endian UInt64 words, bit `i%64` of word `i/64` for source
parent index `i`. Metadata declares `encoding:uint64_lsb0`,
`activity_scope:parent_mechanical_activity`, and `phase:accepted_endpoint`.
Unused high bits in the last word must be zero, even after a supplied file is
rehashed. The descriptor has one column named `parent_activity_bits`.
349645 parents use **43712 bytes**, including six extra bytes compared with the
minimum byte-packed bitset. Full-word boundaries do not shift by 64.

`WriteActivity(root, stem, record)` creates `<stem>.activity.bin` followed by
`<stem>.activity.json`. It serializes and checks both destinations before either
write; metadata is last. `ReadActivity(root, context, record_file, expected_stamp)`
authenticates exact metadata/value hashes, lengths, schema, scope, layout,
phase and padding before publishing an immutable result.

`WriteDeclaration`/`ReadDeclaration` bind the same identity, layout, counts,
fixed timestep, encoding and allowed states. This small activity-semantics
sidecar is distinct from the actual vehicle source-declaration sidecar. Each
frame also carries its complete association, so the record reader does not
depend on a mutable external schema registry. A future archive owner must list
the optional capability and both records in its authenticated inventory.

Calls use the existing externally serialized create-only ArtifactIO contract.
An I/O failure may leave incomplete evidence. There is no completion manifest,
overwrite, accepted-prefix publication or claim that a failed write succeeded.

## Budgets and qualification

Before input/readback buffers are examined, activity startup charges retained
Context payload once, record objects, the input extent, four packed-buffer
reservations and bounded JSON scratch. `Context::retained_payload_bytes()` uses
checked actual vector/string capacities; allocator/control-block overhead and
process RSS are separate. Defaults allow 64 MiB; explicit host caps up to 512 MiB
may be supplied. The 349645-parent synthetic capacity fixture charges 14773040 B
and passed under a 512 MiB process limit.

`PlanWithActivity(context, request, declaration_filename)` preserves V1
`PlanArchive`. It reserves packed bytes plus 16 KiB metadata for every saved frame
and the extra accepted-prefix frame, consumes 16 KiB explicit static reserve for
the activity declaration, and counts two extra files per frame plus the sidecar.
The static reserve is already fully charged; the sidecar consumes its available
space rather than being charged twice. This first profile rejects other optional
frame channels; optional interval records retain their existing accounting.

The full-count forecast test includes the known 144417279 B static package,
3648589 B actual vehicle declaration, and named wall/config/manifest/index
reserves within 192 MiB. Bounded placeholder package slots are forecast fixtures,
not a new actual file inventory. The synthetic native parent/point table is a
capacity test, not original-vehicle runtime admission. Near-cap old plans must
be recomputed with activity; a former passing forecast can reject.

Owning host target: `robo_dyna_parent_activity_check`, CTest
`full_shell_parent_activity`. Standalone configure `output/full_shell/activity`
with `Chrono_DIR`, `ROBO_DYNA_FULL_SHELL_RECORD_TESTS=OFF` and
`ROBO_DYNA_SOURCE_MAPPING_TESTS=OFF`. Tests cover full count/order/lifetime,
poisoned-input preflight, late invalid flags, exact retry, expected-phase
association, 12 metadata/payload corruptions, rehashed padding, truncation,
sidecar semantics, late destination collision, I/O failure and full forecast.
They do not claim live failure capture or Chrono activity visibility. That next
integration must stage actual activity with geometry/fields at the same common
commit and adopt an explicit display policy while retaining original topology.
