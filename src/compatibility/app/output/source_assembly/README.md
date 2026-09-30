# Complete source assembly accepted output

This boundary prepares the original six-part Yaris assembly for accepted mesh
output: **1030 source nodes, 804 QEPH + 111 T3 parents, 1719 display triangles,
915 complete three-point plastic sections**. The inventory remains the immutable
`robo-dyna.source-assembly-inventory.v1`, 1731843 bytes, SHA-256
`afbc9cc6b9cbbceec766e1aa106b548fcc7468ce1a0d902d5d7b69afb1873d00`.
It does not integrate mechanics, activate attachments, apply contact, choose a
timestep, produce a crash trajectory, or declare a new replay bundle readable.

## Component boundaries

| Component | Responsibility and reuse |
|---|---|
| `SourceAssemblySurface` | Immutable source/display association from the already authenticated source; retains complete source ownership. No shell reference or mass rebuild. |
| `SourceAssemblySectionFields` | Validates exact family views, preserves all 3 x 7 point values, native section diagnostics and reported thickness; serializes original source parent/part/material/section/curve IDs, original ELFORM and family index. |
| Existing `NodalMeshOutput` | Live-owner accepted-only capture and Chrono surface publication, now with bounded active storage; retains complete owner stamp including velocity phase and rigid-group association. |
| `SourceAssemblyAcceptedOutput` | Composes source, the existing joined shell publication and nodal output. Stages both families against the live accepted stamp; checks every source reference/history endpoint before output selection. |
| Existing `AcceptedSurfaceMesh`, `SurfaceBindingFields`, `MeshArchive` | Physical-scale SI mesh, full source topology fields, full-precision Chrono JSON and OBJ. No replacement serializer or scene engine. |
| Existing `ReplayParentScalarColors` | Maps each native parent maximum plastic strain to its display triangles; both triangles of a quad use one source value. |

`SourceAssemblyAcceptedOutput::Initialize` consumes the engine's existing
`SourceAssemblyBindings`. The engine remains responsible for authenticating that
these are the same source/material bindings used by its joined Q/T participants.
An active owner rigid-group descriptor must exactly match this binding's source
instance, group count and member count. Only an all-zero descriptor admits the
explicit inactive-group qualification scope; partial descriptors are rejected.
Each `Publish(owner, qeph, t3, publication)` reads only accepted APIs, with no
owner/batch pointer retained between calls. It checks native reference identities
and accepted history stamps for every parent. Readback of a missing or foreign
participant cannot partially update output. A prepared candidate cannot advance
output; failed/rejected candidates leave the prior output visible. Successful
mesh publication is followed only by infallible selection of staged output
sections, colors and the owner engine's supplied native diagnostics.

The formatting helper deliberately accepts ordinary borrowed `SectionView`
values; formatting alone is **not proof of acceptance**. A future bundle writer
must take those views from `SourceAssemblyAcceptedOutput` after a successful
publication and bind its document to that same full `NodalStamp`.

All geometric coordinates remain canonical world metres at scale 1. Attach the
existing Chrono shape to an identity-frame fixed visual carrier. Native work,
plastic work and isotropic kinetic partitions remain explicit diagnostics; this
module constructs no connected-system energy or constraint-work formula. Plastic
work must not be added again to native shell work. Actual rigid-group recurrence
and its energy/admission gates belong to the TL owner and the case engine.

## Storage and failure contracts

The shared nodal adapter retains its legacy 128-node default. The explicit
`NodalCaptureLimits` supports at most 2048 nodes and 512 KiB of capture payload.
It allocates exactly two active arrays at startup: 6 doubles/node each for a
translation-only owner, or 13 each for x/v/omega/q. The assembly's 1030-node
extended capture therefore uses **214240 bytes**. No per-step allocation occurs.
Both node and byte limits are checked before allocating or reading owner fields.

Assembly output separately limits source topology (2048 nodes/1024 parents,
1 MiB default derived payload), and section/native-result capture (4 MiB default,
8 MiB hard cap). Overflow-safe preflight uses TL's existing `BoundedArenaLayout`.
Shared authenticated source/native bindings, Chrono mesh allocations, allocator
metadata and CUDA owner/driver allocations are outside these named payload
ledgers; they retain their own existing bounds. Native result staging is one
active array per family. Sections/thickness/color values use two active captures.
The only accepted selector here selects **output copies**, never mechanics state.

Views and Chrono geometry must be externally serialized with stepping/rendering.
Exposed output views remain valid until the next successful Publish or destruction.
Initialization and failed publication preserve previously published output.
Document formatting and file writing occur at caller-selected output cadence and
may allocate; neither runs inside the physical step or native solver.

## Focused qualification

Configure this directory as a standalone project with explicit `Chrono_DIR`,
`ROBO_DYNA_TL_ROOT` and `ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY`. Target
`robo_dyna_source_assembly_output_check` runs eight host cases covering complete
actual topology/source IDs, physical-scale geometry and native mesh roundtrip,
all point/diagnostic values, existing parent colors, late invalid fields, and
atomic node/parent/byte admission. Distinct synthetic section values in these
host tests are mapping fixtures, not claimed mechanical results.

Opt in with `ROBO_DYNA_SOURCE_ASSEMBLY_OUTPUT_CUDA_CHECK=ON`, an explicit CUDA
compiler and architecture. Target
`robo_dyna_source_assembly_accepted_output_check` reuses the existing
`qualification/source_assembly/SourceAssemblyFlightFixture.cu` and runs two
actual-source GPU cases. It compares complete accepted nodal and family fields,
source references, native thickness/diagnostics, completed and rejected candidate
exclusion, late missing-family failure, exact one-byte-short limits, retry,
retained allocation counts and the same two reusable output addresses. It uses
three very short uniform-translation intervals in both the inactive-group scope
and with **all six internal groups active**. All ten host/GPU functions pass in
`source-assembly-output-tests-1`; this is accepted-output/free-flight
qualification, not a connected crash demo.

The root coordinator owns the legacy `tl_nodal_output_integration` regression
after this shared active-capture change.

## Next replay gate

The existing `AcceptedReplay` reader and `AcceptedReplayScene` dispatch explicit
case schemas; their source-part schema assumes the older single-part catalog.
This change does **not** mislabel the six-part assembly as that schema. Once the
connected case owns its accepted diagnostic contract, add one assembly bundle
kind with a completion manifest/inventory and full nodal timing, source/material
catalog and released-frontier declarations. Reuse generic `MeshArchive`,
`SurfaceBindingFields`, mesh validation and `ReplayParentScalarColors`; admit the
new kind to the existing fixed-carrier scene at physical scale 1. Then qualify
roundtrip provenance, truncated/modified/unaccepted sample rejection, complete
assembly plastic color mapping and Chrono scene publication before rendering a
real accepted connected mesh-wall trajectory.
