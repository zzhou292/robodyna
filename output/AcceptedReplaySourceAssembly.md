# Assembly accepted replay

`AcceptedReplay` supports `robo_dyna.source_assembly_wall_artifacts.v1` through
its existing streaming reader and Chrono mutable-mesh scene. The displayed
surface contains all 1,030 original nodes and 1,719 triangles from the 804 QEPH
and 111 T3 parents. It uses physical coordinates, the authenticated placed
finite wall, and each parent's maximum recorded equivalent plastic strain
across its three native thickness points. The viewer labels six active internal
rigid groups and explicitly released external connections.

The source assembly has a separate typed sidecar. Single-part material fields
remain specific to the old schemas. `modelio/source_assembly/Reader.cmake`
owns the immutable inventory parser independently of shell/material startup.
`ReadBytes` authenticates the explicit expected size/hash before copying and
parsing once. Replay requires `ROBO_DYNA_TL_ROOT` (or the established
`CRASH_TL_FEA_SOURCE_DIR`) for neutral `Fixed3.h` declarations; it links no TL
solver, CUDA owner, material batch or native reference implementation.

The small reader modules separate complete source declarations, groups and
released frontier, placed-wall setup, segmented intervals, native sections,
accepted fields, kinetic bookkeeping and contact records. Existing artifact
I/O, bounded JSON values, CSV segmentation and placed-wall authentication are
shared. The pure `robo_dyna_source_assembly_wall_forecast` target owns the same
`PlanWallArchive` operation used by the writer and reader.

Admission authenticates every declared file, including the unchanged 1,731,843
byte inventory. It checks every source parent/material/section/curve/family
mapping, original ELFORM, all node and triangle mappings, complete internal
memberships and the released frontier. Schema-specific admission allows at
most 1 GiB including the manifest, 32 MiB per file and 1,000 frames; the declared
smaller caps and exact shared forecast are also enforced. The file inventory
must contain exactly the required static files, indexed accepted frames and
declared interval segments. An extra canonical wall cannot replace the placed
wall selected by this schema.

The complete CSV pass authenticates every accepted epoch/base/attempt and the
stored midpoint phase, even when only sparse endpoints are saved. Saved frame
fields must match that interval and the complete source map. Section values,
quaternions, native coefficient/kinetic partitions, stable kick bookkeeping,
contact node/parent identities, projected finite-wall faces, moments, surface
power, local work relations and cumulative contact counts are checked before
publication. `Open` and `Load` preserve the previous complete reader state on
failure. `Load` does not interpolate rejected or missing samples.

These are source, representation and recorded-arithmetic checks. Replay does
not rerun mechanics, reconstruct a collocated constrained state, certify the
physical trajectory, or infer independent global energy conservation. Native
shell work, plastic work, constraint reaction work and effective stored kinetic
remain their separately declared channels. In particular, a bookkeeping
residual is not an independent physical conservation certificate.

The owning tests are `robo_dyna_accepted_replay_source_assembly_check`, enabled
by the existing replay checks option with an explicit
`ROBO_DYNA_SOURCE_ASSEMBLY_REPLAY_FIXTURE`. They use the real completed archive
for valid input and isolated hard-link clones for rehashed corruptions;
replacement first unlinks the clone entry, preserving the original artifact.
They cover all source families, late source/layer faults, groups, phases,
contact arithmetic, original inventory authentication, extra-wall rejection,
exact forecast/cap admission and failure preservation. A separate synthetic
CSV-only test exercises the declared 8 MiB segmented-file boundary; it is not
mechanical evidence. The existing scene suite includes the assembly kind's
physical-scale, actual-wall and mutable native-color dispatch.

## Qualified assembly replay checkpoint (2026-09-10)

App57d6805 passes the owning Release reader/scene build and32 functions in three
CTest groups (`source-assembly-replay-tests-1`). Appa8cdc88 strengthens the actual
scene oracle: both the514-frame yielding connector and65-frame connected assembly
stream through Chrono with independent archived coordinate bits, source EIDs,
three-point plastic histories, fixed wall and face-color checks. All three
selected scene functions pass (`source-assembly-actual-scene-tests-1`).

The actual RTX5090 capture contains all65 accepted assembly frames through
15.258789us. `crash-work/renders/source-assembly-wall-diagnostic-1.mp4` is a
physical-scale integration diagnostic:6.5s presentation,10FPS,1280x720 H264,
98,005B. All PNG hashes, video probe and full decode pass, and decoded initial/
final images were inspected. Motion is barely visible at this microsecond
horizon; the next visibly deforming connected-impact video remains separate.
