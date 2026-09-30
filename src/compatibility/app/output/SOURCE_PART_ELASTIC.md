# Original source-part elastic archive and replay

`robo_dyna.source_part_elastic_artifacts.v1` is the bounded original Yaris
PID 2000157 experiment: 117 source nodes, 88 QEPH parents and 6 native T3 parents.
The display mesh contains 182 triangles. Q4 display diagonals do not turn the
mechanics into triangles. Source EIDs, node IDs, ordered connectivity and each
family's native index remain in `configuration.json`.

The experiment uses original coordinates, density and thickness, with explicitly
experimental LAW1 elasticity. Original MAT024/ELFORM2 behavior is not reproduced.
The source's nodal rigid groups and unresolved tied scope are unapplied input
metadata. This archive describes a free part, without wall contact. The existing
pinned source loader authenticates the actual input; the replay reader checks
archive identity and geometry association, and does not reimport the full deck.

`SourcePartElasticArtifacts` composes `NodalMeshOutput`, `ArtifactInventory`,
`CsvLedgerWriter` and `MeshArchive`. It owns output cadence and presentation data,
not nodal mechanics, force histories or a clock. `SourcePartElasticFields` handles
serialization. The generic `SurfaceBindingFields` utility is shared with the
existing coupon/guided writer; `SurfaceBinding.h` contains the common plain
source/topology types without any TL solver or rendering dependency.

Each accepted frame archives exact endpoint positions and wxyz orientations,
raw velocity and world angular velocity, and the actual `NodalStamp` scheme,
velocity phase/time, preceding base time/epoch and kick duration. Epoch zero has
physical collocated zero velocities; the first accepted step has a half kick;
subsequent raw velocities belong to the preceding midpoint. Sparse saved frames
retain their actual preceding interval through the contiguous CSV ledger. The
separately named synchronized velocity arrays are derived from the complete
endpoint right-hand side and are never substituted for the raw owner fields.

Diagnostics include native family work, QEPH viscous work, carried kinetic
energy with total native J and its separate physical/added partitions,
synchronized kinetic energy, actual external kick/drift work, absolute external
drift work, work/energy residuals and deformation/geometry metrics. Full private
material histories and restart support are outside this visualization archive.

The shared ledger writer segments output before the existing 32 MiB per-file
limit. Forecasts reserve 702 bytes per interval, 256 KiB per saved frame and
128 KiB per field file, and 1 MiB for configuration/index/final output, within the 256 MiB aggregate cap.
The initial frame, epoch one, requested cadence and final frame are supported,
with one extra last-accepted frame reserved for a failed run. Such a run retains
partial files and `failure.json`; it never receives a completed replay manifest.

The existing `AcceptedReplay` reader and Chrono/VSG viewer handle completed
source-part bundles. Default rendering uses physical coordinates. For a small
elastic signal, an explicit presentation-only option is available:

```
robo_dyna_replay BUNDLE --capture NEW_DIRECTORY --require-frames N \
  --fps 25 --deformation-scale 25
```

Only this source-part kind accepts magnification. The scene computes
`X0 + scale * (accepted_X - X0)` into a separate mutable display mesh. The input
archive and raw fields remain unchanged. The overlay and capture manifest record
the factor and law; an immutable gray wireframe shows the original source outline.
The fixed camera uses bounds over all transformed saved
frames. Playback timing remains the saved frame cadence; actual simulation time
is visible in the overlay. The viewer executes no mechanics.

The source replay test executable requires
`ROBO_DYNA_SOURCE_PART_REPLAY_FIXTURE` to name a completed accepted bundle. It
checks original-part counts, epoch zero/one/sparse/final streaming, rehashed
phase/source/connectivity/mass/geometry faults, and preservation after rejected
load/reopen. The existing scene test additionally checks explicit magnification,
input immutability and rejection/retry. Actual VSG capture and MP4 full decoding
remain runtime evidence, recorded separately from these tests.
