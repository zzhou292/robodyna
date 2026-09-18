# Robo-dyna

Robo-dyna composes TL-FEA mechanics and Chrono geometry, archives, and rendering
into a modular vehicle-impact application. The first delivery case is the
selected Yaris assembly hitting an authenticated rigid mesh wall, with actual
elastic/plastic deformation and part-colored playback. LS-DYNA-like CAE
functionality is the longer-term direction.

Start with the [architecture guide](docs/ARCHITECTURE.md) for source ownership,
data flow, and where to make changes. Use the
[Yaris delivery roadmap](docs/YARIS_DELIVERY_PLAN.md) for remaining capabilities
and the workspace [current execution plan](../planning/CURRENT_EXECUTION_PLAN.md)
for active branches, evidence, resource policy, and the next runnable gate.

## Source map

| Directory | Responsibility |
| --- | --- |
| `modelio/`, `models/`, `tools/` | Source declarations, authenticated model conversion, selected-model inputs, and thin preparation CLIs |
| `case/vehicle_runtime/` | Retained source backing, startup configuration, and resource forecasts |
| `case/vehicle_dynamics/` | One TL physical owner and ordered prepare/commit/discard composition |
| `case/vehicle_wall/`, `case/vehicle_self_contact/` | Contact setup and adapters into that same dynamics transaction |
| `case/vehicle_run/` | Explicit run profiles, CLI, accepted-step loop, bounded summaries, and stop/closure policy |
| `output/physical_frames/`, `output/physical_run/` | Accepted frame mapping, authenticated archive writing, and immutable replay input |
| `chrono/physical_run/`, `chrono/full_shell/` | Accepted samples to Chrono scene geometry, part colors, and native plasticity display |
| `viewer/` | Replay presentation, capture validation, video encoding, and postprocessing |
| `qualification/`, `benchmarks/`, module `tests/` | Formulation evidence, isolated measurements, and owning regression tests |

TL-FEA owns the state, constitutive histories, explicit stepping, contact
algorithms, and common physical publication. Robo-dyna configures and composes
those capabilities. Chrono owns the reusable geometry/visualization infrastructure;
the application does not advance a second Chrono dynamics clock.

## Current delivery boundary

The preserved 10,000-step / approximately 2 ms Yaris result demonstrates
wall-only impact and local plastic deformation. Its
[overview](../crash-work/renders/yaris-wallremoval-10000-review-1/overview-video/movie.mp4)
and [impact detail](../crash-work/renders/yaris-wallremoval-10000-review-1/impact-detail-video/movie.mp4)
remain the existing vehicle videos.

The historical first-profile self-only gate prepared, sealed, and discarded
one attempt; the combined wall+self gate committed one 200 ns interval.
Wall+self controller composition is implemented; the pending acceptance
checkpoint is two consecutive genuine 200 ns commits, a closed authenticated
archive, and exact Chrono replay. That checkpoint and a longer wall+self crash
are not yet accepted. GPU/heavy execution is currently paused at the user's
request; the workspace plan records how to resume.

The contact path is hybrid: CUDA mechanics/broadphase/force operations and host
feature/continuous-geometry certification. Visualization archives are saved
results, not physical restart checkpoints.

## Development entry points

Read the owning module's README and tests before editing. The
[architecture guide's validation section](docs/ARCHITECTURE.md#validation-and-build-entry-points)
distinguishes lightweight host checks from full vehicle acceptance. Existing
build trees pin different TL worktrees: follow the current workspace plan rather
than assuming `Total-Lagrangian-FEA/` is the qualified source.

The previous root README is preserved verbatim as
[early qualification history](docs/history/README_EARLY_QUALIFICATION.md).
Its commands, small-fixture limits, and status statements are historical; its
relative links retain their original root-README location and should be resolved
from the application root. They are not the current launch instructions.
