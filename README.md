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

The qualified native V6 Yaris run completed **100 ms with wall and self-contact**:
666,667 accepted intervals at 150 ns, 301 recorded states and two reviewed
60.2-second videos. Native TYPE25 friction/history and source-declared shell
removal are active in this profile. The [delivery checkpoint](docs/YARIS_DELIVERY_PLAN.md)
records source pins, preserved evidence, claim boundaries and remaining work.

The local consolidation combines the qualified simulation, rendering and
postprocessing histories. It does not qualify a new binary by itself. Physical
restart and complete balance ledgers remain future work; visualization archives
are saved results. A valid long-impact OpenRadioss CPU speed comparison remains
to be established.

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
