# Robodyna

<p align="center"><img src="assets/brand/robodyna-logo-primary.png" alt="Robodyna" width="680"></p>

Robodyna is being developed as one source-owned CUDA multiphysics repository
with Bazel as the product build. It absorbs the qualified TL structural solver,
application and all inherited Chrono capabilities. FEA and multibody dynamics are being separated into sibling modules with shared
mechanics contracts and explicit coupling. The current production backend still
uses the qualified combined owner while those boundaries are extracted.

## Simulation videos

Robodyna-branded videos from recorded simulations. **Yaris uses CUDA FEA**;
the retained **spring, rigid-body contact and SCM examples use CPU physics**.
All five views use Vulkan GPU rendering. See the linked qualification records
for the tested scope and source provenance. The [media record](docs/verification/README_MEDIA.json)
records native captures with the transparent Robodyna logo, playback validation
and the preserved numerical evidence.

### Yaris wall crash — front view

100 ms of simulated impact with wall and self-contact, shown in 60.2 seconds of
slow-motion playback. The selected vehicle assembly includes elastic/plastic
deformation; inflated tires remain outside this profile.

https://github.com/user-attachments/assets/71270cf4-bf3e-41bf-9e55-e940a0b763a2

<details>
<summary>Robodyna poster</summary>

![Robodyna Yaris crash — front view](docs/verification/images/yaris-front.png)

</details>

### Yaris wall crash — overview

The same 100 ms trajectory, with the complete selected assembly and mesh wall
visible. Parts keep their own colors and deformation is shown at physical scale.

https://github.com/user-attachments/assets/16654165-39e5-437b-879e-b314db5bfd45

<details>
<summary>Robodyna poster</summary>

![Robodyna Yaris crash — overview](docs/verification/images/yaris-overview.png)

</details>

### Spring dynamics

Six seconds of the retained spring–mass example. Its native and callback force
models produce matching motion. [Evidence](docs/verification/CHRONO_DEMOS.md).

https://github.com/user-attachments/assets/46613fa4-a2e5-4cc0-8014-0b2ec49d1b1c

<details>
<summary>Robodyna poster</summary>

![Robodyna spring dynamics](docs/verification/images/spring.png)

</details>

### Rigid-body collisions

Six seconds with all 87 falling objects and the rotating mixer, using the retained
Bullet/NSC contact model. [Evidence](docs/verification/CHRONO_DEMOS.md).

https://github.com/user-attachments/assets/a906314f-778f-4fb5-9fcb-57b1420c00fe

<details>
<summary>Robodyna poster</summary>

![Robodyna rigid-body collisions](docs/verification/images/collision-nsc.png)

</details>

### SCM wheel and deformable soil

Six seconds of the retained lugged-wheel example, with soil deformation and a
visible rut. [Evidence](docs/verification/CHRONO_DEMOS.md).

https://github.com/user-attachments/assets/c779247e-bded-46ad-a7aa-95077b73918c

<details>
<summary>Robodyna poster</summary>

![Robodyna SCM wheel and soil](docs/verification/images/scm.png)

</details>

## Current migration status

This branch is in active restructuring. Source retention, build qualification,
functional qualification and CUDA coverage are tracked separately. Historical
names and aggregate backends are temporary compatibility boundaries. The current
100 ms Yaris result remains preserved outside this repository.

Start with [the execution plan](docs/migration/EXECUTION.md),
[the module architecture](docs/architecture/MODULES.md),
[the source manifest](docs/migration/SOURCES.json), and
[the capability coverage](docs/migration/CAPABILITIES.md).

The native solver, application and VSG viewer now build through the root Bazel
graph. The normal CLI passed the short vehicle regression with byte-identical
recorded output. See [the evidence](docs/migration/QUALIFICATION.json) for the
exact scope; full domain independence and optional-module coverage remain work
in progress. [Operating the product](docs/migration/OPERATING.md) gives the build,
run, inspect and render commands; [build notes](build_defs/README.md) describe SDKs.

The original MBD/SCM setups run through this repository's Bazel build. Their
complete telemetry matched between headless and captured runs, and their videos
passed full decoding. [Qualification and commands](docs/verification/CHRONO_DEMOS.md)
record timesteps, measured behavior and reproducible entry points.

All imported Chrono demo and unit-test source files are retained. The
[complete catalog](docs/verification/CHRONO_DEMOS_TESTS.md) separates CMake
declarations, Python scripts, direct Bazel targets and runtime qualification.
All 452 original demo programs now have public root-Bazel routes and pass the
complete compile matrix: 338 native programs (including 26 CUDA FEA programs),
97 Python and 17 C# examples. The separately tracked preCICE application also
builds. Focused optional binding runtime and ownership checks pass.
Use [the build matrix and operator](examples/README.md) and
[current plan](docs/migration/DEMO_BUILD_PLAN.md) for live scope and evidence.
The inherited unit-test suite and full interactive/GPU trajectories have separate
remaining qualification work. List the retained historical catalog
with `bazel run --config=host //tools/verification:chrono_catalog -- --help`.

## Robodyna C++ API migration

The first public headers expose `robodyna::mbd::RbBody`,
`robodyna::fea::RbMesh`, and `robodyna::simulation::RbSystemNSC`, with supporting
math, contact and solver types. `RbBody`, `RbMesh`, the System/NSC/SMC/Assembly family, and
the inertia utilities now have actual canonical definitions under their modules; their legacy
names alias those same types. The remaining initial names are aliases while their
implementation families migrate. No wrapper objects or alternate physics are added.

Build the small headless examples with
`bazel build --config=host //examples/api:rigid_spring //examples/api:fea_spring`.
Their [source and usage](examples/api/README.md) use Robodyna public includes.
The [rename qualification](docs/migration/RENAME_QUALIFICATION.json) records the
passed API, archive, mechanics and build checks. Full independent FEA/MBD ownership
and the remaining module/binding migrations are still in progress.

The body checkpoint preserves frozen object archives, the complete recorded
results of the six-second spring/NSC/SCM examples, and every saved file from the
101-step GPU Yaris regression. Core Python runtime and both native binding wrappers
are qualified against one shared backend. All 17 C# assemblies compile, and the
baseline and optional profiles passed their managed/native checks. Named profiles
have separate import, object-lifetime and shared-owner evidence; these checks
do not claim that every original demo trajectory has run.

## Chrono acknowledgement and source cutoff

We thank the [Project Chrono Development Team and contributors](https://projectchrono.org/)
for the simulation infrastructure and examples on which this work builds.
Project Chrono's [retained BSD license](src/compatibility/chrono/LICENSE) and
copyright notices remain in place.

The imported local-fork snapshot is
`a5ec9bf5463d7c5ef98817a23051fa07fc0a8a38`
([source commit](https://github.com/zzhou292/robodyna/commit/a5ec9bf5463d7c5ef98817a23051fa07fc0a8a38)),
dated **2026-09-11**. Its baseline import into Robodyna occurred on **2026-09-30**
([import commit](https://github.com/zzhou292/robodyna/commit/557e501b926526ca1ca918d670a2abd80b19cfdb)).
These dates identify our local source snapshot and import, not an official upstream
release or synchronization date. Robodyna develops independently from this pinned
snapshot and does not automatically synchronize with Chrono upstream.
The [source manifest](docs/migration/SOURCES.json) records the exact revisions.

Licenses differ by component. OpenRadioss-derived ports retain their
[AGPL-3.0 notices](src/fea/legacy/lib_src/collision/radioss_type25/LICENSE.md);
the Chrono BSD license is not a blanket license for the whole repository.

## Source and data policy

Component licenses and original attribution notices are retained. Imported source histories remain
reachable. Large result archives, binaries, build caches and videos stay outside
source control; the videos above are linked public media. Required test fixtures and tracked model assets retain their
original identity; Git LFS objects are accounted separately from Git blobs.
