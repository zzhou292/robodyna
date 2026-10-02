# Unified Robodyna demo build

Worktree: `robodyna`, branch `work/robodyna-demo-builds`, based on the published
branding checkpoint `e16909532138ea0cc2a57225a504cff05d0c5bbe`.
This milestone is complete and retained on the local branch. No remote push was made.

## Deliverable and counting rules

Compile every retained demo through the root Bazel workspace with descriptive
Robodyna targets. The scope is **452 original programs**:

- 338 native programs: 311 inherited C++ mains, one FMI template, and 26 CUDA FEA programs.
- 97 Python demos, with their real native extensions and Python dependencies.
- 17 C# demos, with compiled managed assemblies and their native wrappers.

The retained preCICE YAML application is an additional application, tracked
separately. Multiple targets for one original main do not increase the source
count. Third-party DEM-Engine examples and test drivers are separate from the
26 first-party CUDA FEA programs.

A source file, target declaration, successful compilation, and completed
simulation are different evidence. The historical source inventory remains
unchanged. The new operator reads Bazel's expanded target graph and records
explicit target completion; it must reject missing or skipped programs.

## Current progress

| Family | Completed evidence | Separate follow-up scope |
| --- | --- | --- |
| Native | All 338 programs compiled, including all 26 CUDA FEA programs | Individual GUI/GPU trajectories and asset admission |
| C# | All 17 assemblies compile; baseline, OpenCRG, Sensor and ROS Sensor runtime/ownership checks pass | Portable launchers for the 16 assemblies beyond the existing core preset |
| Python | All 97 original programs have package-build evidence; optional SPH, YAML, Sensor and ROS runtime checks pass | Full interactive/GPU trajectories |

The final complete controller, `robodyna-demo-matrix-full-4`, passed all 23 batches
on the frozen final source snapshot. Its 457 explicit labels map to 452 distinct
original demo programs plus the separately tracked preCICE application; aliases
do not increase the source count. All 35 build phases passed their guards with
complete cleanup. All 15 required runtime groups and nine additional
operator/source/MPI regression targets passed. Earlier runtime evidence records
its explicit source-delta boundaries rather than claiming identical snapshots.
See [the qualification record](../verification/DEMO_BUILD_QUALIFICATION.json).

Managed runtime qualification now passes the reviewed helper/type-collision repairs.
Eight generated helper pairs have exact hash approvals after comparison of their
C# proxies and C++ bodies, preserving the original first-owner ordering. Three
RoboSimian shape classes are different native types from similarly named Core
shapes; the C#-specific interface now gives them explicit RoboSimian-prefixed
names. Core, Python and C++ names remain unchanged. New runtime checks cover
borrowed pointer disposal, vector copies/exceptions and construction, fields and
disposal for both shape families. No general duplicate-class exception was added.
The separate managed test-marker quoting fix passed all three actual marker
checks. The camera demo's two call sites now explicitly supply the current
native defaults before gamma, with original scene parameters preserved.

Detailed recipes, logs, guard receipts and the current machine-readable status
are in `crash-work/investigations/robodyna-demo-builds-1` and
`crash-work/reports` in the enclosing workspace.

Recent completed checks include:

- Four optional field-FEA demos and five material, geometry and ownership cases.
- Six ordinary parser demos, real URDF parsing and Python embedding.
- Six Vehicle FMUs, five original drivers, and their six runtime/source/ELF checks.
- Six baseline Vehicle co-simulation demos and a real four-process MPI exchange.
- Nine distributed demos, actual CDR/FlatBuffers serialization and a two-process MPI exchange.
- Three OpenCRG vehicle demos, road-height checks and source ownership.
- Eight CRM vehicle/robot demos with consistent SPH and VSG settings.
- Real standard and custom ROS message transport. The custom gate covers all 16
  retained schemas and a typed Body exchange in both directions.
- Baseline Python imports, one shared core/image owner, and native object exchange.
- NumPy core conversions, input/output copy ownership, and exact declared runtime origin.
- Real plotting output and PythonOCC/native shape exchange, mass, inertia and
  lifetime checks through the existing OCCT implementation owner.
- The original native `core/build_system` mechanism through the new launcher:
  50 reported frames and 250 solver steps. Its last requested frame time was
  2.5 seconds; that is not an independently sampled System clock.
- Driver/guard regression and the source/branding verification tests.

## Architecture and source boundaries

Use domain paths such as `examples/mbd`, `examples/fea`, `examples/fea/cuda`,
`examples/vehicle`, and `examples/sph`. Reuse the original executable and module
owners. An alias must not introduce a second solver or a replacement main.

Optional configurations must agree across implementation, headers and language
wrappers. In particular, field-FEA, Multicore, SPH, VSG, Sensor, YAML and ROS
capabilities can affect declarations or layouts. Do not enable those interfaces
only in a demo translation unit. Demo-only renderer availability remains limited
to cases where it does not change shared layouts.

Each Python or C# package uses one configured native core and shared module
owners. Baseline wrappers remain separate from unsupported optional layouts.
The ROS middleware node stays in its own process, with the original protocol.

The existing Yaris physical owner, numerical settings, source notices and
accepted archives remain preserved. This task does not claim that every
inherited API name has already migrated or that FEA and MBD implementation
linkage is fully separated.

## Reviewed fixes discovered during qualification

Narrow repairs have explicit provenance and focused checks:

- FMU C++ exports are isolated to prevent interposed static-state cleanup.
  Temporary archive metadata is stored safely; frozen JSON, XML and binary
  archive tests still pass.
- Peridynamics needed small visualization API/include repairs. Its original
  experimental fluid guard remains; compilation does not qualify fluid dynamics.
- `ChDrawer.h` now includes the three visualization types it directly uses.
- ROS/Sensor SWIG declarations receive the actual shared configuration and valid
  concrete template registrations, preserving original native implementations.
- Python packages load native libraries through their original runfile paths.
  The Pardiso shared owner retains the declared vendor runtime flags.
- The inherited tensor method read column 3 from a three-column matrix. The
  correction to column 2 passes six float/double stress, strain and engineering
  strain cases checking residuals, orthogonality and reconstruction. The original
  bounds failure and exact inverse are preserved.
- The shared core owns the complete existing YAML library, including parser
  entry points that ordinary transitive static linking omitted. Native/Python
  parser checks and the single-owner symbol check pass.
- preCICE's real configuration validator initializes MPI. Its unchanged test
  backend now uses the existing isolated single-rank launcher with declared,
  hashed fixture inputs. The existing two-rank no-input behavior also passes.

All 558 current inverse records restore the immutable original source hashes.
The historical branding manifest is unchanged; a separate follow-up registry
records four later edits to branded files, including the C# camera's explicit
native constructor defaults.

## Completed gates and next work

The focused profile builds, complete matrix and reviewed 15-group runtime roster
passed. The roster contains 23 test-target occurrences across configured profiles;
nine additional operator/source/MPI targets also passed. The four controller
tests cover 28 behavioral cases. A real selected-batch build and resume passed,
and read-only status now reports validated batches during execution. Failed
attempts remain preserved alongside the successful replacements.

The next work is separate from this completed build milestone:

1. Expose and qualify the broader inherited unit-test suite by module.
2. Add audited asset/output/resource presets for more native, CUDA and managed
   demos, then run selected GUI/GPU trajectories explicitly.
3. Continue the independent FEA/MBD implementation boundaries and remaining API
   migration in `NEXT_SEAMS.md`; retain the qualified coupled physical owner.

CPU-only Sensor checks use GPS. OptiX camera construction creates a CUDA stream,
so compilation and host binding checks do not qualify camera execution.

The matrix is `examples/BuildMatrix.json`; operator instructions are in
`examples/README.md`. The native launcher uses an explicit audited preset,
create-only working directory and a sibling data link. Building programs never
starts their simulations automatically.

## SDK and runtime limits

SDKs are explicitly admitted workspace-local dependencies. Their source/package
hashes, ABI and real link/runtime checks are recorded. No installed Chrono
implementation supplies a hidden second physics owner.

The three Python CAD examples use the complete 57-module import closure built
from the real PythonOCC targets, with the existing OCCT and NumPy owners. Other
PythonOCC wrapper families are outside this SDK profile. The required OCCT
libraries are loaded from the declared dependency graph before importing the
package; a raw standalone OCC import before that bootstrap is not advertised.

Known inherited field-contact limitations remain documented in
`src/fea/multiphysics/PROFILE.md`. The CPU field checks do not qualify those
contact paths. GUI, GPU, coupled OpenFOAM and long-duration simulation claims
require their own explicit runs.

## Resources and disk audit

Keep the shared workstation lock. Ordinary compilation uses eight affinity CPUs,
four compiler workers, a 16 GiB sampled RSS limit and at least 32 GiB available
RAM. A measured two-wrapper SPH phase peaked at 8.286 GiB. Generated wrappers use two
compiler workers within the unchanged guard; a batch can return to one if needed.
Heavy jobs remain serialized, and other GPU work stays running.

The disk audit is complete and no deletion was performed. The first candidates
are 24.04 GiB of copied CMake data and 8.32 GiB of staged benchmark inputs, subject
to final identity/use checks before removal. Old worktrees and build directories
need a separate dependency review. Preserve accepted crash archives, videos,
frozen binaries, source checkpoints and failure receipts.

Exact cleanup paths and retention notes are in
`crash-work/investigations/disk-cleanup-audit-20261001-1`.
