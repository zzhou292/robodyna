# Retained demo and unit-test inventory

**All pinned Chrono demo and unit-test source files are present. Most are not yet
directly runnable through the root Bazel build.** Source retention, build exposure
and successful execution are separate results.

The [machine-readable inventory](CHRONO_DEMOS_TESTS_INVENTORY.json) compares
current files with the original imported Git tree
`bec2d91ed3ed83cfa194256fe13d1a68d26d3be2`, from source commit
`a5ec9bf5463d7c5ef98817a23051fa07fc0a8a38`. Original blob identities remain pinned;
maintained capture/branding changes have separate current hashes. Refresh those
hashes after source edits using the command below. The inventory does not alter
sources, enable modules, configure CMake, or run the listed simulations/tests.

## What is present

| Pinned scope | All files | Named example/test source files | Breakdown |
| --- | ---: | ---: | --- |
| `src/compatibility/chrono/src/demos/` | 564 | 425 | 311 C++ `demo_*`, 97 Python `demo_*`, 17 C# `demo_*` |
| `src/compatibility/chrono/src/tests/unit_tests/` | 175 | 143 | 133 C++ `utest_*`, 10 Python `pyutest_*` |
| Additional named example outside those trees | 1 | 1 | `template_project_fmi2/demo_FmuComponentChrono.cpp` |

The 739 files in the two standard trees include 98 `CMakeLists.txt` files,
headers, helper code and documentation. The full demo tree contains 331 C++ and
115 Python files, including non-`demo_*` helpers. These are **source-file counts**,
not counts of GoogleTest cases, CTest instances, independent binaries or passing
simulations. Parameterized tests can produce many runtime cases from one file.

No expected file in these scopes is missing. Optional dependency repositories
are recorded separately; their own demos/tests are not counted as first-party
Chrono cases. Benchmarks, templates without `demo_*` names and third-party test
suites are outside this inventory's stated demo/unit-test scope.

At this snapshot, 396 demo-tree files and seven unit-test-tree files retain edits
relative to the import; 336 files in the two trees remain byte-identical. The
additional FMI template example is also unchanged. The inventory records those
differences explicitly rather than confusing presentation/capture maintenance
with missing sources or claiming every edited case was rerun.

## What the root build actually exposes

| Original retained source | Real root target | Recorded execution scope |
| --- | --- | --- |
| `demo_MBS_spring.cpp` | `//examples/mbd:spring` | Six-second CPU dynamics and Vulkan capture/video |
| `demo_MBS_collisionNSC.cpp` | `//examples/mbd:collision_nsc` | Six-second CPU dynamics and Vulkan capture/video |
| `demo_VEH_SCMTerrain_RigidTire.cpp` | `//examples/scm:rigid_tire` | Six-second CPU SCM dynamics and Vulkan capture/video |
| `utest_CH_composite_inertia.cpp` | `//src/mechanics/inertia:inertia_test` | Original source included in a passing native unit-test target |

Other original cases are retained as source, including through the compatibility
source filegroup, but have no direct root `cc_binary`/`cc_test` exposure found by
this audit. A filegroup is not a runnable test. The many additional Robodyna
regression tests are valuable and separately qualified; they do not establish
that all inherited Chrono unit tests have been built or run.

Named demo evidence is in [CHRONO_DEMOS.md](CHRONO_DEMOS.md) and
[examples/QUALIFICATION.json](../../examples/QUALIFICATION.json). The JSON
inventory records whether each historical demo source hash matches the current
file. Source-hash equality alone does not qualify every dependency change.
Fresh branding/media qualification can coexist with preserved historical
receipts. No new simulation or video was produced by this audit.
The separate [transparent-branding checkpoint](TRANSPARENT_BRANDING.json) records
the fresh captures of the three selected demos with current caption/source hashes
and exact physics comparisons; the inventory deliberately retains the original
qualification record and labels its source mismatch as historical.

## Inherited CMake discovery has real gaps and conditions

The retained project still traverses its demo tree with `BUILD_DEMOS` and its
unit-test tree with `BUILD_TESTING`; module and per-family options add further
gates. The inventory records uncommented source declarations, lexical conditions
and ancestry. This is static evidence, not a successful configure or SDK check.
The current root CMake compatibility profile explicitly disables inherited demos
and tests while building its library; its three wrapper tests are separate.

**The inherited GoogleTest gitlink is empty.**
`src/compatibility/chrono/src/CMakeLists.txt` checks for
`chrono_thirdparty/googletest/CMakeLists.txt` and forcibly turns `BUILD_TESTING`
off if it is absent. Consequently, simply requesting `-DBUILD_TESTING=ON` in
that retained project does not currently expose the full suite. Root Bazel's
separately pinned `@googletest` works for the targets already declared, but does
not populate the inherited gitlink.

Three retained C++ unit-test files are absent from active local source lists:

- `dem/utest_DEM_mini.cpp` — commented out in its CMake list.
- `multicore/utest_MCORE_mpm.cpp`.
- `multicore/utest_MCORE_svd.cpp`.

Nine listed unit-test programs explicitly use `build_utests(NO ...)`, which builds
executables when their configuration is enabled but does **not** register CTest
runs:

- Seven sequential SMC programs: `cohesion`, `cor_normal`, `rolling_gravity`,
  `sliding_gravity`, `spinning_gravity`, `stacking`, and `sphere_sphere`.
- `utest_SYN_MPI` and `utest_SYN_agent_initialization`.

Three named demo sources have no active literal declaration in their local
CMake file: `demo_PERI_benchmark.cpp`, `demo_PERI_fluid.cpp`, and
`demo_CS_VEH_CRGTerrain_IRR.cs`. The C# ROS sensor demo has a local declaration,
but the retained C# root does not traverse its `ros` directory. These are inherited
exposure limitations, not files lost during import; this audit does not silently
enable them.

Python demos are installed as a directory by the Python module's CMake script.
They are scripts requiring their matching binding/modules and dependencies, not
individual native CMake executable targets. C# demos use a separate managed
subbuild with a compiler/generator check. Compiling the native C# wrapper does
not demonstrate managed C# example execution.

## Optional dependencies and qualification boundary

All five recorded Chrono gitlink directories are currently empty: `googletest`,
`googlebenchmark`, `flatbuffers`, `fmu-forge`, and `HydroChrono`. Their original
commits are preserved in [DEPENDENCIES.json](../migration/DEPENDENCIES.json).
GoogleTest blocks the inherited CMake unit-test tree; the others affect benchmark,
SynChrono, FMI and TDPF profiles respectively. Existing root SDK/dependency targets
must not be confused with those uninitialized source directories.

Other optional profiles retain their source-side checks for packages such as
OpenCASCADE, MUMPS, MKL, OptiX/CUDA, ROS 2, preCICE and communication libraries.
Their availability was not guessed or tested through configure in this audit.
The inventory's `cmake_files` records the owning `find_package` calls and module
conditions; [CAPABILITIES.md](../migration/CAPABILITIES.md) distinguishes retained
modules from qualified native backends. Do not label the entire FSI, vehicle,
GPU or managed-binding demo collection as qualified because its source exists.

## Discover and refresh

From the root workspace, list only real executable/test exposure:

```sh
bazel run //tools/verification:chrono_catalog -- --status runnable
bazel run //tools/verification:chrono_catalog -- --kind unit_test --module fea --status pending
bazel run //tools/verification:chrono_catalog -- --kind demo --language python --format json
```

The command prints exact source paths, CMake evidence/gates, an actual native
target or `pending`, and recorded execution evidence. It never launches a case.
The `qualified` filter selects named historical evidence and explicitly labels
source-hash mismatches; it does not qualify the current dependency graph.
For a checkout-only invocation without building the discovery tool:

```sh
python3 -B -m tools.verification.catalog_cli --kind demo --module core
```

Refresh the source snapshot after coordinated source edits:

```sh
python3 -B -m tools.verification.chrono_inventory --repo . \
  --output docs/verification/CHRONO_DEMOS_TESTS_INVENTORY.json
```

The checker preserves the original Git pins and reports modified or missing files;
it does not repin originals or fetch dependencies. Its actual behavior is tested
by `//tools/verification:catalog_tests`, including a temporary Git repository with
unchanged, modified and missing sources. Discovery checks are not numerical tests.

## Practical exposure sequence

1. Keep this source inventory and historical evidence current. Resolve the
   GoogleTest dependency deliberately; do not silently adopt the different old
   checkout revision recorded in the import notes.
2. Add real root targets in small module groups, starting with dependency-light
   core demos and core/geometry/physics tests. Reuse existing native library and
   GoogleTest owners, preserving each case's source and data inputs.
3. Expand mechanical FE and MBD cases with explicit renderer/solver requirements.
   Keep interactive demos opt-in and respect per-case resource/time requirements;
   do not launch hundreds of unbounded examples together.
4. Add optional SDK profiles and investigate the unlisted/CTest-disabled cases
   individually. Record why each is enabled or remains pending; do not equate
   source listing with a working implementation.
5. Publish runtime qualification only after each real target builds and passes
   its appropriate checks. Keep the existing vehicle and coupled mechanics gates
   intact while increasing inherited-case coverage.
