# Native Vehicle and model ownership

This batch declares the retained mechanical Vehicle profile with FEA, VSG and
Irrlicht support. CPU SCM remains the existing qualified implementation. CRM/SPH,
OpenCRG, SCM GPU, FMI and MPI co-simulation are separate profiles, not silently
enabled by the presence of a header or source directory.

| Public target | Compiling ownership |
| --- | --- |
| `//src/vehicle:vehicle` | 229 new host sources in 28 nonempty CMake subsystem groups, plus the four existing `native_scm` sources |
| `//src/vehicle/models:models` | 389 sources in 19 final vehicle-family groups; the twentieth CMake group contains common headers |
| `//src/vehicle/visualization:vsg` | Five new sources plus the existing `native_scm_vsg` source |
| `//src/vehicle/visualization:irrlicht` | Five retained sources, depending on the existing native Irrlicht owner |

All targets use the same existing native core. The two STB implementations are
reused from `native_stb`. The generated `ChConfigVehicle.h` is an alias to the
existing SCM configuration owner: no second header with competing feature values
is generated. Existing physical sources, model constants and header definitions
remain unchanged in `src/compatibility/chrono`.

`sources.bzl`, `models/sources.bzl` and `visualization/sources.bzl` are explicit,
human-editable CMake-derived inventories. There is no source glob that silently
adds an optional algorithm. `defs.bzl` declares compilation owners from those
groups; the public aggregates retain the original complete module symbol closure.
The groups are build organization, not independently qualified mechanical domains.

The original Vehicle CMake list repeats `CV_WV_BRAKE_FILES`; CMake still compiles
each file once. This list is deduplicated in the native ownership map. Model lists
also contain nested helper groups; only the final `ChronoModels_vehicle` groups
become owners, so those helper files are never compiled a second time.

The source inventory tests compare the admitted implementation against the
retained CMake groups, verify all 64 demo dispositions, and authenticate 1,378
preintegration source/header/helper inputs. `SourceBaseline.json` binds the current
materialized repository snapshot without replacing original import/rename history.
The Bazel analysis gate separately inspects actual transitive compile owners,
including the reused SCM, SCM-VSG and STB sources. It catches an extra, omitted or
second compilation even if the intended source manifest looks correct.

The host test exercises a terrain through the shared world-frame owner, attaches
a rigid terrain patch to the canonical System and loads the retained HMMWV tire
class/constants. It constructs no renderer, starts no GPU job and takes no
physical step. These small tests do not qualify every driving scenario or new
FSI/co-simulation profile.

Build and runtime receipts remain explicit follow-up. The first admission contains
35 actual demo programs; the other 29 remain in the complete roster with concrete
requirements. See [the demo catalog](../../examples/vehicle/DemoCatalog.json) and
[runtime asset plan](../../examples/vehicle/RUNTIME_ASSETS.md). No heavy build or
simulation was performed by the agent authoring this source partition.
