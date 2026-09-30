# Native host mechanics build

`//build_defs/chrono:native_core_fea` compiles the absorbed implementation using
native Bazel C++ actions. There is no CMake action in its dependency graph. It
remains a combined inherited core/FEA aggregate; this is not the completed FEA/MBD
architectural separation.

The initial profile matches the temporary bridge: Linux, Release-style `-O3`,
static library registrations, mechanical FEA enabled, thermal FE and optional
modules disabled, no GPU, no Thrust/multicore collision, no OpenMP/SIMD/HDF5/YAML.
The original source of all excluded capabilities is retained. Native support for
each additional profile needs an explicit source/configuration/dependency gate.

`native_sources.bzl` records 482 translation units in the original CMake groups.
The SHA-256 identifies the source CMake list that was reviewed. Mechanical FEA,
its visual shape helper, the retained Reissner frame helper and Linux socket
sources are included. Optional collision-multicore, thermal, HDF5 and YAML sources
are excluded. Implementation files are not selected with a recursive glob.

`native_config.bzl` generates the existing `ChConfig.h` and `ChVersion.h` templates
as declared outputs. The source-path trim is zero because Bazel compilation paths
are execution-root-relative; no historical absolute path is embedded. This only
changes diagnostic filename presentation. Header feature metadata must be checked
against the bridge before numerical parity is claimed.

The core implementation preserves the inherited Eigen-plugin include order by
including `ChCorePCH.h`; this is an ordinary include, not a new PCH implementation.
Bundled collision/geometry dependency sources retain their separate no-PCH warning
policy. `CH_STATIC`, Eigen serial mode, Bullet fixed-point/thread-safe flags and
static factory registration are explicit. There is no fast-math option. Any
toolchain-level numerical flags still need to match during parity verification.

Qualification is pending. After the parent build/resource guard admits the job:

```sh
bazel test --config=host --jobs=4 --local_test_jobs=1 //tests/chrono:native_host_tests
```

Run the bridge suite separately under the same compiler/Eigen configuration. Do
not link both implementations into one executable. Compare the actual CMake
`compile_commands.json` translation-unit inventory with this manifest, generated
feature definitions, flags and test outcomes. These short tests do not establish
bitwise parity for all element/contact formulations or a speed advantage.

Next extraction remains the documented mechanics plan: low-level interfaces,
independent domain targets, explicit mixed attachments, then the existing strong
coupling regression. Avoid optimizing algorithms during build migration.
