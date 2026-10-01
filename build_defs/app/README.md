# Native application libraries

This port declares the selected production dependency closure of the qualified
vehicle run, source preparation and preview-control libraries. It contains 95
native `cc_library` targets and 473 unique C++ translation units, arranged in
46 build packages matching their inherited source ownership. No CMake action,
test executable or native-reference Fortran library implements these targets.

The public bootstrap aliases are:

- `//build_defs/app:native_vehicle_run`
- `//build_defs/app:native_v6_sources`
- `//build_defs/app:native_preview_controls`
- `//build_defs/app:artifact_io`

`production_targets.json` records each original CMake target, owning source-file
hash, ordered translation-unit list, dependencies, retained numerical options
and corresponding native Bazel target. `check_port.py` checks these declarations
against the imported CMake files; it does not run CMake or generate a build at
execution time. Edit the small owning BUILD file when a library changes and
review the corresponding manifest change. The scanner deliberately supports
only the literal selected library subset, not arbitrary CMake evaluation.

Headers are declared from the source includes and transitive library coverage.
Inherited CMake sometimes supplied a broad include path without linking the
library that owns an included TL header. `header_deps` makes those dependencies
explicit per target; the manifest records the owning target and header evidence
separately from the original CMake link list. This avoids concealing missing
dependencies behind a global header set or absolute include directory.
Temporary rooted-include adapters preserve inherited `case/...`, `output/...`
and `collision/...` paths. An include adapter carries no implementation or hidden
header set: the actual libraries remain explicit dependencies. Source bytes are
unchanged. This is a packaging step, not completion of the proposed independent
FEA/MBD APIs.

Run the read-only checks from the Robodyna root:

```sh
python3 -B -m build_defs.app.check_port --workspace .
python3 -B -m unittest discover -s build_defs/app/tests
```

The current static checks pass; native graph analysis, compilation and a short
physical comparison are separate required gates. A source-list match alone is
not evidence that all inherited header/link dependencies are correctly declared.

## Explicit dependency providers

TL production dependencies use the native `@legacy_fea` targets. C++ CUDA users
consume `@rules_cuda//cuda:runtime`. Chrono uses
`//build_defs/chrono:native_core_fea`, whose native aggregate remains a transition
before independent mechanics domains. Eigen uses the root-pinned module.

Artifact hashing uses `@openssl//:crypto`, an explicitly declared local SDK.
`ROBODYNA_OPENSSL_ROOT` selects its absolute root; the initial Linux default is
`/usr`. The repository rule declares generic and architecture-specific headers,
the actual `libcrypto` shared library and a generated `sdk.json` recording paths
and header version. No OpenSSL binary is committed, downloaded or installed.
This matches the previous system-SDK dependency and is intentionally not claimed
as a hermetic pinned-source package. Qualify
`//build_defs/sdk:openssl_sdk_test` for header/runtime version and SHA-256 behavior
before using a new SDK in the production artifact writer.
