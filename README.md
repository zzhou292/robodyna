# Robodyna

Robodyna is being consolidated into one source-owned CUDA multiphysics repository
with Bazel as the product build. It absorbs the qualified TL structural solver,
application and all inherited Chrono capabilities. FEA and multibody dynamics are being separated into sibling modules with shared
mechanics contracts and explicit coupling. The current production backend still
uses the qualified combined owner while those boundaries are extracted.

## Current migration status

This branch is in active restructuring. Source retention, build qualification,
functional qualification and CUDA coverage are tracked separately. Historical
names and aggregate backends are temporary compatibility boundaries. The current
100 ms Yaris result remains preserved outside this repository.

Start with [the execution plan](docs/migration/EXECUTION.md),
[the module architecture](docs/architecture/MODULES.md), and
[the source manifest](docs/migration/SOURCES.json), and
[the capability coverage](docs/migration/CAPABILITIES.md).

The native solver, application and VSG viewer now build through the root Bazel
graph. The normal CLI passed the short vehicle regression with byte-identical
recorded output. See [the evidence](docs/migration/QUALIFICATION.json) for the
exact scope; full domain independence and optional-module coverage remain work
in progress. [Operating the product](docs/migration/OPERATING.md) gives the build,
run, inspect and render commands; [build notes](build_defs/README.md) describe SDKs.

## Source and data policy

Preserve original attribution and license files. Imported source histories remain
reachable. Large result archives, binaries, build caches and videos stay outside
source control. Required test fixtures and tracked model assets retain their
original identity; Git LFS objects are accounted separately from Git blobs.
