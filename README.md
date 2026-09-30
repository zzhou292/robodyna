# Robodyna

Robodyna is being consolidated into one source-owned CUDA multiphysics repository
with Bazel as the product build. It absorbs the qualified TL structural solver,
application and all inherited Chrono capabilities. FEA and multibody dynamics are
separate target modules with shared mechanics contracts and explicit coupling.

## Current migration status

This branch is in active restructuring. Source retention, build qualification,
functional qualification and CUDA coverage are tracked separately. Historical
names and aggregate backends are temporary compatibility boundaries. The current
100 ms Yaris result remains preserved outside this repository.

Start with [the execution plan](docs/migration/EXECUTION.md),
[the module architecture](docs/architecture/MODULES.md), and
[the source manifest](docs/migration/SOURCES.json).

Supported build commands will be added as they are verified. Do not treat proposed
architecture command examples as an already-qualified build interface.

## Source and data policy

Preserve original attribution and license files. Imported source histories remain
reachable. Large result archives, binaries, build caches and videos stay outside
source control. Required test fixtures and tracked model assets retain their
original identity; Git LFS objects are accounted separately from Git blobs.
