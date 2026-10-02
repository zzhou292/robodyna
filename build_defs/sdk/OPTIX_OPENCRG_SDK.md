# Additional SDK sources and owners

The OptiX source is NVIDIA `optix-dev` 9.1.0, revision
`f1f6dd803f3159992d248178f6e09421c6eb8b6d`, with observed immutable-archive SHA-256
`c36d6e52289138685dcce77befb8f966d2761c6840556f6a04945b01700c8ebc`.
`optix_pins.json` also records the NPP and cuRAND archives from the already
authenticated NVIDIA CUDA 13.2.2 redistribution manifest. All three downloaded
archives total 312,165,846 bytes. The guarded extraction receipt is
`crash-work/reports/robodyna-optix-sdk-extract-1.json`; the workspace prefix is
`crash-work/install/optix-r0`. Extraction is not native or GPU qualification.

`local_optix_sdk(name = "optix_sdk")` from `optix.bzl` reads
`ROBODYNA_OPTIX_ROOT`. The provider verifies its exact receipt, source/header
identities and observed NPP SONAME/dependency closure. It reuses the existing
CUDA runtime and CUDA math providers. OptiX and cuRAND headers retain their own
notices; they are not relabeled as first-party BSD sources. The installed driver
must supply the actual OptiX implementation at runtime.

OpenCRG follows the repository named by the retained Chrono
`contrib/build-scripts/linux/buildOpenCRG.sh`: `hlrs-vis/opencrg`, tag 1.1.2,
revision `31a67e1b92ad990ee6c95c5b2bba7783c9e28819`. Its 60,939-byte source archive
has SHA-256 `0c39a81256ccdbd0d6a8290d0394a4e3845c723b1d21e54800f78644e5342681`.
This is the historical source mirror used by that recipe, not a claim to be a
new ASAM release. The archive is cached at
`crash-work/dependencies/opencrg-r0/opencrg-31a67e1b92ad990ee6c95c5b2bba7783c9e28819.tar.gz`.

`local_opencrg_sdk(name = "opencrg_sdk")` from `opencrg.bzl` reads
`ROBODYNA_OPENCRG_ARCHIVE`, verifies archive and per-file hashes, and declares a
native Bazel C library from the exact 11 original implementation units and two
headers. It preserves the Apache 2.0 license and original per-file notices.
No prebuilt OpenCRG, Chrono, or mechanics binary is imported by that provider.
