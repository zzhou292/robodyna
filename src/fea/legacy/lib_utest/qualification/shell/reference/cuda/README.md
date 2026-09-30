# Immutable selected CUDA donor source

The eight files listed by `source-manifest.json` are exact bytes from OpenRadioss
revision `a62b27e6baa555d222a580d6218867d0be4d70b5`, relocated from the frozen
workspace checkpoint. Original paths, git blob IDs, SHA256, retrieval provenance,
notices and the complete AGPL-3.0-or-later license are preserved.

The focused build compiles selected kernel templates through private fixture
translation units. It does not build the donor driver or its process-exiting
host launchers as the fixture API. `../../operators/generate_gauss3.py --verify`
checks this exact inventory plus the separately generated elastic Gauss adapter.
No original file is rewritten and no missing dependency is fetched by the build.
