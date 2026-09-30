# Immutable vehicle source plan

`VehicleSourcePlan` retains the complete canonical shell selection and shares
its immutable `CanonicalSource` backing. A compact authenticated sidecar adds
typed source declarations and explicit unresolved part obligations. No geometry
JSON copy, FE owner, native M/J, runtime family, force or clock is created.
`SupportedDeclaration` means only that the source material/section is within
the current declaration domain. Connections, native reference validity and
complete runtime admission remain separate. Full source attachment evidence is
retained in `canonical().data().scope_bytes`; no component release is inferred.

The sidecar's supported table uses the existing V3 value codec directly:
`ReadMaterialPolicy` and `ReadDeclarations`, with explicit vehicle table limits.
The component reader and V1/V2/V3 admission are unchanged. Every typed scalar is
also compared to the original raw fixed-width card and source block hash. Every
unresolved part retains original PART/SECTION/MATERIAL blocks and parser reasons.
Canonical parent order and ascending source NID index order are explicit; no
force/mass accumulation order or runtime-family index is inferred.

The owning dependency on `output/full_shell/static_bundle::CanonicalSource`
reuses the existing source decoder. It introduces no dependency on a solver or
accepted-replay state. Copy/move construction retains shared backing; assignment
is disabled. Failed `Read`/`ReadBytes` creates no published plan. Startup limits
cover input/DOM/typed values, decoded indexes and one canonical backing; the
reported conservative budget is not a native allocation forecast or RSS limit.

Frozen actual input: scope10 SHA256
`fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0`.
`crash-work/reports/yaris-vehicle-declarations-1.json` is 3,648,589 bytes, SHA256
`a96bc12b9c8467253da0898565c7875ad80f58f963b45d1dc405f5dddab76b1d`.
It contains 867 selected parts/349,645 shells: 823 parts/278,301 shells have typed
declarations; 44 parts/71,344 shells remain unresolved. Both populations remain
in the 359,785-node canonical selection. Generation uses existing metadata/card
parsers and reads no geometry; arrays are authenticated by `CanonicalSource`.

Owning target: `robo_dyna_vehicle_source_check`; CTest: `vehicle_source_plan`.
Configure `-S modelio/vehicle_source` with `Chrono_DIR`, `ROBO_DYNA_TL_ROOT`
(neutral vector header only), and these explicit fixture cache variables:

| Variable | Workspace-relative value |
| --- | --- |
| ROBO_DYNA_VEHICLE_CANONICAL | crash-work/assets/yaris-vehicle |
| ROBO_DYNA_VEHICLE_SCOPE | crash-work/reports/yaris-full-shell-scope-10.json |
| ROBO_DYNA_VEHICLE_DECLARATIONS | crash-work/reports/yaris-vehicle-declarations-1.json |
| ROBO_DYNA_VEHICLE_ELASTIC | crash-work/reports/yaris-elastic-assembly-inventory-1.json |
| ROBO_DYNA_VEHICLE_MIXED | crash-work/reports/yaris-section-mixed-assembly-inventory-1.json |

Disable unrelated optional suites with `ROBO_DYNA_FULL_SHELL_RECORD_TESTS=OFF`
and `ROBO_DYNA_SOURCE_MAPPING_TESTS=OFF`. Build with `--target
robo_dyna_vehicle_source_check -j1`; run `ctest -R '^vehicle_source_plan$'
--output-on-failure`. The existing bounded fixture wrapper authenticates/extracts
the original member and invokes all four real-source functions, without skips.
Coverage includes every original parent, actual 149/631-parent V3 projections,
copy/move backing, 21 source/late mutations, eight pre-read cap/identity cases,
length/hash rejection and exact retry. These are host source tests only.

Python gates: `PYTHONPATH=tests python3 -m unittest test_vehicle_declarations
test_assembly_sections test_assembly_auxiliary` (nine functions). The first
actual metadata compilation passed in 1.83s/92,500KiB RSS under one CPU/512MiB;
the owning C++ first build passed in 25.43s/295,196KiB. Final host evidence and
source freeze are recorded in `crash-work/reports/vehicle-source-*`. All four
actual-source functions pass in 1.39s/152,504KiB; no CUDA or heavy jobs were run.


Root integration: app `687cee0`. The owning four C++ functions and nine Python
checks pass under the workstation guard; detailed root evidence is
`crash-work/reports/vehicle-source-root-{configure,build,tests,python}-1` and
`vehicle-source-root-functions-1.xml`. Root source review found no blocker.
The C++ gate completed in1.505 s with163,627,008 B peak sampled RSS and no skips.
Actual native reference construction and runtime admission remain separate.
