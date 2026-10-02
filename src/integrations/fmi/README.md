# FMI import and export

The native owner compiles the retained `ChExternalFmu.cpp` and FMI 2/3 wrapper
headers against current Robodyna mechanics. Its dependency is the exact
fmu-forge gitlink recorded at import:
`cbf2a58ba9518e6230e831ff0b3640f9a0b4905a`. `dependency.json` authenticates the
official source archive; the preserved imported gitlink directory is untouched.
The library source and all license notices remain intact.

The FMI examples produce **eight FMUs and five driver executables**, counted
separately. Each FMU compiles its original component and the retained fmu-forge
exporter. It links the current statically compiled mechanics implementation;
it does not load an older Chrono physics library. The original native
`FmuForgeModelDescription.cpp` helper loads that new shared object and generates
its real XML descriptor. Only then does the packaging action validate identity,
mode and version and create the ZIP, preserving binary/resource bytes.

`export/` owns the small packaging rule and utility. `runtime/` owns input-path
resolution using the existing rules_cc runfiles library. `exports.json` in the
example package records the eight original model identifiers and explicit stable
GUID namespace. This uses the same UUIDv5/SHA1 operation as the retained CMake
when given an explicit namespace. Model equations, solver choices and time
advancement are unchanged.

The template's two stale `ChFmuTools*.h` includes now name the existing
`ChFmuForge*.h` headers. `SOURCE_TRANSFORMATIONS.json` reverses these exact two
substitutions to their original hashes. `SourceBaseline.json` authenticates all
50 retained FMI source/resource files against the preintegration checkpoint.
No replacement implementation or compatibility class was introduced.

This first profile is Linux x86-64 with the qualified baseline mechanics
configuration. The independent OpenMP and FE multiphysics profiles remain
incompatible until separately admitted. Optional Irrlicht inside the exported
FMUs is disabled, as supported by the original CMake. The hydraulic-crane model
exchange driver uses actual VSG. Exported resources and upstream license/notice
files are copied into the FMUs and hashed in their packaging receipts.

Build targets, under the shared workstation guard:

```text
//src/integrations/fmi:fmi
//examples/integrations/fmi:fmus
//examples/integrations/fmi:native_demos
//tests/fmi:tests
```

The packaging tests cover paths, identity, byte preservation and native-helper
failure; their mocked helper is explicitly not a physics qualification. The
source test checks actual CMake identities, all five original entrypoints,
source hashes and the dependency pin. The native test loads the real FMI 2/3
Van der Pol exports and advances them through the existing System clock.
Compilation and that runtime gate must pass before describing this profile as
qualified.

Each FMU target's default artifact is its `.fmu`. The `metadata` output group
contains XML, a build specification and a SHA receipt; `unpacked` exposes the
actual packaged directory. The bundled prebuilt `fmusim` is not used by this
initial source/native admission.

Driver input paths resolve through declared runfiles. Their inherited unpack
directories remain relative to the current directory, so launch them from a
fresh guarded output directory. They retain their original horizons, prompts
and visualization behavior; compiling the aggregate does not run the demos.
The inherited importer does not provide a complete unload/lifetime guarantee,
and this bounded gate does not claim reusable import/unload cleanup or full FMI
standard certification.

FMI archive bindings require the `ChOutputFMU` serializer to live as long as the
registered variables. The retained `FmuChronoComponentBase` owns that serializer
as a member and satisfies this contract; a standalone temporary serializer is
not qualified. Constant metadata owns its copied values, while ordinary member
bindings retain their original live references. The actual template admission
test reads constants and live clock/counter fields before and after a real step.
