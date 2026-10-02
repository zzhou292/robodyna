# Vehicle runtime input closure

This is the runtime plan for the new native demo targets, not a claim that all
assets have already been packaged or exercised. Compilation consumes code and
headers; a useful run also needs the complete transitive model description and
geometry, a renderer SDK/asset set, and a fresh writable output location.

## Preserve the two existing data owners

Core assets use `SetChronoDataPath` / `GetChronoDataFile`; Vehicle models use the
separate `SetVehicleDataPath` / `GetVehicleDataFile` store. Their defaults remain
`../data/` and `../data/vehicle/`. Those are working-directory conventions, not
portable Bazel runfile resolution. VSG snapshots its data path during construction,
so configure the actual selected directory before constructing the visual system.

Do not embed an arbitrary developer checkout path or equate a directory name with
a verified input closure. A later launcher/profile must resolve an explicit file
anchor from runfiles or an authenticated external model pack, set both stores,
and put outputs in a create-only run directory. Preserve the existing CMake data
directory interface when providing installed/native-package compatibility; the
new build declarations do not silently invent another global asset store.

## Close references transitively

For each chosen demo/profile, generate a bounded input manifest covering:

1. The entry JSON and referenced chassis, suspension, steering, powertrain, tire,
   terrain and driver JSON files. Resolve each reference using the actual API's
   core/Vehicle data-root convention, retaining the original relative paths.
2. All mesh resources. For OBJ, include referenced MTL files and each material's
   image maps; for glTF, include external buffer and image URIs, with embedded
   data handled explicitly. A GLB can still refer to external images: the binary
   container alone is not proof of closure. Other supported mesh/heightmap formats
   need their format-specific companion files as well.
3. Terrain heightmaps, road/CRG input, tabular tire/driver/controller data, textures
   and any profile-selected sensor or robot resources outside the Vehicle root.
   Runtime-selectable vehicle models must select a corresponding manifest; one
   fixed HMMWV pack cannot represent every model chosen by the original menu.
4. The existing VSG/Irrlicht assets, fonts, colormaps and Robodyna logo. These use
   their current owners and checked renderer inventories rather than duplicating
   image libraries or copying an older Chrono installation.
5. For optional FMI/MPI cases, the actual FMUs/helper processes, platform ABI,
   launch ranks and companion configuration. Building the driver does not produce
   or validate those runtime artifacts automatically.

Record logical relative name, byte count, SHA-256, reference origin and owning
root for each file. Reject unresolved, escaping or unexpected network references
instead of fetching them during a solver run. Retain original assets and failed
closure reports; do not fabricate missing mesh/material files. Validate source
inventories before and after a run or replay when promoting a result to evidence.

## Admit and qualify runs separately

Start with short cases using primitive or closed local model inputs. Capture
declared resource limits, timestep, selected model/solver, source/library hashes
and all input-manifest hashes. Then verify initialization, a bounded number of
real steps, finite states and case-specific invariants. Rendering adds an actual
visual review and decode gate; it does not establish numerical accuracy.

The first build batch preserves CPU SCM semantics. CRM, OpenCRG, SCM GPU and
co-simulation require their own coherent module configurations and runtime tests;
enabling their macros in a demo alone is not implementation or qualification.
