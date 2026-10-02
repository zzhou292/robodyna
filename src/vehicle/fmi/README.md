# Vehicle FMI models and drivers

Six native FMUs reuse the original Vehicle FMI components, current native vehicle
library, retained FMI exporter and pinned fmu-forge source. No old binary solver
is imported. The five original demonstration mains live under
`//examples/vehicle/fmi:native_demos`.

| FMU target | Original model identity | Composition |
| --- | --- | --- |
| `wheeled_vehicle` | `FMU2cs_WheeledVehicle` | Vehicle without tires or powertrain |
| `wheeled_vehicle_powertrain` | `FMU2cs_WheeledVehiclePtrain` | Vehicle and powertrain, external tires |
| `powertrain` | `FMU2cs_Powertrain` | Standalone shaft engine/transmission |
| `tire` | `FMU2cs_ForceElementTire` | Force-element tire with external wheel/terrain data |
| `path_follower` | `FMU2cs_PathFollowerDriver` | Co-simulation steering/speed controller |
| `path_follower_model_exchange` | `FMU2me_PathFollowerDriver` | Controller states assembled into the host System |

`exports.json` pins original identities, UUID derivation, source owners, resources
and visualization selection. Each archive contains the actual newly linked model,
its native-generated XML, original resource directory and source/license notices.
Visual co-simulation models retain their actual Irrlicht implementation. The
admitted Irrlicht DSO is copied beside the native model and resolved with
`$ORIGIN`; original Linux graphics/image-codec dependencies remain platform
prerequisites. Bounded physics tests instantiate these same visual-capable FMUs
with `visible=false`; they do not claim GUI/runtime rendering qualification.

Each FMU keeps its own intended component state. Its native C++ symbols are hidden
behind the FMI C interface, preventing host/FMU class-factory or quadrature-state
interposition. Co-simulation callers still drive component steps at the original
communication cadence. The model-exchange controller enters the existing host
System clock. No coupling algorithm or timestep is replaced by this build port.

## Running original demos

Build from the baseline configuration:

```text
bazel build //src/vehicle/fmi:fmus //examples/vehicle/fmi:native_demos
bazel test //tests/vehicle/fmi:tests
```

The drivers use their original explicit-FMU argument branch. Their driver-only
configuration places extraction directories in the current working directory;
it does not write into Bazel runfiles. Run each program in a fresh writable
directory and supply absolute paths to the archives below, in order. Preserve the
original data layout: these programs also read the complete vehicle data tree
and several contain a literal `../data/vehicle/` path. They retain interactive
run lengths and output behavior; building them does not authorize an unbounded
simulation or constitute a runtime receipt.

| Program under `//examples/vehicle/fmi` | Archive arguments under `bazel-bin/src/vehicle/fmi/` |
| --- | --- |
| `vehicle_native_subsystems` | `wheeled_vehicle.fmu` |
| `vehicle_fmu_subsystems` | `wheeled_vehicle.fmu powertrain.fmu path_follower.fmu tire.fmu` |
| `vehicle_powertrain_native_subsystems` | `wheeled_vehicle_powertrain.fmu` |
| `vehicle_powertrain_fmu_subsystems` | `wheeled_vehicle_powertrain.fmu path_follower.fmu tire.fmu` |
| `path_follower_model_exchange` | `path_follower_model_exchange.fmu` |

## Admission and inherited limits

Source gates authenticate 57 retained source/data files, six original component
identities and five actual main functions. Artifact gates inspect the six linked
ELF interfaces, archive/XML identities and visual-dependency packaging. Native
gates check uniform-gravity response of both vehicle compositions, tire separation
versus compression, speed-controller response, and model-exchange initialization
plus a common host step with the original constant target speed of 12 m/s.
These are focused migration checks, not complete vehicle FMI validation.

Two existing numerical issues remain visible in the preserved sources:

- Standalone Powertrain calls `powertrain->Advance(h)` while its packaged shaft
  engine/transmission Advance methods are empty. Its owning System is not stepped.
  Loading and finite output are admission checks only; this does not establish
  shaft dynamics. The vehicle+powertrain model advances its actual vehicle System.
- The model-exchange controller binds longitudinal integral metadata to index0
  instead of index1, and its derivative implementation returns state values rather
  than controller errors. The retained demonstration uses zero integral states
  and zero integral gains. The runtime gate covers that default proportional
  profile and original input API; nonzero-integral correctness is not claimed.

No source fix is hidden in these target declarations. The original constant-speed
SetRealParameterValue call is valid: the retained wrapper explicitly accepts
continuous inputs through that API. The original main remains unchanged.
