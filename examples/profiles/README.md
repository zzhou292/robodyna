# Explicit native demo build batches

`Admission.json` maps 21 additional original mains to real targets and required
configurations. This is a compilation admission list, not a pass receipt. Keep
each phase's guard, complete target list, failures and acceptance evidence separate.
Never treat wildcard-skipped incompatible targets as completed programs.

These target groups supplement the existing baseline Vehicle, Robot and Sensor
groups. Apply the selected configuration in the existing guarded Bazel invocation
with its declared SDK environment; the table is not an unguarded launch command.

| Profile configuration | Build group | Additional mains | Focused gates |
| --- | --- | ---: | --- |
| baseline Vulkan | `//examples/profiles:sensor_vulkan_additions` | 2 Robot Sensor | existing Sensor host, shader and ownership tests |
| `--config=fsi-sph` plus the qualified CUDA architecture | `//examples/profiles:vehicle_crm` | 5 Vehicle and 3 Viper | `//src/vehicle/crm:host_profile_test`, Vehicle aggregate ownership, optional source inventory |
| `--config=opencrg` | `//examples/profiles:vehicle_opencrg` | 3 Vehicle | `//src/vehicle/opencrg:host_terrain_test`, Vehicle aggregate ownership, optional source inventory |
| `--config=multicore` | `//examples/profiles:vehicle_multicore` | 5 Vehicle and 1 RoboSimian | existing Multicore contracts, Vehicle ownership |
| `--config=sensor-optix-sph` plus the qualified CUDA architecture | `//examples/profiles:sensor_optix_sph` | HMMWV fog and actual CRM rendering | OptiX source/owner/host/NVRTC gates, CRM profile gate |

The underlying typed flags are `//build_defs/features:fsi_sph`, `:opencrg`,
`:multicore`, and `:sensor_backend` (`vulkan` or `optix`). The existing
`:fea_multiphysics` flag remains separate. YAML profiles and other module families
join the operator's complete serialized matrix through their own declarations;
this table does not claim whole-repository coverage.

The complete denominators remain 64 Vehicle, 17 Robot and 16 Sensor C++ sources.
These additions give every Robot/Sensor main an explicit proposed build route;
Vehicle MPI/co-simulation and FMI routes still belong to their separate batches.
Keep the existing baseline catalog and these profile rows distinct until actual
build receipts are recorded. Runtime qualification is another step: interactive
or GPU programs are compiled here, never silently executed by a build group.

Required SDK additions:

- `ROBODYNA_OPTIX_ROOT`: the authenticated workspace OptiX/NPP/cuRAND prefix.
- `ROBODYNA_OPENCRG_ARCHIVE`: the pinned original OpenCRG 1.1.2 source archive.
- The existing CUDA math, Sensor graphics, VSG, Irrlicht and applicable sparse
  solver SDK variables remain in the selected guarded recipe.

Sensor runtime shaders require the main repository runfiles root as their working
directory until the product launcher owns that resolution. Preserve original
scene assets, output locations, timestep selection and device/resource admission
when subsequently running a demo. A successful compile or NVRTC coupon does not
certify a GPU simulation or video.
