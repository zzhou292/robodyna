# Inherited capability coverage

Robodyna retains all imported Chrono and TL source. Qualification is narrower than
retention: a source import does not establish a working root build, runtime parity,
or GPU execution. The source pins and preserved histories are in `SOURCES.json`.
The checked source census is Chrono's `src/CMakeLists.txt` at that pin.

Paths below start beneath `src/compatibility/chrono/src/` unless noted. Proposed
owners describe the destination architecture; they are not empty packages or
already completed migrations.

| Retained source | Proposed Robodyna owner | Root build / execution evidence |
| --- | --- | --- |
| `chrono` core math, geometry, archives | `core`, `geometry`, `numerics` | Native aggregate and host tests passed; first neutral extraction qualified |
| `chrono/physics` bodies, joints, shafts, motors | `mbd`, shared `mechanics`, mixed `simulation` | Native aggregate, rigid host tests and original spring/NSC six-second videos passed; independent MBD gate pending |
| `chrono/fea` mechanical, thermal and scalar elements | `fea` | Native aggregate and focused FE host tests passed; full domain coverage and independent FE gate pending |
| `chrono/collision`, contact classes | `collision`, `contact`, domain adapters | Included in native aggregate; inherited profiles retain separate semantics |
| `chrono_mumps` | `numerics/linear` optional backend | Source retained; SDK and native target qualification pending |
| `chrono_pardisomkl` | `numerics/linear` optional backend | Source retained; SDK and native target qualification pending |
| `chrono_fsi` SPH and TDPF | `sph`, `potential_flow`, FSI coupling | Source retained; native build and coupled regression pending; HydroChrono dependency not initialized |
| `chrono_parsers` | `io` | Source retained; native targets and external format SDK qualification pending |
| `chrono_irrlicht` | `visualization` optional backend | Source retained; native target qualification pending |
| `chrono_vsg` | `visualization` | Native library/viewer build, SDK and asset tests passed; short and 100 ms CLI media workflows passed |
| `chrono_fmi` | `integrations/fmi` | Source retained; fmu-forge dependency not initialized; native target qualification pending |
| `chrono_cascade` | `io` CAD adapter | Source retained; OpenCASCADE SDK and native target qualification pending |
| `chrono_modal` | `analysis/modal`, `analysis/reduction` | Source retained; mixed rigid/FE assembly dependency must remain explicit |
| `chrono_postprocess`, `importer_blender` | `postprocess` | Source retained; existing Robodyna accepted-state video helpers are separately qualified |
| `chrono_peridynamics` | `peridynamics` | Source retained; current FE node inheritance remains declared until extracted |
| `chrono_multicore` | CPU `execution`, numerical/contact backends | Source retained; native targets and focused runtime qualification pending |
| `chrono_dem` | `dem` | Source retained; distinct from optional TL DEME integration; native qualification pending |
| `chrono_vehicle` | `vehicle` | Native CPU SCM terrain and six-second lugged-wheel/rut demonstration passed; broader vehicle subsystems pending |
| `chrono_models` | `models/vehicle`, `models/robot` | Source retained; reusable catalogs and asset admission pending |
| `chrono_vehicle/cosim` | vehicle coupling / integrations | Source retained; external communication/runtime qualification pending |
| `chrono_vehicle/fmi` | vehicle FMI adapters | Source retained; FMI dependencies and qualification pending |
| `chrono_sensor` | `sensor` | Source retained; CUDA/OptiX and other SDK profiles require separate qualification |
| `chrono_synchrono` | `integrations/synchrono` | Source retained; flatbuffers/MPI and native target qualification pending |
| `chrono_ros` | `integrations/ros` | Source retained; ROS environment and native target qualification pending |
| `chrono_swig` | `bindings` | Python/C# interfaces retained; generated binding and archive/API compatibility gates pending |
| `chrono_precice` | `integrations/precice` | Source retained; external SDK and coupled runtime qualification pending |
| `demos`, `tests` | `examples`, `tests`, `benchmarks` | All sources retained; only named focused root gates have run |
| `src/fea/legacy` explicit CUDA mechanics | `fea/backends`, contact and mechanics services | Native build plus 101-step wall/self-contact archive parity passed; original 100 ms evidence preserved |
| `src/fea/legacy` implicit/ANCF, Newton/cuDSS and optimization paths | Opt-in `fea/backends` and `numerics` | Source and explicit target entry points retained; migration runtime qualification pending |
| `src/fea/legacy` DEME contact | Optional `dem` / coupling backend | Exact dependency checkout preserved; not used by the qualified Yaris native contact path |
| `src/compatibility/app` | `simulation`, `io`, `visualization`, `apps` | Production CUDA CLI built; all saved output from the short regression matches the frozen baseline |

The immediate sequence is runnable product parity, neutral mechanics extraction,
then independent FEA/MBD and their assembled coupling. After those boundaries are
stable, port SPH/FSI/DEM, vehicle/model utilities and optional integrations in bounded
batches. Keep inherited CPU backends available while their CUDA replacements are
implemented and qualified. No feature is removed merely because its SDK is absent
on this workstation.
