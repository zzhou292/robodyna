# Public and compatibility Python packages

Robodyna packages assemble actual generated proxies and native extensions. The
resolved `package.json` target's parent is the import root. `robodyna` and the
compatibility spelling `pychrono` expose the same module/class objects; they do
not load two physics backends. The exact declared interpreter is CPython3.10.

| Package target | Required feature configuration | Purpose |
| --- | --- | --- |
| `//bindings/python:core_package` | Baseline | Qualified original core, NumPy disabled |
| `//bindings/python:core_fea_package` | Baseline | Qualified core and FEA, NumPy disabled |
| `//bindings/python/baseline:package` | Baseline | Core/FEA, VSG, Irrlicht, Vehicle, Robot, postprocess, Pardiso |
| `//bindings/python/numpy:core_package` | Baseline | Separate generated native NumPy C-API bridge |
| `//bindings/python/numpy:package` | Baseline | NumPy plus the eight baseline modules |
| `//bindings/python/plot:package` | Baseline | NumPy, Matplotlib and Core/FEA/Irrlicht/Pardiso |
| `//bindings/python/profiles:fsi` | `fsi-sph`, `vsg`, selected CUDA architecture | Real SPH/CRM/native module composition |
| `//bindings/python/profiles:parsers` | `yaml` | Original combined URDF/YAML parser interface |
| `//bindings/python/profiles:sensor` | `sensor-optix`, selected CUDA architecture | Original rendered-Sensor interface plus host sensors |
| `//bindings/python/profiles:ros` | Baseline | Actual simulation handlers; middleware is a separate node |
| `//bindings/python/profiles:ros_sensor` | `ros-sensor`, `sensor-optix`, selected CUDA architecture | Coherent ROS/Sensor declarations and implementations |
| `//bindings/python/profiles:ros_urdf` | `yaml`, `ros-urdf` | Native TF/URDF adapter and matching parser wrapper |

These are package definitions, not claims that every optional profile has passed
its native import or full simulation gate. Current evidence belongs to the
operator verification inventory and its closed receipts. The source-built
PythonOCC/Cascade package remains pending its SDK and shape-exchange qualification.

One interpreter selects one complete package. The loader rejects a second package
before native loading, even if code changes `sys.path` or retries an import.
Implementation DSOs use their original declared runfile layout so compiled ELF
`$ORIGIN` paths remain valid. The same core/factory, shared STB image owner and
selected module implementations are reused by every extension in that package.
The local MKL owner explicitly carries its authenticated vendor runtime layout.

NumPy and plotting wheels live in separate declared SDK directories. Their files,
versions and origin are authenticated; no ambient pip environment is borrowed.
The baseline no-NumPy package is preserved. Initial optional wrappers reject
Multicore, extra FE fields and TDPF configurations until their matching language
interfaces and runtime gates are qualified together.

For embedded C++ Python, add the declared package import root before constructing
the retained `ChPythonEngine`; dynamically depend on the same native core and
disable bytecode writes in generated/runfiles trees. Keep the original native
object and pointer lifetimes: a borrowed field view is not an owning checkpoint.
