# Retained Python demos

`Inventory.json` records all 97 original `demo_*.py` mains, their imports,
potential native API references, data-file requests and two local helper files.
The scripts are unchanged. `Admission.json` maps the current real launch targets
and required configurations, with declaration, compilation and runtime kept
separate. This source audit is not a simulation result.

The public package is `robodyna`. The compatibility spelling `pychrono` remains
available because the retained scripts and SWIG imports use it. Both spellings
share the same generated module objects and native implementation owners; no
second physics library or class factory is created. The current inherited Python
class/method names remain available without claiming every canonical C++ rename
has a corresponding new Python spelling.

The package gates use the declared `/usr/bin/python3.10` SDK, actual core
and FEA native extensions, and real shared body/mesh ownership. The baseline
wrapper batch also covers VSG (`vsg3d` in Python), Irrlicht, postprocess, Vehicle,
Robot and Pardiso. It reuses the shared optional implementation DSOs, including
one common image/STB owner. Wrapper generation and syntax checks do not substitute
for native compilation and actual module-import gates.

Remaining profiles are explicit. Sensor array access needs the real NumPy-enabled
wrapper path and the chosen actual renderer. CRM needs coherent FSI/SPH/Vehicle
bindings. YAML/URDF/Cascade and ROS depend on their corresponding native modules
and matching generated declarations. Native backend availability is not enough
if that interface is still absent from the generated Python module.

Among these 97 mains, 15 directly import NumPy, two use matplotlib, and three
Cascade cases need pythonOCC. The unrelated TensorFlow/OpenCV training and utility
scripts are outside this exact main-program roster. NumPy and Matplotlib are supplied by separate pinned workspace SDKs, whose actual
array/plot probes are separate from wrapper and demo tests. The original native
SDK interpreter is not modified, and unrelated Python environments are not used.

Runtime assets remain separate from compilation: a literal data filename does
not include meshes/textures selected inside constructors or referenced by JSON,
OBJ/MTL, glTF or exported CAD scripts. Launchers must select the real data root,
preserve relative helper imports and original arguments, and use a fresh output
directory and the workstation guard. Interactive loops and GPU allocations are
not executed merely to list or build these programs.

The [package map](../../bindings/python/README.md) names the actual compositions.
All 97 original mains now have declared routes. The three CAD routes use the
source-built PythonOCC SDK and the same OCCT owner; their native shape exchange
and lifetime gate passed. This is not a count of completed simulations.
Full Sensor images, nonempty array-buffer lifetimes, SPH stepping,
GUI operation and ROS transport each require their relevant runtime gates.

Run a selected target with `--robodyna-data-root` and a new
`--robodyna-output-dir`, followed by `--` and any original script arguments.
The launcher uses the admitted isolated CPython interpreter and preserves the
script's original loop, physical timestep and output behavior. ROS routes bind
the real declared node into that new work directory; its own launcher configures
middleware, which is not linked into the simulation process.

The two plotting demos still construct Irrlicht windows. The FEA shell script
also calls `plt.show`; the qualified Agg image/PDF dependency test does not supply
an interactive Tk/Qt backend. The original MBD crank plotting script runs 20 s
(despite its 2 s comment) and has no final `show` or `savefig`. Those behaviors are
preserved rather than rewritten for a declaration count.
