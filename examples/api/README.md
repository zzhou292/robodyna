# First Robodyna C++ examples

These small headless examples use the new public names while retaining the
existing CPU mechanics implementation. Each advances 1000 steps of 0.0001 s,
checks the final spring response against the continuous oscillator solution, and
prints its position and velocity. They request one solver thread and attach no
renderer. No model files or GPU are needed.

```sh
bazel run //examples/api:rigid_spring
bazel run //examples/api:fea_spring
bazel test //tests/api:api_tests
bazel test //tests/time_api:time_tests //tests/io:paths_test
```

Workspace agents must launch these through the existing bounded build/test
procedure in `docs/migration/EXECUTION.md`, rather than concurrently with another
heavy job. The first build can compile the native mechanics aggregate.

`rigid_spring` connects an easy box body to a fixed body using a TSDA.
`fea_spring` connects two FE nodes using the inherited spring element. Both use
mass 3, stiffness 12, rest length 1, initial extension 0.1, and zero gravity or
damping. The expected displacement is calculated for validation only; the actual
trajectory is advanced by the existing mechanics solver.

The public API now includes actual Robodyna definitions, retained type aliases
and small forwarding functions. Their ownership differs:

| Family | Current implementation |
| --- | --- |
| Mass properties and inertia utilities | Actual definitions in `robodyna::mechanics`, backed by the neutral inertia owner. |
| Body, BodyAuxRef and all eight Easy-body types | Actual definitions in `robodyna::mbd`. |
| Mesh | Actual definition in `robodyna::fea`. |
| System, SystemNSC, SystemSMC and Assembly | Actual definitions in `robodyna::simulation`; Assembly owns the existing mixed participant composition. |
| Math types, BodyFrame, PhysicsItem, TSDA, FE xyz node/spring, contact materials, solvers and timesteppers | Robodyna aliases to their retained inherited definitions. Allocation and vector helpers import existing functions. |

For the moved definitions, the legacy names are reverse aliases: `ChBody` names
the actual `RbBody` type. Both spellings still denote one implementation. Body,
Mesh, System and Assembly still compile in the combined native FEA/MBD backend;
their module-specific names do not yet establish independent domain libraries
or new CUDA dynamics. See the [API ownership map](../../docs/migration/API_ALIAS_MAP.json)
and [qualification record](../../docs/migration/RENAME_QUALIFICATION.json).

Native C++ now supports `GetTime()` and `SetTime(double)` on System and the
existing object hierarchy, including Body, Mesh and Assembly. They forward to
`GetChTime()` and `SetChTime(double)`, which remain supported and are still used
by these example sources. A setter changes only its owner's existing timestamp;
it does not advance the simulation, update other participants or synchronize the
timestepper's separate workspace clock. The legacy binding surface intentionally
retains its old method names for now.

The native header `robodyna/io/RbPaths.h` provides `robodyna::io::SetDataPath`,
`GetDataPath`, `GetDataFile`, output/test-output accessors and the two existing
`CreateOutputDirectory` overloads. It uses the same three existing path stores;
there is no separate Robodyna asset directory state. Data-file lookup concatenates
strings literally, so supply any required trailing separator in the configured
path. Output getters retain their directory-creation side effect. Configure paths
before constructing a renderer or starting concurrent consumers. These new IO
names are also native-only; existing legacy helper bindings are unchanged.

The time and IO additions passed their focused native tests. Broader binding,
CMake and product qualification is recorded separately; native method availability
does not establish a branded Python or managed C# API. The remaining work includes
moving the still-aliased implementation families, proving independent FEA/MBD
dependency boundaries, and qualifying the remaining optional-module and binding
surfaces. Progress is tracked by those concrete families and gates, not by a
percentage derived from this small initial API map.
