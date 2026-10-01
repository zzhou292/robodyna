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
```

Workspace agents must launch these through the existing bounded build/test
procedure in `docs/migration/EXECUTION.md`, rather than concurrently with another
heavy job. The first build can compile the native mechanics aggregate.

`rigid_spring` connects an easy box body to a fixed body using a TSDA.
`fea_spring` connects two FE nodes using the inherited spring element. Both use
mass 3, stiffness 12, rest length 1, initial extension 0.1, and zero gravity or
damping. The expected displacement is calculated for validation only; the actual
trajectory is advanced by the existing mechanics solver.

This is the initial alias phase: `RbBody` and `ChBody`, for example, name exactly
the same C++ type. Public headers are module-specific, but these domain facades
still link the combined native FEA/MBD backend. They do not establish independent
domain libraries, new CUDA dynamics, or complete API renaming. Existing methods
such as `GetChTime()` retain their names in this phase.
