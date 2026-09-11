# Shell scope on the complete physical domain

`ShellPhysicalBinding` retains an existing complete nodal coefficient ledger
and shell failure catalog, which already owns its material declarations. The
shell binding and source-node map come from that ledger. No node/material data
is copied, and the legacy shell-local combined-mass input is rejected here.

The four host functions qualify scrambled physical node order, actual layered
QEPH/T3/QBAT topology, an extra typed point-mass producer with J0, missing
coverage, foreign material/failure scope, exact byte-cap retry and retained
source/curve lifetime. A declared zero mass remains zero: this source join does
not admit a DOF or invent an inertia/mass floor.

Root evidence: `shell-physical-binding-root-tests-1`. The two owning Bazel
targets are `//lib_src/assembly:shell_physical_binding` and this package's
`host_check`. CUDA participant initialization, CIN stiffness and full common
publication are the next integration steps; this object alone does not enable
the vehicle trajectory.

Standalone gate:

```sh
cmake -S lib_utest/qualification/shell_physical_binding -B BUILD
cmake --build BUILD --parallel 4
ctest --test-dir BUILD --output-on-failure
```

The payload budget conservatively charges both complete retained producer
reports, including any shared inventory counted by each. It accounts for the
new handle/control structure but allocates no count-sized new arena. Inputs
must be preprepared and valid; this API neither parses source files nor proves
that omitted model families were physically optional.
