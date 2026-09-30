# Complete original shell binding

`VehicleShellBinding` connects the authenticated full original reference
assessment to one TL-FEA QEPH/T3/QBAT `ShellBatchBinding`. Each original parent
retains its source row, formulation-local index and constitutive/rigid role.
The adapter reuses the exact reference inputs and TL's existing initializer;
it introduces no material or mass formula.

This prepares the shell-local domain. Beam/solid/rigid-extra nodes, constraint
coefficient transfers, independent DOFs and the common runtime owner remain
separate integration steps. The5,102 rigid-shell roles are preserved; their
reference mass does not authorize an ordinary elastic shell force update.

Root qualification on2026-09-11: all3 actual-source integration functions PASS
in `crash-work/reports/vehicle-shell-binding-root-tests-1.json`. All349,645
source geometries match by named native fields and all359,785 node IDs and SI
coordinate bits agree. Counts are324,094 QEPH /21,301 T3 /4,250 QBAT. Independent
per-parent reductions verify once-only contributions, including coincident
layers. Incomplete profiles and allocation limits reject before publication;
immutable copies and retry preserve the accepted binding.

Shell-only mass is673.875154 kg; this is not complete vehicle mass. Native
binding payload300,494,216 B and startup scratch28,074,817 B fit their explicit
reservations. The adapter conservatively forecasts2,015,266,268 B including
the retained source/reference bound and full configured TL reservations;
the three-test run samples1,409,785,856 B RSS under2 CPUs/4 GiB and takes1.757 s
including the guard. The2 GiB host cap is separate from the2 GiB archive budget.

Include `../VehicleShellBinding.cmake` to reuse the producer. This directory's
standalone CMake gate requires the same original canonical/scope/declarations
and authenticated glass-resolution paths as the existing reference gate.
Run only `ctest -R '^vehicle_shell_binding_'` for this increment. No native
donor, existing production consumer, CUDA state or Chrono code changed.
