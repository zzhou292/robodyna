# Complete resident collections with explicit TAB1 glass failure

The existing `ShellBatchFailureBinding` now owns `ShellLayeredTab1Parameters`
for an explicit `Tab1AnyPoint` parent. Admission requires its complete retained
catalog to resolve analytic LAW44, `FilteredZeroC`, single-layer NIP3 and native
AnyPoint removal. The per-parent reference may select the separately qualified
centered, top, or bottom reference plane; quadrature remains the qualified
shifted NIP3 rule. No source MID, NLOC interpretation, NUMINT conversion, contact
eligibility or whole-vehicle admission is added here.

Every parent remains in its original QEPH/T3 collection and source input order.
None and ConstantAllPoints retain their prior value operations. TAB1 dispatch
calls the qualified full family adapter; geometry, native coefficients, force
arithmetic, and work are shared. Placement is admitted only through the exact
immutable geometry/catalog/failure binding. A per-step override is not exposed.

One optional failure arena contains explicit policy/parameter arrays and two
family-indexed history slabs. The existing shell accepted selector selects both
material and failure history. There is no additional clock, commit, allocation
per step, or batch per material. The common publication coordinator compares the
complete failure binding, including the other family's rows.

`ShellBatchFailureState` has one explicitly constructed tagged point payload.
`constant_points()` and `tab1_points()` expose only the selected point type;
None returns neither. TAB1 retains uncapped native damage, separate capped
maximum damage, actual failure time, table cache, and point activity. Current
unmasked force-point stress and final parent activity remain separate from saved
material-point stresses. Only active named fields are an observation contract;
padding and inactive union bytes are not values or identities.

The owning host staging reconstructs the correct member before CUDA byte copies,
including after a rejected read. It checks the copied policy against immutable
source policy before reading the union. Infallible final copy assignment starts
the corresponding member lifetime in caller outputs under C++17. Bad tags,
nonfinite/invalid history, stale identity and incomplete capacities do not
publish outputs or select a trial slab. CUDA errors retain the existing poison
contract; numerical rejection retains the existing common retry path.

## Resource scope

The default 1,024-parent / 1 MiB failure-device / 16 MiB failure-host limits and
explicit 524,288-parent / 256 MiB / 512 MiB vehicle limits remain unchanged.
Every optional byte must additionally fit the existing 2 GiB family budget.
The typed state and parameters change in-process sizes; no persistent archive
ABI is changed. Host accounting includes the complete owned failure declaration,
temporary startup arena, retained readback, and shared scope exactly once.

The complete no-tire count layout test (328,344 Q, 21,301 T, 359,785 nodes,
1,024 curve points per family) reports 1,617,558,576 B for Q's complete family,
104,367,860 B for T, and 174,822,584 B for their failure arenas together.
These are checked layout forecasts, not native admission of every original part;
nodal/contact/publication and other system budgets remain separate.

## Qualification

The host suite checks complete source/parameter/placement identity, rejection and
retry, safe typed copy/assignment, uncapped damage, display cap, bad flags, and
mixed None/Constant/TAB1 dispatch across all planes through removal and two later
intervals. The old seven constant-failure host tests remain owning regressions.

Two native tests use the unchanged complete placed Q/T oracle with its actual
native M%SSP, all eight initial point masks, both offset signs and centered
geometry. They compare every observable force, saved stress/PLA/rate, current
force-point stress, TAB1 field, section diagnostic and work channel. First-point
removal and all-point removal are distinguished; two inactive updates follow.
Transient constitutive increments are not invented as resident readback fields.

Four actual CUDA cases use complete four-Q/four-T collections and 28 owner nodes:
native recurrence after real owner history transactions/common publication,
placement and sibling-scope rejection, last removed-parent geometry failure with
exact retry, and late tagged-history readback failures with unchanged outputs.
These are prescribed-history integration qualifications, not crash trajectories.
No new native or CUDA execution is claimed by the host author checks.

```sh
cmake -S lib_utest/qualification/resident_shell_tab1 -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DTL_RESIDENT_TAB1_NATIVE=ON \
  -DTL_RESIDENT_TAB1_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> --parallel 2 --target resident_shell_tab1_host_check \
  resident_shell_tab1_native_check resident_shell_tab1_cuda_check
ctest --test-dir <build> -j1 --output-on-failure -R '^resident_shell_tab1_(host|native|cuda)$'
```

The host target also belongs to Bazel:
`//lib_utest/qualification/resident_shell_tab1:resident_shell_tab1_host_check`.
Root schedules native/CUDA and the existing resident constant/mixed regressions.


Root integration (2026-09-10) passes all11 owning functions:5 host,2 native and4
actual CUDA. Accepted reports: `resident-shell-tab1-root-tests-1` / functions1.
Build1 exposed missing root Fortran language initialization for the cross-directory
native runtime link; root enable_language(Fortran) fixes it and build2 passes.
No resident equation or tolerance changed.

A subsequent independent review identified a pre-existing shared Q/T LAW44
sound-speed handoff omission in both production and retained native force
wrappers. TL `b12cd18` corrects it; all11 owning functions pass again with the
actual returned native point SSP (`law44-ssp-root-resident-tab1-tests-1` /
functions1). The corrected force gates also pass24 ordinary/failure and13
placement/glass functions. Historical results remain unchanged; remaining
affected resident/source-flight regressions are tracked in the active plan.
No full vehicle source/run is admitted by this gate.
