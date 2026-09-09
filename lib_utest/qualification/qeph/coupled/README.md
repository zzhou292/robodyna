# BQ3 short coupled recurrence qualification

The first numerical execution passes all three actual CUDA tests in
`qeph_coupled_check`; native Q2 is linked only as its independent test oracle.
The production `QephBatch`, `FENodalState` and T2 history admission are unchanged.

The frozen pre-run contract is `planning/QEPH_COUPLED_GATE_DESIGN.md` in the
workspace. One 20 mm square under opposed edge couples and two shared 20 mm Q4s
under balanced transverse forces use E=200 GPa, rho=7890 kg/m³, nu=.3 and
t=1.648 mm. Native total inertia and DM/DN remain unchanged. Starting from rest,
four intervals at 2^-24 s and eight at half that step exercise initial half kick,
endpoint-cache feedback, zero external load and reversal. The separate receipt
rejection test starts with a nonzero accepted cache and retries the same endpoint.

`QephCoupledFixture` reuses BQ mesh/owner/readback/load helpers.
`QephCoupledNative` composes native element calls with fixed-array expected nodal
values; it publishes those expected values only after the CUDA joint commit.
`QephCoupledLedger` independently accounts actual assembled forces, kick energy,
linear and angular momentum, and cached internal kick/drift work. The angular
identity includes the measured roundoff in stored endpoint position. Full native
history/force/rate comparisons reuse the unchanged Q3c field budgets.

Forty native cell-interval evaluations and forty-two CUDA cell evaluations pass
across the three functions; these counts include the rejected CUDA
candidate. Global kinetic accounting sums shared native m/J once per node.
The tests require observable wrong-full-kick and omitted-cache controls; a zero
motion result cannot pass. No per-step device allocation is added, and the tests
check the original batch/owner allocation counts and sizes throughout.

The independent source-work ledger also checks all six aggregate EINT/EVIS
totals and increments before passing receipts, including rejected/retry cases.
The smallest omitted-cache control is 46.384 times the frozen nodal budget
(required >32); the maximum observed angular-momentum budget ratio is
2.478e-8 and source-work ratio is 6.015e-14. These are implementation checks,
not a measurement of the method's physical or temporal error.
Evidence is in workspace `crash-work/reports/qeph-bq3-cuda-tests-1.json` and
`qeph-bq3-cuda-xml-1/`; the original build and test-only ledger rebuild are
retained separately. No production equations or tolerances changed.

These are short complete recurrence/publication checks, not a long-trajectory
stability, order, source-MAT024, contact, constrained-shell or vehicle gate.
Every numerical failure must prevent receipt publication. The longer pulse,
unforced response and h/h2/h4 decision remain separately unexecuted.
