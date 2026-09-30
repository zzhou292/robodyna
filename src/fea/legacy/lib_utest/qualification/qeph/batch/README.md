# BQ1/BQ2: bounded resident QEPH history and cached force

All eight CUDA functions passed on their first numerical execution at the
frozen budgets below. The production participant composes the retained Q3c
operation and T2 nodal owner. It does not
contain native Fortran, a second clock, or an admission for free shell dynamics.
The opt-in `qeph_batch_check` target links native Q2 only as a test oracle.

The initial domain is 1–4 Q4 cells covering at most 16 free physical nodes,
positive native mass/isotropic inertia, unchanged LAW1/DM/DN/ISROT0/IDRIL0,
and exact reference geometry at rest at epoch zero. Host startup references
are checked against their own `InitializeReference` producer. One module
allocation holds immutable model, two complete element-history/cache slabs
and control scratch, capped at 1 MiB. No allocation occurs during steps.
Initial zero internal cache has no completed interval/kinematic diagnostics;
actual rest/mass validation gates every result read and candidate.

`BatchUsage::PrescribedFields` permits joint publication along the explicitly
prescribed nodal operands without consuming cached QEPH loads. `CoupledForces`
requires assembly in the same owner/epoch/attempt. Usage is immutable and the
diagnostic records participation. This check prevents accidental omission;
neither usage nor a scalar receipt proves a coupled stability/accuracy argument.
The coupled test below remains identically at rest. Nonzero free response is
blocked on the whole-recurrence admission in `planning/QEPH_BATCH_IMPLEMENTATION.md`.

Eight actual-CUDA functions passed:

- One/two/four shared and disjoint cells, up to 16 nodes, initial binding,
  accepted result phases and unchanged allocation counts through publication.
- Forty prescribed native cell intervals over planar/warped shared patches,
  all Q3c fields and independent global shared-node mass/J kinetic ledgers.
- Nonzero cached force/couple signed scatter against actual CUPDTN3, shared
  addition, duplicate sticky rejection, unchanged accepted history after capture.
- Immutable prescribed/coupled usage, missing force participation, rejected
  direct owner commit and clean zero-load joint-publication retry.
- Bad references, capacity, shared identity, actual mass/inertia and rest states;
  no initial result may be read following failed binding.
- Foreign/stale/tampered operands, full result and diagnostic byte preservation,
  late receipt failure and comparison to the owner's re-borrowed authentic token.
- A late fourth-cell geometric failure after a nonzero accepted history,
  rollback and clean retry identical to an uninterrupted prescribed sequence.
- A safe invalid kernel launch after candidate evaluation: the owner detects
  it before epoch publication, both participants remain unpublished/poisoned,
  and a poisoned batch still marks a fresh assembly sticky.

Budgets frozen before execution: complete native comparisons reuse Q3c's
unchanged `2e-12*(physical dimension+abs(reference))` per field. Independent
kinetic ledgers use `2e-12*max(abs(reference),1e-24 J)`. Exact cache copies,
native signed scatter and clean retries require exact values; failed output
snapshots require complete unchanged bytes. No time/speed assertions or new
dynamics tolerance is introduced. Missing CUDA fails instead of skipping.

The tests reuse `qualification/nodal/NodalTemporalFixture.h` for the actual
owner readback/load helpers and Q3c's named native comparison utilities.
Only prescribed loads advance nonzero trajectories in this gate; inspected
dependent cached assembly is discarded. The independently admitted T2
operation and its required receipt are used without a constant-load bypass.
Root builds/runs serially under `run_bounded.py` and `workstation.lock`.

First-execution evidence under `crash-work/reports` is
`qeph-bq12-{configure,build,cuda-tests}-1.json` and
`qeph-bq12-cuda-xml-1/qeph_batch_check.xml` (8 tests, no failures).
The batch owns exactly 14,824 device bytes in one allocation. Build wall time
was 11.537 s with 403,243,008 bytes peak sampled host RSS; the CUDA guard
completed in .613 s (GTest .260 s). These process measurements do not establish
kernel throughput or peak CUDA-context memory. The safe zero-thread-launch
fixture accepted the already retained T1/T2 error-code pair before this first
run; numerical source and tolerances were unchanged. Legacy scatter regressions
and owning Bazel registration are separate root-run checks. Nonzero coupled
force recurrence, its work ledger and free-response accuracy remain BQ3.
