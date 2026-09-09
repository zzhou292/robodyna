# BQ4 fixed free-response qualification

Source preparation only: no response build, host tests or CUDA trajectory is
claimed here. The preceding native full-recurrence matrix audit passed every
frozen grid point and selected H0=2^-24 s. Its retained evidence is TL `a2f325a`,
`crash-work/runs/qeph-bq4-native-audit-1/{raw-matrices,decision}.json` and checkpoint
`qeph-bq4-native-recurrence-1`. That audit is a finite-horizon exclusion screen,
not a general CFL theorem or a completed nonlinear response test.

This implementation follows the unchanged
[free-response admission contract](../../../../../planning/QEPH_FREE_RESPONSE_ADMISSION.md).
No production element, material, nodal owner, integrator or CUDA batch changes
are made. `QephCoupledFixture` now accepts optional qualification/configuration
IDs and is a shared qualification library. Its original BQ3 defaults, arithmetic,
three tests and frozen budgets stay unchanged and must be rerun.

## Boundaries

* `ResponseRuntime.cu` composes the existing Rig, NativeSequence, QephBatch and
  FENodalState. Every interval compares the complete native recurrence, actual
  assembled RHS, kick/momentum/internal-work and six source-work aggregates.
  Native state is accepted only after joint CUDA publication. No receipt follows
  a failed native, ledger, identity, small-response or observer check.
* `ResponseSamples` observes endpoint x/q/history/cache and carried midpoint
  rates. It reconstructs synchronous rates using the endpoint total RHS, native
  m/J and h/2; initial velocities remain physical rest. Reconstruction evaluates
  no element and advances no history. Physical and area-added isotropic kinetic
  partitions remain separate from native total-J kinetic energy.
* `ResponseFieldVisitor` defines one stable field ordering for the dictionary
  and observations: 22 fields/node and 153/element, therefore 241 or 438 fields.
  All 38 history values (37 continuous plus activity), 24 cache components,
  80 kinematic numbers plus planar flag, and 10 force diagnostics are retained.
  Initial completed kinematics are unavailable; no dt=0 force is invented.
* `ResponseComparison` applies the frozen response/energy limits at 257 common
  endpoint times. Directed binary64 bounds reuse the existing Q4 bound utilities
  strictly as arithmetic; they introduce no contact law. Upper differences and
  ratios, lower refinement RHS values, and nonvacuity lower bounds avoid false
  passing decisions from rounded threshold comparisons. Arithmetic overflow or
  unresolved positive underflow fails conservatively, without a new tolerance.
* `ResponseSampleValidation` binds retained fields to reconstructed rates,
  global kinetic/source-work sums, initial rest/zero-history/cache, observed
  domain bounds and the final accepted sample. It checks retained samples and
  lower bounds on all-endpoint extrema, not the omitted interval history.
* Protocol/report/read modules use existing ArtifactIO utilities. JSON parsing
  uses full precision and iterative parsing; duplicate keys and malformed,
  nonfinite, stale-phase, changed-model or changed-scale data reject. Numeric
  reports are not the accepted replay/rendering schema.

## Frozen experiment and publication

One source-scale 20 mm square or two squares sharing nodes; E=200 GPa,
rho=7890 kg/m3, t=1.648 mm, nu=.3. Native m/J, DM=DN=.015 and the centered
LAW1/ISROT0/IDRIL0/ITHK0 branch are unchanged. Configuration ID is
`0x4251344d4f444531`; qualification ID is `0x4251345245535031`.

The existing BQ3 load directions/amplitudes are multiplied by
sin²(pi*t/Tpulse), with exact zero at support endpoints and outside the pulse.
Tpulse=1024H0; horizon=4096H0. Refine by 1, 2 or 4 only: respectively
4096, 8192 or 16384 intervals. No adaptive h or threshold adjustment exists.
The first kick remains h/2 from known reference/rest and zero initial load/cache.
Every 16*refinement intervals yields the same 257 exact binary endpoint times.
Full source fields, phase labels, all work/kinetic partitions and native DT are
saved. All-endpoint small-response and energy-residual maxima are also retained.

Wext uses actual drift and h*omega_plus with applied base loads. EINT0, EINT1
and EVIS are source work, not conservative potential or necessarily positive
dissipation. The report preserves `Ksync+Wsrc-Wext`, rather than silently
renaming native work as energy. External impulses use actual kick duration and
base-position moment arms. Six completed runs plus both comparisons are needed
to qualify the named free experiment; a completed coarse run alone does not.

All owned owner/batch/sampling storage is allocated before stepping. The test
reuses existing GTest/native oracle diagnostics, which may allocate transient
host objects. This is not a claim of globally allocation-free qualification.
One host thread; no GPU limit changes, new owner capacity or missing-device skip.
Root applies the existing 240 s / 1 GiB RSS / 4 GiB device-growth guard. Device
free-memory changes are explicitly device-wide, not owned allocations. Progress
is printed every 30 s. A detected failure retains the last accepted observation;
an external timeout retains guard/stdout evidence and is not a completed report.

## Build and execution

Owning option: `TL_QEPH_ENABLE_RESPONSE=ON`, additionally requiring the existing
CUDA, batch, coupled and free-response-audit options. Root owns the parent
registration and serialized build. Host/CUDA strict arithmetic flags are
retained; report code alone links ArtifactIO/OpenSSL. Eigen remains single-thread
through the existing audit target. No external solver process is launched.

Targets and expected counts:

* `qeph_response_observer_check`: seven host functions, independent scalar
  shared-node mass/J/reconstruction, phase/pulse, failure preservation, synthetic
  refinement classification and directed nextafter boundary checks.
* `qeph_response_report_check`: four host functions, full-precision round-trip,
  malformed identity/domain/kinetic/startup reports, exact matrix admission
  bindings and create-only output preservation. Synthetic reports are explicitly
  protocol fixtures, not physical trajectories or matrix-qualification evidence.
* `qeph_response_run`: one GTest function `QephResponse.FullFrozenPulse`, selected
  by mandatory fixture/refinement CLI operands. It is deliberately absent from
  default CTest; root launches coarse first and stops on any failure.
* `qeph_response_compare`: host-only comparison of three completed numeric runs.

```text
qeph_response_run 1 1 DECISION.json RAW.json PROVENANCE.json NEW-DIRECTORY \
  --gtest_output=xml:NEW-TEST.xml
qeph_response_compare H/run.json H2/run.json H4/run.json NEW-COMPARISON.json
```

`PROVENANCE.json` must contain root-supplied `matrix_decision_sha256`. Admission
checks exact decision bytes, its raw byte/hash binding, the fixed grid/amplitudes,
both actual recurrence dictionaries and required numerical thresholds through
2H0; it does not trust `audit_passed` alone. Higher 4H0 evidence stays retained
without becoming a new requirement for selected H0. The runtime records its own
executable hash. Refinements must use identical matrix and executable hashes.
Root-supplied provenance is authority; hashing is not issuer authentication.

Output directories/files must be new. ArtifactIO writes are externally serialized,
not atomic exclusive creation. Each report is capped at 32 MiB; the 257x438
numeric forecast plus <=1 MiB input provenance is below 8 MiB/run. Three inputs
to a comparison are capped at 96 MiB total. CLI exit 0 means that process's
explicit stage passed; 2 means a retained scientific rejection/incomplete run;
1 means a protocol/IO failure. `simulation_ready` remains false throughout.

Before any production promotion, root must review actual coarse, h/2 and h/4
response evidence, all frozen gates and resource use. Contact, constraints, T3
coupling, MAT024, source attachments, vehicle duration and rendering remain
separate gates. `source-map.json` is a direct source/prerequisite map, not a
hermetic toolchain or full compiler dependency closure.
