# TB1/TB2 standalone resident T3 gate

Source staged before execution; no passing batch or dynamics result is claimed.
The qualified T3 startup/rates/force code in `a34a9ba` and native R1/R2/R3
remain unchanged. This stage implements the standalone part of
[the resident design](../../../../../planning/T3_RESIDENT_BATCH_DESIGN.md).
Mixed QEPH/T3 binding and publication are not implemented.

`elements/t3/T3Batch.h` owns one immutable model and two complete native T3
result/history slabs in one allocation. The existing FENodalState owns all
physical nodal state, stream, clock, token and constraints. There is no native
production runtime or new force law. The batch admits at most two T3s, sixteen
covered owner nodes and one MiB total device storage; root's [host declaration probe](../../../../../crash-work/reports/t3-batch-abi-probe-1.json)
measured Model2064, Slab1952, Control248, ForceTrial976 and Diagnostics232 bytes,
with complete Storage6216 bytes/alignment8. Production asserts this 64-bit ABI.
This is host layout evidence only: CUDA agreement and actual
`allocations().device_bytes` remain pending. The nodal owner still owns six
allocations. No per-step allocation, device-limit changes or missing-GPU skip.

The model reruns the exact startup producer, checks every derived field,
retains uint64 source node IDs and consistent shared reference positions, and
assembles native angle/pi m and total J once in listed element/native-node
order. Physical/area-added inertia are separate diagnostic sums. Neither
angles nor total-J partitions are normalized or recombined. Actual initial
owner x/v/omega, unit q, free masks and inverse m/J are validated before the
known zero cache is bound. No h=0 force evaluation occurs. Initial diagnostics
have no completed interval; untouched kinematics/native dt remain unavailable.

Cached loads use the native positive internal convention and enter the owner
RHS with sign -1. A failed later scatter can change disposable assembly scratch;
the sticky failure prevents sealing, and accepted nodal/history/cache state
stays intact. Candidate force is evaluated from accepted history and authentic
borrowed endpoint/midpoint fields. Complete result readback stages all requested
outputs. `CommitT3Trial` checks the actual token/prepared view and receipt before
one owner commit followed only by an infallible slab/metadata swap. Accepted
readback after rejection never evaluates history. CUDA error poisons the
participant; no device/context recovery is promised.

T3 history is FOR5/FOR_G5/MOM3/STRA8/THK/EINT2/EPSD/activity. There is no HOURG
or EVIS. FOR includes instantaneous DM and FOR_G is persistent; MOM is Pa and
physical moment per length is t_eff²*MOM. EINT is signed accumulated native
work in J, not potential energy. Kinetic scalar partitions include drilling.

## Frozen fixtures and numerical scope

Coordinates in metres: `(0,0,0), (1,0,0), (.25,.75,0), (1.5,1,0)`.
The one-T3 fixture uses `{0,1,2}`; two T3s use `{0,1,2}` and `{1,3,2}`,
sharing the edge `{1,2}`. Both are scalene, with areas .375 and .5625 m².
Source node IDs are `2^54+100+global_index`. E=2e6 Pa, nu=.3, rho=1024 kg/m³,
t=1/32 m. Fixed h=1/1024 s. These are explicit synthetic LAW1 transaction
fixtures, not original source MAT024 or shell accuracy experiments.

For initial coordinates x,y, the prescribed vector patterns are
`V=(.001*(x+.25*y), .0005*(y-.5*x), .00075*(x-y))` m/s and
`W=(.002*y,.003*x,.001*(x+y))` rad/s. Four intervals prescribe multipliers
`{1,0,-1,0}` with initial multiplier0. All force/couple arrays are fixed from
these initial operands: `F=m*(target-previous)*V/s`, `C=J*(target-previous)*W/s`,
where s=h/2 initially and h thereafter. They are state-independent loads;
no batch cache is consumed in committed nonzero intervals. Holds therefore
stop the prescribed velocities/spins up to kick roundoff, rather than merely
setting acceleration to zero while carrying nonzero rates.

The existing receipt-gated `AdvanceStaggeredHistory` route is used with the
new test-only identity `0x5433423250524531`. It qualifies prescribed operands
and joint publication only. `CoupledForces` usage is exercised solely at the
exact zero rest state to test participation guards. Nonzero cached loads are
assembled for native scatter comparison and then discarded; they do not drive
a TB1/TB2 trajectory. No QEPH BQ4 timestep claim is imported.

Eight actual-owner CUDA functions in two small test files cover:

1. One/two-cell startup, native independent angle m/J checks, complete zero
   cache, high IDs, missing initial binding and unchanged allocation counts.
2. Four load/hold/reverse/hold intervals for each fixture: twelve native cell
   intervals with all92 force/kinematic/history/diagnostic fields compared;
   independent continuum work and actual-current fixed-resultant power checks.
   Actual v/omega and positions are checked directly against the analytic
   prescribed targets, including both zero-rate holds. Shared-node kinetic and
   source-work totals/increments are independently
   reduced in long double. Nonzero membrane >1e-12 J, bending >1e-14 J and
   force >1e-5 N witnesses prevent a zero-response pass.
3. Retained complete native C3UPDT3 negative scatter into a seeded shared RHS,
   duplicate assembly rejection and unchanged accepted history/cache.
4. Immutable usage mismatch and missing receipt; first-half-kick reuse after
   rejection, with zero response only for the coupled usage branch.
5. Eleven malformed startup/reference/capacity cases, staged initialization
   retry and five actual inverse-mass/inertia/rest/mask binding failures.
6. Foreign owner/stream/time, stale/altered diagnostics, short/aliased outputs,
   bad receipt and fabricated foreign prepared buffers rejected at commit.
7. A second-triangle-only collapse or finite DBL_MAX velocity after a real owner
   advance, preserving nonzero accepted history/cache/output; clean retry
   matches a clean owner's complete typed fields exactly. These fault injections
   are test-only candidate corruption, not admitted shell motion.
8. Pending invalid launch before commit prevents both publications and poisons
   the batch; the accepted stamp and caller output survive. A recoverable zero-
   thread launch may report InvalidConfiguration or InvalidValue, as in T1/T2.

Native/HD comparisons retain the force port's frozen dimensional
`2e-12*(dimension+abs(native))` budget. Independent material/work checks retain
its 2e-11 field floor, 2e-10 relative coefficient and 2e-22 J work floor,
with only the already qualified stored-work subtraction allowance. New aggregate
energy checks use `256*epsilon*sum_absolute_terms + 1e-12*1e-6 J` (1e-18 J
absolute floor). The 1e-6 J experiment scale was fixed before execution, not
estimated from test results. It applies to global kinetic and EINT ledgers;
no E*t*L² aggregate floor can hide small signals. Exact identity/byte/unchanged
cache and native signed-scatter comparisons have no numerical tolerance.
Direct prescribed position/v/omega checks retain the shared actual-owner
fixture's 2e-13*(1+abs(expected)) arithmetic tolerance.

## Shared helpers and registration

New `solvers/NodalTrialIdentity.h` contains the exact existing QEPH stamp/view
predicates and host range checks. New `elements/ShellBatchFields.h` owns the
same small read/dot/difference/mean helpers, native3/4-node gather and signed
cache work arithmetic. They own no state or numerical policy. T3 calls them;
the proposed mechanical QEPH adoption is separately supplied to root at
`/tmp/t3-batch-qeph-helper-adoption.patch`. It has not been applied. QEPH's
current production/source freeze is preserved pending its response checkpoint.

New `elements/t3/T3Batch.cmake` defines `tl_t3_batch` from six owning sources,
linked only to `tl_t3` and `tl_explicit_nodal_state`. This directory's fragment
defines `t3_batch_check`, linking `t3_r3_native` only in qualification. Suggested
owning option: `TL_T3_ENABLE_BATCH`, requiring existing CUDA and force options;
reuse the existing owner registration and include both fragments. Root owns
all top-level CMake/Bazel edits and live pinned build/manifest updates. Keep
CPU no-fast-math/ffp-contract-off, CUDA fmadfalse/precise divide+sqrt/ftzfalse,
RUN_SERIAL, one CPU and120s timeout. No standalone project is added.

Before promotion root runs the eight functions, unchanged T3 force/startup/
rates/native regressions and owning nodal/scatter gates. If it adopts the shared
helper patch, rerun unchanged QEPH batch/coupled functions before any nonzero
mixed work. Freeze/retain actual source, binary, allocation and runtime evidence.

## First-execution fixture correction

The first build and all47 prior functions passed; seven of eight new CUDA
functions passed. [The failing run](../../../../../crash-work/reports/t3-batch-cuda-tests-1.json)
rejected the kind5 source-ID fixture before reaching batch binding: incrementing
its first ID duplicated the triangle's third ID. The fixture now adds1000,
keeps all three native IDs distinct, and explicitly expects shared global node1
binding to reject at element1. No production, load, tolerance or budget changed.
Later malformed-reference and actual mass/rest/mask variants were source-reviewed
for the same setup error; none require another correction. Reexecution is pending.

The original source map and failed runtime remain in the immutable
[first-execution snapshot](../../../../../crash-work/checkpoints/t3-batch-first-execution-1/manifest.json),
manifest SHA `ccdee048f2d94dd7b6c13691e91bd76cc94475e905c09647d662d8c98f792ea9`.
Its source-map SHA was
`c413aab686f431c3be2dfde4037b7679ab967327cbeb996b7bde8df5809b5afb`.
