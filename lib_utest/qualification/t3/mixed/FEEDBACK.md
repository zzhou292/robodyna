# Joined mixed feedback: contract and passing result

This extension admits matching `CoupledForces` usage in the existing immutable
one-Q4/one-T3 union. It adds no owner, force law, history, timestep selector,
allocation or native dependency to production. The same owner/source/token
checks, global kinetic measurement, one owner commit and two infallible typed
slab swaps remain in force. Both caches must have participated in the SAME
attempt. Mixed prescribed/coupled usage rejects before coordinator allocation.
Reference-rest startup and all existing capacities remain unchanged. Uniform
incoming velocity, contact feedback, long response and source-part capacities
are later gates.

The new three-function CUDA test uses the retained `shell_binding_test::Edge`:
Q4 nodes0,1,2,3 and scalene T3 nodes1,4,2; positions (0,0,0), (1,0,0),
(1,1,0), (0,1,0), (1.75,.25,0), in metres. Both use density1024 kg/m3,
thickness1/32 m, E2e6 Pa and nu0.3. Native angle-weighted T3 mass and native
TOTAL J remain separate from their physical/added partitions. No contact area
or mass renormalization enters these tests.

H=1/1024 s and the existing `Schedule(r,0)` finite pulse are reused exactly.
The pulse produces the retained velocity/spin pattern during the first H/2
kick, followed by three H kicks with ZERO external loading. Thus the next
nodal increments come from nonzero Q4 AND T3 accepted force/couple caches.
This four-interval execution is a feedback/publication gate, not a claim that
H is stable for arbitrary mixed mechanics. Qualification/configuration IDs are
0x4d42314645454431 / 0x4d4231464d4f4431, distinct from prescribed MB1.

The preparation helper reuses actual owner BeginLoad, ordered Q4/T3 assembly,
SealAssembly, AdvanceStaggeredHistory and BorrowPrepared. An independent scalar
negative-cache reduction is compared exactly with all six device RHS arrays.
The actual total RHS is retained for independent long-double kick/drift work,
linear/angular velocity and position checks. Global kinetic values use the
retained world-area/atan2 mass oracle once per physical node; each family's
source increments and signed cache work are checked separately. Q4 EVIS is
retained only in the Q4 family. The owner tolerance remains
2e-13*(1+abs(truth)); work/kinetic tolerance remains
256*DBL_EPSILON*sum_abs_terms + 1e-12*1e-6 J. Native force/history comparison
uses the unchanged QEPH/T3 point helpers and their existing dimensional bounds.

Nonvacuity requires each family to exceed the retained 1e-5 N force and
1e-12 J membrane-work floors. At the first force-feedback kick, independently
omitting Q4 at its unique node0 or T3 at its unique node4 must change velocity
by more than32 times the owner arithmetic tolerance. No prescribed target is
imposed after the pulse.

The three functions cover:

1. Four accepted pulse/free intervals, alternating candidate evaluation order,
   both nonzero native caches, independent assembly/kick/source/kinetic checks
   and unchanged actual device allocations.
2. Both mismatched usage combinations; either missing cache; either tampered
   participation flag; unchanged accepted owner/typed/common output and clean
   same-base retry after nonzero history.
3. Late invalid geometry in either family after the other succeeds; complete
   accepted-state preservation and exact retry against a clean owner, with
   native force/history comparison before final publication.

The three new functions pass and execute 10 QEPH and 10 T3 native reference intervals in
total (4+2+4 per family). Native references are test-only; production APIs never
call them. Existing eight prescribed mixed functions remain regression gates;
their numerical fixtures/helpers are untouched. One old invalid-usage assertion
now supplies `Unspecified`, because matching `CoupledForces` is the feature.

The two new CUDA files are registered in the existing `mixed_shell_batch_check`;
no new runner, target or native production dependency was introduced. All three
new functions and 42 affected existing functions pass in seven test targets.
The new pulse/free test reaches maximum Q4/T3 force 0.256172/0.252560 N and
membrane work 2.12444e-6/1.91799e-6 J. Omitting either family's unique-node
contribution changes velocity by 6.99626e-6/1.15886e-5 m/s, well above the
unchanged arithmetic allowance. All allocations remain unchanged.

Evidence is `crash-work/reports/mixed-shell-feedback-{build,tests}-1.*` and
`mixed-shell-feedback-qeph-{build,tests}-1.*`, including GTest XML directories.
The two build batches take 16.827/35.203 s and peak at 1,059,913,728 B sampled
RSS. Test batches take 0.719/1.259 s and peak at 165,191,680 B. The workstation
guard permits two compiler workers on four affinity CPUs with a 12 GiB RSS
ceiling, then one CPU/1 GiB for tests with GPU monitoring. All guards pass.
Reviewed local commits and these logs retain the integration evidence; another
large source snapshot is unnecessary. The new functions pass on first build
and execution without changes to tolerances.
