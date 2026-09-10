# Wall-response CUDA/native runtime contract

This file freezes the execution and publication boundary. The host observer and
comparison own their separate numerical contracts. This runtime is source-only
until root records the bounded test and six response runs; it makes no new
production mechanics, material or whole-vehicle admission claim.

`Execute(Run&)` follows the existing response command/GTest pattern. Root prepares
a fresh valid `Config`, its immutable matching `Model`, and exactly257 reserved
sample slots before any device allocation. The actual final selector index SHA
is carried by `config.screen_index_sha`, with the root-authenticated selected h.
The qualified current selection is H0=2^-24 s; the API also retains the frozen
H0/2 selection possibility. Root, not a CUDA parser or hash-format check,
authenticates the final index, all four boost comparisons, producer provenance
and selected step. There is no fallback, resampling or per-run step selection.

## Reuse and fixed physical values

`ScreenedWallFixture.h/.cu` and `ScreenedWallChecks.cu` mechanically extract the
retained incoming initialization, native initialization and initial/rigid checks.
The old incoming wrappers keep exactly their binding/refinement/horizon/schedule
guards and numerical operands. CW0 rest initialization, both ledgers and the
existing transaction/native force helpers are unchanged in this extraction.

The runtime consumes `Model.screened()` directly: native reference at X=-gap,
not the touching tangent-probe state; original one/two Q4 cells, actual native
mass/total-J and separately retained physical/added partitions; exact directed
penalty-law binary64 coefficient and finite wall. It checks the resulting
typed references/mass/coordinates against the observer model. Configuration and
qualification IDs are the owning `WallResponseData.h` constants, distinct from
the entry-only prefix. K0/P0/Side*P0 ledger scales are still the original ordered
native mass sums, prepared once and used unchanged. The existing .001-rad trial
rotation bound, zero-rigid checks and all native/ledger/contact tolerances stay
fixed. There is no mass recombination, surrogate source geometry or new damping.

The known uniform startup binds through the actual-owner accepted assembly
overload before K0. An actual accepted-base wall assembly/readback supplies the
initial contact records and is then discarded without a force evaluation or
owner advance. Initial native/owner/history/K0 checks and both host observation
stages finish before the epoch-zero sample becomes available.

## One interval and publication

`runtime_detail::Participants` contains the existing WallRig, test-only native
oracle sequence, accepted element values and ledger scalars. It creates no
second physical state/clock or generic callback/owner framework. Its references
to Run.config and Run.model remain valid for its local lifetime and are read
immutably. Copying Participants is prohibited.

`PrepareStep` reads the actual accepted owner/cache and checks their native
counterparts. It computes the host contact law on the independently accepted
native base **before** native `Propose`, then uses the existing WallRig
Begin/Evaluate path. The actual owner consumes accepted shell cache plus base
contact with first h/2 kick, subsequent h kicks and h drift. Candidate evaluation
order alternates by base epoch. Full native force/history and owner parity,
finite-wall contact agreement, independent assembly/kick/momentum/source-work
ledgers, and the unchanged rigid-response checks all precede observation.

The input to `ObserveEndpoint` contains the same checked candidate coordinates,
carried midpoint rates, element cache/history and actual contact readback. The
host observer assembles the complete endpoint internal+contact RHS for its
synchronous velocity/energy/impulse reconstruction. Runtime does not call the
old free-response observer or infer synchronous values from contact alone.

Cumulative positive wall impulse uses actual `kick_dt*base_wall_reaction` from
the checked device interval. `AddImpulse` encloses each existing nominal/radius
pair with the owning directed arithmetic, adds the two intervals and certifies
the new nominal binary64 sum. Its lower bound may clamp to zero because wall
reaction impulse is nonnegative. No ad hoc arithmetic tolerance replaces those
bounds. Each individual increment already passed the independent wall ledger.

`ObserveStep` stages a fixed Sample and Summary through the owning host functions
at every accepted endpoint, including unsampled endpoints. It checks the next
sample slot prospectively. An observer failure preserves its earlier staged
Sample/Summary while clearing the private publication-ready flag; accepted
output is unchanged. `PublishStep` rechecks base/sample/capacity association and
scientific assertions before calling the existing joint publication exactly
once. Afterwards it only copies fixed native/cache/contact/ledger/Sample/Summary
values and appends a trivially copyable sample into already-reserved storage.
There is no allocation, force operation, device read or fallible validation
after owner commit in that publication path.

Execute stops at the first failed stage and discards all pending contributors.
`last_accepted`, sampled endpoints and summary retain the accepted prefix for
the owning failure report. Each available `PrepareStep` call increments
`attempted_steps`, bounded by MaxSteps. Execute makes one attempt per accepted
endpoint and at most one final rejected attempt, so its count is accepted_steps
or accepted_steps+1; completion requires equality. The focused retry test counts
its deliberately repeated same-base proposal as another actual attempt.
`native_cell_intervals` counts the cells in fully successful native proposals,
including a proposal followed by a rejected device/observer candidate; a partial
failed native proposal is not counted as a complete cell set. Therefore a normal
stop-on-first-failure run stays between accepted*cells and attempted*cells.

The ten runtime ledger maxima have this fixed order: kinetic kick, linear
momentum, angular momentum, internal cache work, source work, contact work,
contact impulse, contact conservative defect, regular rigid rates, HG rigid
rates. The source and contact budget ratios must all stay at most1.

## Bounded runs and focused transaction test

Full runs are separate invocations for one/two cells and refinements1/2/4,
through exactly4096H0=0.000244140625 s. At selected H0 they contain4096/8192/16384
intervals; the six runs total86016 native cell intervals. Common output endpoints
number257 including zero, spaced16H0, with cadence16*refinement intervals. The
alternate selected H0/2 uses twice those interval counts and the same physical
output times, still within32768 intervals/run. No full response is hidden in a
default CUDA unit test.

Root's prospective per-run guard is120 s, one CPU and1 GiB RSS, with the existing
serialized workstation/GPU lock. Allocation counts/bytes are captured after
initialization and checked unchanged before every receipt. The retained prefix
measured116028/116850 device bytes in8 allocations for these one/two-cell layouts;
the full runtime must measure its own actual allocations rather than asserting
an invented new ABI. Host sample storage remains fixed-capacity before stepping.

One actual CUDA test exercises the real observer/publication boundary near
entry. It stages a valid candidate, changes the staged potential radius to-1,
and requires `ObserveEndpoint` to reject it without changing its Sample/Summary.
Publication then rejects. After restoring valid observation, a failed owner
receipt is rejected. All accepted owner/history/K0/contact/native state,
last_accepted/summary/ledger fields and every common sample remain unchanged.
A fresh retry uses the opposite participant order and reproduces candidate
fields/numerics exactly apart from attempt identity, then publishes entry and
the first actual contact kick. At selected H0 this test computes398 native cell
intervals (792 for selected H0/2), including its deliberately repeated proposal.
It is a short transaction fixture, not a serialized completed response run.

Root registers the runtime from WallResponseInitialize.cu,
WallResponseTransaction.cu and WallResponseRuntime.cu, linking the host observer,
qeph_coupled_fixture, tl_nodal_wall_contact_device and owning wall recurrence
model/schedule support plus the three existing CW0 fixture/transaction/ledger
TUs and the two new ScreenedWall TUs. The CUDA test adds
WallResponseRuntimeTest.cu and the existing WallIncomingFixture.cu solely to
reuse its strict environment binding reader. C++/CUDA17 and the retained
`-fno-fast-math -ffp-contract=off`, `--fmad=false --prec-div=true
--prec-sqrt=true --ftz=false` flags remain mandatory. Root owns all registration,
maps, builds, executions and retained evidence.
