# Rigid groups in the common nodal owner

`FENodalState::Initialize(..., dofs, NodalRigidGroupModel)` optionally attaches
the complete immutable source model to the existing owner. The model may be
destroyed after successful initialization. Members must have exactly the model's
reference coordinates, reciprocal structural mass and authoritative native total
scalar J. Every member is free in translation and rotation. All attached members
start with the same prescribed translation and zero spin; ordinary nodes retain
the usual owner initialization rules. The physical source-node inventory is
unchanged: no generated primary is inserted as another physical node.

`NodalRigidGroupStorage` validates this association and owns the copied source
properties, native regularization ledger and source-ordered member inventory.
One immutable CUDA arena stores compact group ranges/principal inertias/masses,
member indices/masses/J, and node membership flags. Capacity is 64 groups, 256
members per group, and at most the owner's admitted physical-node count. All
counts and the whole device payload are checked before device allocation.

The evolving center, velocity, world spin and principal axes occupy 18 doubles
per group at the tail of **each existing accepted/trial nodal slab**. There is no
group selector, separate clock, step counter, commit or mutable public device
view. `BeginTrial` copies the full accepted slab; the existing owner pointer swap
publishes physical nodes and group values together. The legacy six-allocation
layout is unchanged without groups. Attached groups add one immutable allocation
and their two state tails under the same 1 MiB total device cap. All host vectors
and diagnostic staging are sized at initialization; stepping/readback allocates
no module storage. Runtime/driver allocations remain outside that established
module accounting contract.

## One assembly and one transaction

1. `BeginTrial` exports physical-node force/couple destinations. Every producer
   adds its complete contribution on the owner stream; `SealAssembly` checks it.
2. `AdvanceStaggeredRigidGroups` requires `NodalStaggeredHistoryAdmission` with a
   named case qualification and explicit step/new-spin limit. The bounded single
   CUDA writer advances ordinary nodes with the existing shared nodal utility.
   For each group it reduces every member force plus couple and moment arm in
   exact source order, then calls the qualified native primary/member packets.
3. `CopyPreparedRigidGroups` and `BorrowPrepared` provide immutable candidate
   diagnostics. The coordinator completes its validators and calls
   `CompleteNodalValidation`; only the matching receipt admits `Commit`.
4. Any numerical, admission or validation rejection leaves the accepted slab
   and owner epoch unchanged. Partial writes to trial nodes/groups are discarded.
   Retry reuses the same first half kick when no step has yet been accepted.

All existing ordinary advance entry points reject owners with attached groups.
The group operation also rejects unattached owners, translational PSD row proofs,
missing qualification and incompatible temporal schemes. Attached assembly mass
is deliberately `kUnspecified`: current shell/contact participants require their
own constrained-mass admission work before they may compose with this path.

## Native temporal phase and TL orientation

The supported plain explicit fresh startup uses durations `(0,h/2,h)` and later
fixed steps `(h,h,h)`. The previous duration comes from the sole accepted owner
epoch, not from attempt count. Source context is documented in the workspace
`planning/NODAL_RIGID_STARTUP_PHASE.md`; native schedule fragments retain the
fresh zero durations and `DT12 = (DT1+DT2)/2`, with an authored fixed-step
selector in the qualification wrapper.

At initialization, positions and prescribed velocity are physical and collocated.
After an accepted interval, group center is at the endpoint, group v/omega at the
previous midpoint, and the stored principal frame is the **previous force-stage
frame**. Its time is `stamp.reaction_time`. The next primary packet saves body
omega in that old frame before native `ROTBMR` rotates it with the previous drift
duration. Torque uses the rotated frame; gyroscopic terms use the saved body
omega. Substituting an endpoint frame or newly kicked omega changes this scheme.

Member positions follow native second-order drift, including its finite-step
distance error. They are not projected onto exact rigid distances. Their shell
orientations separately use the existing TL world-quaternion increment with the
new member omega. These quaternions are not the native lagged principal frame
and are not claimed as native shell rotational-state parity.

## Immutable diagnostics and integration boundary

`NodalRigidGroupInfo` carries source instance, group count and member count in
the owner query, accepted stamp, assembly source identity and prepared view.
Shared identity comparisons include all three fields. Each snapshot retains its
source group/set IDs. Accepted and prepared group readbacks validate capacities,
nonoverlapping output ranges and all staged values before changing caller data.
Prepared readback also validates the current owner token and phase. Accepted
readback during a trial still returns only accepted values.

This increment qualifies owner recurrence and publication under staged loads.
It does not yet qualify a joined plastic shell/contact trajectory. Aggregate
kinetic reporting must replace grouped physical-node kinetic contributions with
the primary translation/principal-spin contribution, preserve raw nodal and
reaction-work channels, and report regularization/frame phase explicitly. The
old uncoupled kinetic/work identity is not a rigid-group energy argument.
Source hierarchy, release/sensors, imposed rigid motion, nonzero initial spin,
implicit/ALE branches, timestep adaptation and inter-group nesting remain outside
this admitted implementation.

## Qualification

The focused directory `lib_utest/qualification/nodal_rigid_group` separates
startup, rollback, sparse-capacity, native phase, and actual-owner native tests.
Configure `TL_NODAL_RIGID_OWNER_CHECKS=ON` to add the two owner targets. Tests
include a 64-step source-scheduled two-group trajectory with changing/off loads,
ordinary-node closed form, all member reactions, exact rejection/retry, late
second-group arithmetic failure, first/new-spin rejection, atomic diagnostics,
source ownership, whole-owner budget and sparse membership reaching node 2047.
The 32-step native phase fixture detects an incorrect first full kick, an extra
principal-frame update, and using newly kicked spin for that update. Native wrappers retain eight complete pinned
source files and seventeen exact fragments; they do not execute the full engine.
All numerical execution uses the workspace guard; a host syntax pass alone is
not a runtime qualification result.
