# Complete physical active prefix

`PhysicalActivePrefix` is a read-only predicate over the actual common physical
publication and all its structural participants. The source handle, owner,
participant objects and candidate token/view/diagnostics must match exactly.
The initial state must be epoch zero and every parent active. QEPH and T3 use
their existing complete validated activity readbacks, including TAB1 and native
one-point state; QBAT and mapped TYPE13/25 use authenticated complete active
counts. Qualified solids, beam18 and TYPE45 have no element-off path; their
complete candidates must still pass the common publisher.

The predicate neither commits nor issues a contact receipt. Its caller must
discard the whole attempt after rejection. This module alone does not admit
failure-bearing TYPE25 contact, prove constant K/gaps or authorize CIN releases.
Those are distinct source and runtime integration obligations.

Tests reuse the actual heterogeneous owner fixture: exact budget/alias/source
checks, genuine all-active candidate, forged/stale candidate rejection, common
discard/commit, and a real material-removal transition driven by nodal force.
The removal test repeats without altering any accepted history.

No new allocation occurs during checks. Complete family readback reuses existing
participant staging and may require transfers; this is not a GPU speed claim.
