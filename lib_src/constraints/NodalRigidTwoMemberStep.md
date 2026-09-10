# Native two-member rigid-group branch

`NodalRigidTwoMemberStep.h` implements the distinct OpenRadioss `RGBODV`
two-secondary-member path. The original source is pinned at
`a62b27e6baa555d222a580d6218867d0be4d70b5`; the complete
`velrot_explicit.F90` and exact acceleration/finite-rotation/cross-product
fragments are retained under the existing nodal-rigid native qualification.

`NodalRigidGroupModel` admits two physical members using their supplied native
mass and scalar rotational J. Generated-primary mass/J, full tensor, principal
inertia correction and source member order retain existing startup rules. This
does not admit a zero-mass point pair or reconstruct mass from positions. One
member is still rejected. Existing group-count and per-group maximum limits
are unchanged.

For exactly two members the owner dispatches the new member acceleration packet.
Its finite-velocity switch tests angle² > 1e-6 and transverse displacement² >
`(1e-8*source_length_to_m)*source_length_to_m`. Original Yaris millimetres therefore
use 1e-14 m². Both branches preserve native acceleration, reaction, kick and
drift ordering, including the correction after `VELROT_EXPLICIT`. There is no
exact-position projection, dummy node or additional state owner.

The two-member primary packet also preserves the native scalar proxy-J
multiply/principal-J division, world transform, then proxy-J division. The
proxy is the existing minimum effective principal inertia; it is not additional
inertia. Cancelling these operations can introduce a one-ULP spin difference
that the displacement/kick and acceleration/kick divisions amplify. Source
shaped small-inertia recurrence tests exercise this difference. The existing
greater-than-two `EvaluatePrimaryStep` API and cancelled-proxy arithmetic are
unchanged. Both paths retain the existing owner old/new increment limits as the
bounded TL domain, although the donor old-spin guard itself applies only to
greater-than-two groups.

The same accepted/trial nodal slabs own group center, carried motion and lagged
frame; optional acceleration capture retains actual prepared A/AR. Observations
admit two members through their unchanged native/aggregate decomposition and
once-only generated-primary channels. Contributor descriptor admission accepts
two members per group while retaining complete live-owner/token authentication
and the `kUnspecified` constrained mass tag; no scalar PSD proof is reopened.

The native wrapper explicitly adapts constant/module context to SI. Its exact
`VELROT_EXPLICIT` fragment receives `EM08` as a parameter with the source-scaled
threshold; this is wrapper context adaptation, not an unchanged SI donor
routine. A second wrapper entry retains original-unit `EM08=1e-8`, independently
checking both switches and unit conversion. Neither reference calls production
math. Existing native source verification checks full files and every fragment.

Four fixtures retain original groups 2200631/2200641/2200751/2200753 and their
eight ordered NIDs/coordinates, authenticated against canonical manifest SHA256
`c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8`
and position-array SHA256
`b15188d013bcae29435a583c42adb2ecf5941369fb72c2304b5e986674a164f1`.
Their positive M/J are explicitly synthetic, so these are geometry/recurrence
qualifications, not full-vehicle mass or load-path closure.

Owning standalone targets are `nodal_rigid_two_member_native_check` and, with
`TL_NODAL_RIGID_OWNER_CHECKS=ON`, `nodal_rigid_two_member_owner_check`.
Existing `nodal_rigid_observation_native_check` and
`nodal_rigid_force_stage_native_check` contain two-member value/native gates.
All old host/native and owner targets remain part of regression qualification.
No new numerical tolerances are introduced.
