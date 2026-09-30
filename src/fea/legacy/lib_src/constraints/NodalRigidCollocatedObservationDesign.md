# Native collocated rigid-group observation: phase contract and next input

The pure value increment is now implemented separately by
`ObserveGroupForceStageKinetic`; see `NodalRigidForceStageKinetic.md` for its
qualified scope. Owner capture and engine integration below remain future work.
Existing `ObserveGroupKinetic`/`ObserveGroupKick` retain their physical-initial or
midpoint/lagged-frame contract and do not reconstruct native collocated values.

All references below use OpenRadioss commit
`a62b27e6baa555d222a580d6218867d0be4d70b5`. The complete `rgbcor.F` is retained in
the qualification native directory with SHA-256
`ca7f8edc40fca44c6c71f81a8bb0ae14226f833c014453bbdf6753e046171ea9`
and Git blob `4a1cd6b377b66ecdf75f030cd70a91c329891759`. Complete RESOL and packet
sources are already retained there; complete `rbycor.F` and `accele.F` remain
in the workspace's pinned nodal-rigid source contract collection.

## Active plain explicit branch

For the supported 3D, free, unsensored, nonhierarchical group and IDYNA=0:

1. RESOL5654–5696 calls RBYFOR/RGBODFP. RGBODFP624–626 saves body omega in
   the previously stored frame; line628 updates that frame with DT1. Torque
   uses the new force-stage frame and gyro uses saved VI.
2. RESOL6868 calls ACCELE. `accele.F`65–83 and116–130 divide assembled force
   and primary proxy rotational force by native mass/J. These arrays now hold
   accelerations, not force vectors.
3. RESOL7750 calls RBYVIT/RGBODV. The latter predicts group omega with DT12
   and overwrites member A/AR with the constrained accelerations, including
   second-order member geometry terms; see RGBODV138–161 and224–236.
4. RESOL7981–7987 calls RBYCOR; `rbycor.F`112–129 selects classical unsensored
   groups and passes ordinary V/VR and A/AR when IDYNA=0. The earlier RESOL6650
   RBYCOR call is inside the IMPL_S==1 block beginning6584, so it is not this
   explicit observation stage.
5. Only later does RESOL8834 call VELOCITY, followed by DEPLA at8927. Therefore
   the RGBCOR inputs still contain the pre-kick midpoint V/VR while A/AR already
   represent this force-stage constrained kick. The frame is the updated
   force-stage RBY, not the prior accepted lagged frame.

The native explicit RGBCOR branch is selected by the outer ELSE at265 and
ISENS==0 at266. Lines267–271 use DT05=DT1/2 for group omega reconstruction;
IDYNA>0 at268 changes that multiplier and is outside the admitted branch.
Lines272–281 rotate this reconstructed omega into RBY and replace primary
proxy-J rotation by principal-tensor rotation. The ordinary 3D small-group
member loop at326–351 reconstructs both member V and VR from their **constrained
A/AR**, removes their kinetic contributions and removes their duplicate mass
and linear momentum. Lines453–454 publish energy correction with factor1/2.

Fresh startup has DT1=0, so the collocated observation retains prescribed
physical v0 and zero spin even when this stage has a nonzero acceleration.
On later fixed steps DT1=h, and reconstruction advances the old midpoint by
h/2 to the current force/position time. The new member omega prediction used
to compute the constrained acceleration must still use the actual DT12 from
that same step. None of these statements makes a stored midpoint energy sample
equal to the native collocated diagnostic.

## Proposed subsequent value input

Use a distinct `ForceStageRigidObservationInput`, not another flag on a stored
midpoint sample. It should contain:

- Immutable model/member metric and source association supplied by the owner.
- Current force/position time, preceding midpoint velocity time, DT1 and the
  actual DT12 used for this force-stage constrained packet.
- Group pre-kick V/VR, group translational/angular acceleration, and **updated
  force-stage principal axes** from the primary force packet.
- Member pre-kick V/VR and native constrained A/AR from the corresponding member
  packet, in the same source order. Raw nodal F/m or couple/native-J is excluded.

The pure observer may then form Vc=V+A*(DT1/2), Wc=VR+AR*(DT1/2) for the primary
and every member using those already phase-qualified values. It must not mutate
accepted frame/history, run a second physical clock, or manufacture A/AR by
independently treating coupled members as free nodes. The owner adapter must
prove that supplied force-stage axes and accelerations belong to the same
accepted state, assembly and candidate packet. If it evaluates a new force
stage for an accepted endpoint, all relevant shell/contact/material history
sampling must share that stage and remain uncommitted.

## Smallest follow-up qualification

Retain complete force-stage native packets for at least32 fresh steps, starting
with uniform v0 and zero spin, followed by changing and zero applied wrenches.
Compare group/member reconstructed Vc/Wc and native kinetic corrections at
every force stage, separately from published midpoint snapshots. Include first
DT1=0, anisotropic gyro, full couples and second-order member drift. Negative
controls must distinguish raw F/m member extrapolation, a full DT12 extrapolation,
lagged versus updated axes, and use of newly kicked V/VR. Add a late-observation
rejection proving that neither frame nor material/group accepted state changes.

The current native wrapper intentionally sets DT1=0 and A/AR=0 around exact
RGBCOR267–283,326–351,453–454 fragments. It verifies kinetic replacement algebra
for supplied values, including primary proxy cancellation. It cannot satisfy
the follow-up phase/owner gate by itself.
