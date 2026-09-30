// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
namespace glass_removal_test {
namespace {
__global__ void DeclaredLoad(double* force_z,double* couple_y,double bending_couple) {
  const unsigned i=threadIdx.x;
  if(i<3)force_z[8+i]-=1.;
  if(i<4)couple_y[4+i]+=(i==1||i==2?1.:-1.)*bending_couple;
}
}
void Rig::Begin(Attempt& a,bool bending) {
  Check(owner.BeginTrial(&a.token,&a.assembly));
  Check(qeph.AssembleMappedAccepted(owner,a.token,a.assembly));
  Check(triangle.AssembleMappedAccepted(owner,a.token,a.assembly));
  // A prescribed physical couple bends the virgin source-authenticated glass.
  // No constitutive state, activity mask or prepared kinematic is overwritten.
  const double couple=bending?base.inertia[4]*.004/(Dt*Dt):0.;
  DeclaredLoad<<<1,32,0,a.assembly.stream>>>(a.assembly.forces.force_z,a.assembly.forces.couple_y,couple);
  Check(cudaGetLastError());
}
void Rig::Assemble(Attempt& a) {
  const auto before=Force(a);
  Check(self.AssembleAccepted(owner,a.token,a.assembly));
  const auto after_self=Force(a);
  Check(wall.AssembleAccepted(owner,a.token,a.assembly));
  const auto after_wall=Force(a);
  self_response_nonzero=wall_response_nonzero=false;
  // Read actual endpoint force/couple additions, excluding STI-only changes.
  for(unsigned i=0;i<6*11;++i){
    self_response_nonzero=self_response_nonzero||after_self[i]!=before[i];
    wall_response_nonzero=wall_response_nonzero||after_wall[i]!=after_self[i];
  }
}
void Rig::Prepare(Attempt& a) {
  Check(owner.SealAssembly(a.token));
  Check(fe::AdvanceStaggeredCin(owner,a.token,{a.assembly.owner_id,a.assembly.accepted.base_epoch,a.assembly.attempt,
      base.Qualification,Dt,.2,true,{fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true}}));
  Check(owner.BorrowPrepared(a.token,&a.prepared));
  Check(qeph.EvaluateCandidate(owner,a.token,a.prepared,&a.materials.qeph));
  Check(triangle.EvaluateCandidate(owner,a.token,a.prepared,&a.materials.t3));
  Check(publication.PreparePhysical(owner,a.token,{&a.materials.qeph,&a.materials.t3},&a.common));
}
void Rig::Seal(Attempt& a) {
  n::Transaction* members[]{&self,&wall};
  const auto sealed=n::Transaction::SealCandidateGroup(members,2,publication,owner,a.token,a.prepared,
      a.common,a.contacts.data(),a.contacts.size());
  Check(sealed.report);
  const fe::ShellPhysicalScratchParticipationReceipt* receipts[]{&a.contacts[0],&a.contacts[1]};
  Check(publication.SealPhysicalScratchParticipation(owner,a.token,{nullptr,nullptr,{receipts,2}}));
}
fe::ShellPublicationReport Rig::Commit(Attempt& a,bool approved) {
  return publication.CommitPhysical(owner,a.token,a.common,{a.prepared.owner_id,a.prepared.kinematics.base_epoch,
      a.prepared.attempt,base.Qualification,approved});
}
void Rig::Discard() {
  self.DiscardTrial();wall.DiscardTrial();publication.DiscardTrial();
  qeph.DiscardTrial();triangle.DiscardTrial();owner.Discard();
}
void Rig::Step(bool bending) {Attempt a;Begin(a,bending);Assemble(a);Prepare(a);Seal(a);Check(Commit(a));}
} // namespace glass_removal_test
