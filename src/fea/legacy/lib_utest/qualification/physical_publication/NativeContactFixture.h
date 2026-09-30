// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "OwnerFixture.h"
#include "lib_src/elements/publication/NativeContactPublicationState.h"
#include <new>
namespace tl::fea::native_contact_publication {
// Qualification-only concrete transaction stand-in. Real physical/material
// assembly and owner commits remain in Rig; these tests qualify publication,
// not native contact force/selection or a moving-contact physical profile.
class QualificationAccess {
 public:
  static bool Attach(NativeContactPublicationState& state,FENodalState& owner,
      std::uint64_t source,ShellPhysicalScratchParticipation& issuer,bool activity=false) {return state.Attach(owner,source,issuer,activity);}
  static bool Stage(NativeContactPublicationState& state,const NodalPreparedView& view,NativeContactSelectors next) {
    return state.Stage(view,next);
  }
  static ShellPublicationReport Record(ShellPhysicalScratchParticipation& issuer,FENodalState& owner,
      const NodalTrialToken& token,const NodalAssemblyView& view) {
    return issuer.RecordSelfContactAcceptedAssembly(721,owner,token,view);
  }
  static ShellPublicationReport Seal(ShellPhysicalScratchParticipation& issuer,FENodalState& owner,
      const NodalTrialToken& token,const NodalPreparedView& view,ShellPhysicalScratchParticipationReceipt* out) {
    return issuer.SealSelfContactCandidate(721,owner,token,view,out);
  }
  static void Consume(ShellPhysicalScratchParticipation& issuer){issuer.Consume();}
  static bool Pending(const NativeContactPublicationState& state){return state.pending_;}
  static void Exhaust(NativeContactPublicationState& state){state.generation_=UINT64_MAX;}
  static void CorruptDt(NativeContactPublicationState& state){state.accepted_stamp_.fixed_dt*=2.;}
};
}
namespace physical_publication_test {
namespace native_contact_test {
using Access=fe::native_contact_publication::QualificationAccess;
struct Attempt {
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics material,common;fe::ShellPhysicalScratchParticipationReceipt native;
};
inline bool Bind(Rig& rig,fe::NativeContactPublicationState& state,bool activity=false) {
  return rig.Initialize()&&Access::Attach(state,rig.owner,SelfContactSource,rig.self_contact_participation,activity)&&
      rig.ConfigureScratch(false,true);
}
inline bool Prepare(Rig& rig,Attempt& a) {
  return rig.Begin(a.token,a.assembly)&&Good(Access::Record(rig.self_contact_participation,rig.owner,a.token,a.assembly))&&
      rig.Advance(a.token,a.assembly,a.prepared)&&rig.Evaluate(a.token,a.prepared,a.material)&&
      Good(rig.publication.PreparePhysical(rig.owner,a.token,
          {&a.material.qeph,&a.material.t3,&a.material.qbat,&a.material.type25,&a.material.type13,&a.material.solids},&a.common));
}
inline bool Stage(Rig& rig,fe::NativeContactPublicationState& state,Attempt& a,fe::NativeContactSelectors next) {
  return Access::Stage(state,a.prepared,next)&&
      Good(Access::Seal(rig.self_contact_participation,rig.owner,a.token,a.prepared,&a.native))&&
      Good(rig.publication.SealPhysicalScratchParticipation(rig.owner,a.token,{nullptr,&a.native}));
}
inline fe::NodalValidationReceipt Receipt(const Attempt& a,bool pass=true) {
  return {a.prepared.owner_id,a.prepared.kinematics.base_epoch,a.prepared.attempt,Qualification,pass};
}
}
} // namespace physical_publication_test
