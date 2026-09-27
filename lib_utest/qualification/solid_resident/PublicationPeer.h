// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solids/resident/Storage.h"
#include "lib_src/solvers/ExplicitNodalStep.h"

namespace tl::fea::solids {
// Test-only closed contributor coordinator: genuine initial proof, all
// preflights, one real owner commit, then the infallible resident slab swap.
class BatchQualificationPeer {
 public:
  static const ShellBatchPublication* Scope(unsigned n=0) {
    static unsigned char tags[2]{};
    return reinterpret_cast<const ShellBatchPublication*>(&tags[n]);
  }
  static BatchReport PreflightAttach(Batch& batch,FENodalState& owner,
      const NodalCoefficientLedger& ledger,const NodalRigidAssemblyBinding& rigid,
      const NodalCinWitnessSource& cin,const Model& model,const BatchConfig& config,
      const ShellBatchPublication* claimant=Scope()) {
    return batch.PreflightAttach(owner,ledger,rigid,cin,model,config,claimant);
  }
  static void Attach(Batch& batch,const ShellBatchPublication* claimant=Scope()) {
    batch.AttachPublication(claimant);
  }
  static void Release(Batch& batch,const ShellBatchPublication* claimant=Scope()) {
    batch.ReleasePublication(claimant);
  }
  static BatchReport Preflight(Batch& batch,FENodalState& owner,const NodalTrialToken& token,
      const NodalPreparedView& view,const BatchDiagnostics& expected,
      const ShellBatchPublication* claimant=Scope()) {
    return batch.PreflightPublication(owner,token,view,expected,claimant);
  }
  static BatchReport Commit(Batch& batch,FENodalState& owner,const NodalTrialToken& token,
      const NodalPreparedView& view,const BatchDiagnostics& expected,bool other_ready=true) {
    auto result=Preflight(batch,owner,token,view,expected);
    if (result && !other_ready) result={BatchStatus::ElementFailure,"Prescribed later contributor rejected"};
    if (result) {
      auto checked=CompleteNodalValidation(owner,token,
          {view.owner_id,expected.base_epoch,view.attempt,expected.qualification_id,true});
      if (checked.status==NodalStatus::Ok) checked=owner.Commit(token);
      if (checked.status!=NodalStatus::Ok) result={BatchStatus::NodalFailure,checked.message};
    }
    if (!result) {
      owner.Discard();
      batch.DiscardTrial();
      return result;
    }
    batch.Publish(owner.accepted());
    return {};
  }
  static double* PreparedLastCacheField(Batch& batch) {
    // Deliberate fault injection into qualification-owned device storage only.
    auto* state=batch.impl_->device_header.solid6z.slab[batch.impl_->TrialSlab()];
    return &state[0].cache.stabilization.modal_force_n[2][3];
  }
  static double* PreparedLastLaw90CacheField(Batch& batch) {
    auto* state=batch.impl_->device_header.solid18_law90.slab[batch.impl_->TrialSlab()];
    const auto count=batch.impl_->model.solid18_law90().size();
    return count ? &state[count-1].cache.rhs_force_n[7].z : nullptr;
  }
  static double* LastMaterialDensity(Batch& batch) {
    const auto index=batch.impl_->model.solid6z()[0].material_index;
    return &batch.impl_->device_header.material42[index].density_kg_m3;
  }
  // Qualification-only prescribed packet launch; never grants publication authority.
  static batch_detail::Storage* DeviceForPacketProbe(Batch& batch){return batch.impl_->device;}
  static const batch_detail::Storage& HeaderForPacketProbe(Batch& batch){return batch.impl_->device_header;}
  static BatchReport ReadConstructed(Batch& batch,ResultBuffers output,BatchDiagnostics& diagnostics) {
    // Qualified constructor values only: this does not attach an owner or label
    // an unclaimed source fixture as an accepted physical run.
    auto& state=*batch.impl_;
    if (state.bound || !state.OutputBuffers(output,&diagnostics,sizeof(diagnostics),&batch,sizeof(batch)))
      return {BatchStatus::InvalidInput,"Constructor qualification requires unclaimed storage"};
    const auto report=state.ReadResults(0,state.accepted_diagnostics);
    if (!report) return report;
    state.PublishResults(output);
    diagnostics=state.accepted_diagnostics;
    return {};
  }
};
} // namespace tl::fea::solids
