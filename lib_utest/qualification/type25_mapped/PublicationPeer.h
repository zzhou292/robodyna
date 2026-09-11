// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/type25/Type25BatchStorage.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
namespace tl::fea::type25 {
// Qualification only. Root's common production coordinator owns actual joined
// composition. This peer exercises its existing claim/preflight/commit seam;
// the only physical commit remains FENodalState::Commit.
class BatchQualificationPeer {
 public:
  static const ShellBatchPublication* Scope(unsigned index=0) {
    static unsigned char tags[2]{};
    return reinterpret_cast<const ShellBatchPublication*>(&tags[index]);
  }
  static BatchReport Claim(Batch& batch,FENodalState& owner,const ShellPhysicalBinding& physical,
      const BatchConfig& config,const ShellBatchPublication* claimant=Scope()) {
    const auto checked=batch.PreflightAttachMapped(owner,physical,config.configuration_id,
        config.qualification_id,config.startup,claimant);
    if (checked.status==BatchStatus::Success) batch.AttachPublication(claimant);
    return checked;
  }
  static void Release(Batch& batch,const ShellBatchPublication* claimant) {
    batch.ReleasePublication(claimant);
  }
  static BatchReport Commit(Batch& batch,FENodalState& owner,const NodalTrialToken& token,
      const NodalPreparedView& prepared,const BatchDiagnostics& expected,bool other_passed=true) {
    auto discard=[&](BatchReport report) {
      owner.Discard();
      batch.DiscardTrial();
      return report;
    };
    const auto checked=batch.PreflightPublication(owner,token,prepared,expected,Scope());
    if (checked.status!=BatchStatus::Success) return discard(checked);
    if (!other_passed) return discard({BatchStatus::ElementFailure,"Test-only final contributor rejected"});
    const auto qualified=CompleteNodalValidation(owner,token,{prepared.owner_id,expected.base_epoch,
        prepared.attempt,expected.qualification_id,true});
    if (qualified.status!=NodalStatus::Ok) return discard({BatchStatus::NodalFailure,qualified.message});
    const auto committed=owner.Commit(token);
    if (committed.status!=NodalStatus::Ok) return discard({BatchStatus::NodalFailure,committed.message});
    batch.Publish(owner.accepted());
    return {BatchStatus::Success,"Test-only sole owner and TYPE25 publication completed"};
  }
};
}
