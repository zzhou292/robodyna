#include "QephBatchStorage.h"
#include <cstring>
#include <utility>

namespace tl::fea::qeph {
namespace {
using trial_identity::Disjoint;
BatchReport InvalidRead() { return {BatchStatus::InvalidInput,"QEPH readback output is missing, overlapping or overflowing"}; }
}
BatchReport QephBatch::CopyAcceptedResults(const NodalStamp& expected,ForceTrial* output,std::size_t capacity,
                                         BatchDiagnostics* diagnostics) {
  if(!impl_) return {BatchStatus::NotInitialized,"QEPH batch is not initialized"};
  auto& s=*impl_;
  if(!s.bound) return {BatchStatus::NotBound,"Initial reference-at-rest mass binding is required"};
  if(!batch_detail::SameStamp(expected,s.accepted_stamp)) return {BatchStatus::StaleTrial,"Accepted QEPH endpoint identity mismatch"};
  if(capacity<s.config.element_count) return {BatchStatus::ResourceLimit,"QEPH result capacity is insufficient"};
  const auto bytes=s.config.element_count*sizeof(ForceTrial);
  if(!Disjoint(output,bytes,diagnostics,sizeof(*diagnostics))||!Disjoint(output,bytes,&expected,sizeof(expected))||
     !Disjoint(diagnostics,sizeof(*diagnostics),&expected,sizeof(expected))) return InvalidRead();
  auto r=s.ReadResults(s.accepted); if(r.status!=BatchStatus::Success) return r;
  std::memcpy(output,s.staging.data(),bytes); *diagnostics=s.accepted_diagnostics;
  return {BatchStatus::Success,"OK"};
}
BatchReport QephBatch::CopyPreparedResults(const BatchDiagnostics& expected,ForceTrial* output,std::size_t capacity) {
  if(!impl_) return {BatchStatus::NotInitialized,"QEPH batch is not initialized"};
  auto& s=*impl_;
  if(!s.bound) return {BatchStatus::NotBound,"Initial reference-at-rest mass binding is required"};
  if(!s.pending||!batch_detail::SameDiagnostics(expected,s.candidate_diagnostics))
    return {BatchStatus::StaleTrial,"Prepared QEPH result identity mismatch"};
  if(capacity<s.config.element_count) return {BatchStatus::ResourceLimit,"QEPH result capacity is insufficient"};
  const auto bytes=s.config.element_count*sizeof(ForceTrial);
  if(!Disjoint(output,bytes,&expected,sizeof(expected))) return InvalidRead();
  auto r=s.ReadResults(s.trial); if(r.status!=BatchStatus::Success) return r;
  std::memcpy(output,s.staging.data(),bytes); return {BatchStatus::Success,"OK"};
}

BatchReport CommitQephTrial(FENodalState& owner,const NodalTrialToken& token,QephBatch& batch,
                           const BatchDiagnostics& expected,const NodalValidationReceipt& receipt) noexcept {
  auto fail=[&](BatchReport r) { owner.Discard(); batch.DiscardTrial(); return r; };
  if(!batch.impl_) return fail({BatchStatus::NotInitialized,"QEPH batch is not initialized"});
  auto& s=*batch.impl_;
  if(s.joined_binding) return fail({BatchStatus::InvalidInput,"Joined shell participant requires two-family publication"});
  if(!s.usable) return fail({BatchStatus::DeviceFailure,"CUDA QEPH batch is poisoned"});
  if(!s.bound||!s.pending||!batch_detail::SameDiagnostics(expected,s.candidate_diagnostics)||
     !batch_detail::SameStamp(owner.accepted(),s.accepted_stamp))
    return fail({BatchStatus::StaleTrial,"QEPH publication does not match the complete accepted/trial pair"});
  if(!receipt.passed||receipt.owner_id!=expected.owner_id||receipt.base_epoch!=expected.base_epoch||
     receipt.attempt!=expected.attempt||receipt.qualification_id!=s.config.qualification_id)
    return fail({BatchStatus::StaleTrial,"QEPH candidate receipt mismatch"});
  NodalPreparedView authentic;
  auto nodal=owner.BorrowPrepared(token,&authentic);
  if(nodal.status!=NodalStatus::Ok)
    return fail({BatchStatus::NodalFailure,nodal.message,UINT32_MAX,UINT32_MAX,Status::kSuccess,nodal.status});
  if(!batch_detail::SamePrepared(authentic,s.candidate_view))
    return fail({BatchStatus::StaleTrial,"QEPH evaluated views differ from the owner's actual prepared token"});
  // Candidate evaluation already completed its stream. Do not drain a pending
  // CUDA error here: the owner must detect/poison it before epoch publication.
  nodal=CompleteNodalValidation(owner,token,receipt);
  if(nodal.status==NodalStatus::Ok) nodal=owner.Commit(token);
  if(nodal.status!=NodalStatus::Ok) {
    if(nodal.status==NodalStatus::DeviceFailure) s.usable=false;
    return fail({BatchStatus::NodalFailure,nodal.message,UINT32_MAX,UINT32_MAX,Status::kSuccess,nodal.status});
  }
  // Sole post-owner publication boundary: no allocation, CUDA, rejection,
  // callbacks, output observers or new mechanics may be added below this line.
  s.Publish(owner.accepted());
  return {BatchStatus::Success,"Owner and QEPH material/cache published"};
}
} // namespace tl::fea::qeph
