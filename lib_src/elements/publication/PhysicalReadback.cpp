// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../ShellBatchPublicationImpl.h"
#include "../ShellPhysicalOutputRanges.h"
#include "../ShellExecutionOutputRanges.h"
#include "../type25/Type25BatchStorage.h"
#include "../type13/resident/Storage.h"
#include "../solids/resident/Storage.h"

namespace tl::fea {
void ShellBatchPublication::Impl::CapturePhysicalDiagnostics(ShellPhysicalDiagnostics& output,bool prepared) const noexcept {
  // Always stage the actual participant records. Equality preflight preserves
  // legacy comparison rules, but caller-supplied signed-zero substitutions
  // must never become a new accepted diagnostic value.
  if (qbatch) output.qeph = prepared ? qbatch->impl_->candidate_diagnostics : qbatch->impl_->accepted_diagnostics;
  if (tbatch) output.t3 = prepared ? tbatch->impl_->candidate_diagnostics : tbatch->impl_->accepted_diagnostics;
  if (bbatch) output.qbat = prepared ? bbatch->impl_->candidate_diagnostics : bbatch->impl_->accepted_diagnostics;
  if (connector) output.type25 = prepared ? connector->impl_->candidate_diagnostics : connector->impl_->accepted_diagnostics;
  if (physical->beams) output.type13 = prepared ? physical->beams->impl_->candidate_diagnostics :
      physical->beams->impl_->accepted_diagnostics;
  if (physical->solids) output.solids = prepared ? physical->solids->impl_->candidate_diagnostics :
      physical->solids->impl_->accepted_diagnostics;
}
bool ShellBatchPublication::Impl::PhysicalOutputDisjoint(const void* output,std::size_t bytes) const noexcept {
  using trial_identity::Disjoint;
  if (!output || reinterpret_cast<std::uintptr_t>(output) % alignof(ShellPhysicalDiagnostics) ||
      !physical || !Disjoint(output,bytes,this,sizeof(*this)) ||
      !Disjoint(output,bytes,scope,sizeof(*scope)) ||
      !Disjoint(output,bytes,physical.get(),sizeof(*physical)) ||
      !Disjoint(output,bytes,physical->owner,sizeof(*physical->owner)) ||
      !shell_physical_owner::OutputDisjoint(physical->binding,output,bytes) ||
      !shell_execution_detail::OutputDisjoint(physical->rigid,output,bytes)) return false;
  if (qbatch && (!qbatch->impl_ || !qbatch->impl_->OutputDisjoint(output,bytes) ||
      !Disjoint(output,bytes,qbatch,sizeof(*qbatch)))) return false;
  if (tbatch && (!tbatch->impl_ || !tbatch->impl_->OutputDisjoint(output,bytes) ||
      !Disjoint(output,bytes,tbatch,sizeof(*tbatch)))) return false;
  if (bbatch && (!bbatch->impl_ || !bbatch->impl_->OutputDisjoint(output,bytes) ||
      !Disjoint(output,bytes,bbatch,sizeof(*bbatch)))) return false;
  if (connector && (!connector->impl_ || !connector->impl_->OutputDisjoint(output,bytes) ||
      !Disjoint(output,bytes,connector,sizeof(*connector)))) return false;
  if (physical->beams && (!physical->beams->impl_ ||
      !physical->beams->impl_->OutputDisjoint(output,bytes) ||
      !Disjoint(output,bytes,physical->beams,sizeof(*physical->beams)))) return false;
  if (physical->solids && (!physical->solids->impl_ ||
      !physical->solids->impl_->OutputDisjoint(output,bytes) ||
      !Disjoint(output,bytes,physical->solids,sizeof(*physical->solids)))) return false;
  return true;
}
ShellPublicationReport ShellBatchPublication::CopyAcceptedPhysicalDiagnostics(const NodalStamp& expected,
    ShellPhysicalDiagnostics* output) const noexcept {
  if (!impl_ || !impl_->physical)
    return {S::NotInitialized,"Physical publication is not initialized"};
  const auto& state = *impl_;
  if (!state.PhysicalOutputDisjoint(output,sizeof(*output)) ||
      !trial_identity::Disjoint(output,sizeof(*output),&expected,sizeof(expected)))
    return {S::InvalidInput,"Physical accepted output overlaps retained state or identity"};
  if (!state.PhysicalUsable()) return {S::DeviceFailure,"Physical publication is poisoned"};
  if (!state.SamePhysicalScope(expected))
    return {S::StaleTrial,"Physical accepted publication belongs to another endpoint"};
  const auto& accepted = state.physical->accepted;
  // Compare named values against every actual cache before caller publication.
  auto actual = accepted;
  state.CapturePhysicalDiagnostics(actual,false);
  if (!SamePhysicalDiagnostics(actual,accepted))
    return {S::StaleTrial,"A participant differs from the complete accepted physical publication"};
  *output = accepted;
  return Ok();
}
} // namespace tl::fea
