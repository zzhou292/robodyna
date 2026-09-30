#include "ExplicitTranslationStep.h"
#include "FENodalStateStorage.h"

namespace tl::fea {
namespace {
using nodal_detail::Phase;
// One serialized work item for the admitted <=64-node gate. Every node is
// updated exactly once. Parallel scatter/reductions remain a separate gate.
__global__ void Advance(nodal_detail::Control* control, const double* accepted,
                        double* trial, const double* force, const double* inverse,
                        const std::uint8_t* fixed, std::uint32_t n, double h,
                        std::uint64_t epoch, std::uint64_t attempt) {
  if (control->status != NodalStatus::Ok) return;
  if (control->rows.base_epoch != epoch || control->rows.attempt != attempt ||
      !stability::IsCurrentLimit(control->rows, control->limit) || control->limit.dt < h) {
    control->status = NodalStatus::StaleTrial; return;
  }
  for (std::uint32_t i = 0; i < n; ++i) {
    if (!nodal_detail::AdvanceTranslationNode(accepted, trial, force, inverse[i], fixed[i] ? 7 : 0, i, n, h)) {
      control->status = NodalStatus::InvalidOutput; control->node = i; return;
    }
  }
}
}  // namespace

NodalReport AdvanceTranslations(FENodalState& owner, const NodalTrialToken& token) {
  if (!owner.impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& s = *owner.impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!s.Matches(token.owner_id_, token.base_epoch_, token.attempt_))
    return s.Reject(NodalStatus::StaleTrial, "Trial token belongs to another owner or attempt");
  if (s.phase != Phase::Sealed) return s.Reject(NodalStatus::WrongPhase, "Assembly has not been sealed");
  if(s.rigid_groups)
    return s.Reject(NodalStatus::MissingStepAdmission,"Attached rigid groups require their dedicated advance operation");
  if (s.config.temporal_scheme != NodalTemporalScheme::VelocityFirst)
    return s.Reject(NodalStatus::UnsupportedTemporalScheme, "Legacy translation stepping requires collocated velocity-first state");
  if (s.has_rotations)
    return s.Reject(NodalStatus::UnsupportedRotation, "Extended nodal state requires separately admitted AdvanceNodal");
  Advance<<<1,1,0,s.stream>>>(s.control, s.accepted, s.trial, s.scratch, s.inverse, s.fixed,
      static_cast<std::uint32_t>(s.config.node_count), s.config.fixed_dt, s.stamp.epoch, s.attempt);
  auto report = s.Check(cudaGetLastError()); if (report.status != NodalStatus::Ok) return report;
  report = s.SynchronizeControl(); if (report.status != NodalStatus::Ok) return report;
  s.phase = Phase::Ready; return {NodalStatus::Ok, "OK"};
}
}  // namespace tl::fea
