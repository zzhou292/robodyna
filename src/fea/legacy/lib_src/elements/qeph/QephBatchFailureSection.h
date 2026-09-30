#pragma once
#include "QephBatchLayeredSection.h"
#include "QephLayeredJ2Failure.h"
#include "QephLayeredTab1.h"
#include "../failure/ShellFailureArenaLayout.h"
#include "../failure/ShellFailureSectionAdvance.h"

namespace tl::fea::qeph::batch_detail {
template<class Section> struct FailureTrialView {
  ForceTrial& force;
  const Section& section;
};
TL_QEPH_HD inline Status EvaluateFailureSectionIntoTrial(const ReferenceData& reference,
    const History& old_shell, const PrescribedInterval& interval,
    shell_batch_plasticity_detail::MixedDeviceStorage& mixed,
    shell_batch_plasticity_detail::FailureDeviceStorage& failure,
    unsigned slab, std::size_t parent, ForceTrial& output) noexcept {
  using namespace shell_batch_plasticity_detail;
  if (slab > 1) return Status::kInvalidInput;
  const auto policy = failure.policy[parent];
  const auto& old = mixed.plastic.section[slab][parent];
  const auto& sidecar = failure.state[slab][parent];
  if (!ValidFailureState(sidecar, policy, &old, interval.base_time) ||
      old_shell.data().active != (sidecar.active ? 1 : 0)) {
    return Status::kInvalidInput;
  }
  if (policy == ShellFailurePolicy::None) {
    const auto status = EvaluateMixedSectionIntoTrial(reference, old_shell, interval, mixed, slab, parent, output);
    if (status == Status::kSuccess) failure.state[1u - slab][parent] = sidecar;
    return status;
  }
  if (mixed.law[parent] != ShellSectionLaw::LayeredLaw44Nip3) return Status::kInvalidInput;
  if (policy == ShellFailurePolicy::ConstantAllPoints) {
    if (reference.input.placement != ShellReferencePlacement::Centered) return Status::kInvalidInput;
    const auto accepted_section = FailureHistory(old, sidecar);
    sections::ShellLayeredJ2FailureResult section;
    const auto status = detail::EvaluateLayeredForceIntoTrial(reference, mixed.plastic.parameters[parent],
        old_shell, interval, output, section,
        sections::LayeredJ2FailureForceAdapter{mixed.plastic.parameters[parent], accepted_section,
            failure.parameters[parent]});
    const FailureTrialView<sections::ShellLayeredJ2FailureResult> next{output, section};
    if (status != Status::kSuccess) return status;
    return PublishFailureSection(old, old_shell.data().thickness, next, mixed, failure,
        1u - slab, parent, output) ? Status::kSuccess : Status::kNonfiniteResult;
  }
  if (policy == ShellFailurePolicy::Tab1AnyPoint) {
    const auto accepted_section = Tab1FailureHistory(old, sidecar);
    sections::ShellLayeredTab1Result section;
    const auto status = detail::EvaluateLayeredForceIntoTrial(reference, mixed.plastic.parameters[parent],
        old_shell, interval, output, section,
        sections::LayeredTab1ForceAdapter{mixed.plastic.parameters[parent], accepted_section,
            failure.tab1_parameters[parent]});
    const FailureTrialView<sections::ShellLayeredTab1Result> next{output, section};
    if (status != Status::kSuccess) return status;
    return PublishFailureSection(old, old_shell.data().thickness, next, mixed, failure,
        1u - slab, parent, output) ? Status::kSuccess : Status::kNonfiniteResult;
  }
  return Status::kInvalidInput;
}
// Preserve the existing source/value dispatcher publication contract. Only the
// resident candidate kernel selects the private partial-trial worker above.
TL_QEPH_HD inline Status EvaluateFailureSection(const ReferenceData& reference,
    const History& old_shell,const PrescribedInterval& interval,
    shell_batch_plasticity_detail::MixedDeviceStorage& mixed,
    shell_batch_plasticity_detail::FailureDeviceStorage& failure,
    unsigned slab,std::size_t parent,ForceTrial& output) noexcept {
  ForceTrial staged;
  const auto status=EvaluateFailureSectionIntoTrial(reference,old_shell,interval,mixed,failure,slab,parent,staged);
  if(status==Status::kSuccess) output=staged;
  return status;
}
} // namespace tl::fea::qeph::batch_detail
