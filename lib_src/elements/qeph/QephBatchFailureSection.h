#pragma once
#include "QephBatchLayeredSection.h"
#include "QephLayeredJ2Failure.h"
#include "QephLayeredTab1.h"
#include "../failure/ShellFailureArenaLayout.h"
#include "../failure/ShellFailureSectionAdvance.h"

namespace tl::fea::qeph::batch_detail {
TL_QEPH_HD inline Status EvaluateFailureSection(const ReferenceData& reference,
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
    const auto status = EvaluateMixedSection(reference, old_shell, interval, mixed, slab, parent, output);
    if (status == Status::kSuccess) failure.state[1u - slab][parent] = sidecar;
    return status;
  }
  if (mixed.law[parent] != ShellSectionLaw::LayeredLaw44Nip3) return Status::kInvalidInput;
  if (policy == ShellFailurePolicy::ConstantAllPoints) {
    const LayeredJ2FailureHistory base{old_shell, FailureHistory(old, sidecar)};
    LayeredJ2FailureForceTrial next;
    const auto status = EvaluateLayeredJ2FailureForce(reference, mixed.plastic.parameters[parent],
        failure.parameters[parent], base, interval, next);
    if (status != Status::kSuccess) return status;
    return PublishFailureSection(old, old_shell.data().thickness, next, mixed, failure,
        1u - slab, parent, output) ? Status::kSuccess : Status::kNonfiniteResult;
  }
  if (policy == ShellFailurePolicy::Tab1AnyPoint) {
    const LayeredTab1History base{old_shell, Tab1FailureHistory(old, sidecar)};
    LayeredTab1ForceTrial next;
    const auto status = EvaluateLayeredTab1Force(reference, mixed.plastic.parameters[parent],
        failure.tab1_parameters[parent], base, interval, next);
    if (status != Status::kSuccess) return status;
    return PublishFailureSection(old, old_shell.data().thickness, next, mixed, failure,
        1u - slab, parent, output) ? Status::kSuccess : Status::kNonfiniteResult;
  }
  return Status::kInvalidInput;
}
} // namespace tl::fea::qeph::batch_detail
