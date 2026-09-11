#pragma once
#include "QephBatchLayeredSection.h"
#include "QephLayeredJ2Failure.h"
#include "../failure/ShellFailureArenaLayout.h"
#include "../failure/ShellFailureValues.h"

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
  if (policy != ShellFailurePolicy::ConstantAllPoints ||
      mixed.law[parent] != ShellSectionLaw::LayeredLaw44Nip3) {
    return Status::kInvalidInput;
  }
  const LayeredJ2FailureHistory base{old_shell, FailureHistory(old, sidecar)};
  LayeredJ2FailureForceTrial next;
  const auto status = EvaluateLayeredJ2FailureForce(reference, mixed.plastic.parameters[parent],
      failure.parameters[parent], base, interval, next);
  if (status != Status::kSuccess) return status;
  ShellBatchSectionState section;
  if (!ProposedPlasticSection(old, next.section.history.saved, next.section.current.diagnostics,
                             old_shell.data().thickness, next.force.kinematics.area, section)) {
    return Status::kNonfiniteResult;
  }
  output = next.force;
  mixed.plastic.section[1u - slab][parent] = section;
  failure.state[1u - slab][parent] = FailureState(next.section.history);
  return Status::kSuccess;
}
} // namespace tl::fea::qeph::batch_detail
