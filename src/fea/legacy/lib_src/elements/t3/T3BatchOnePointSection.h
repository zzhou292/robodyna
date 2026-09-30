#pragma once
#include "T3OnePointForce.h"
#include "../one_point/ShellOnePointArenaLayout.h"
#include "../one_point/ShellOnePointValues.h"
#include "../ShellMixedSectionArenaLayout.h"
#include "../failure/ShellFailureArenaLayout.h"

namespace tl::fea::t3::batch_detail {
TL_T3_HD inline Status EvaluateOnePointSection(const ReferenceData& reference,
    const History& old_shell, const PrescribedInterval& interval,
    shell_batch_plasticity_detail::MixedDeviceStorage& mixed,
    shell_batch_plasticity_detail::FailureDeviceStorage& failure,
    shell_batch_plasticity_detail::OnePointDeviceStorage& point,
    unsigned slab, std::size_t parent, ForceTrial& output) noexcept {
  using namespace shell_batch_plasticity_detail;
  if (slab > 1 || mixed.law[parent] != ShellSectionLaw::Law44Nip1 ||
      failure.policy[parent] != ShellFailurePolicy::ConstantAllPoints) return Status::kInvalidInput;
  const auto& parameters = mixed.plastic.parameters[parent];
  const auto& old = point.section[slab][parent];
  if (!ValidOnePointState(old, parameters, interval.base_time) ||
      old.point.reported_thickness_m != old_shell.data().thickness) return Status::kInvalidInput;
  const OnePointHistoryValues values{old_shell.data(), old.point.saved,
      old.point.failure.history, old.cumulative_plastic_work_J};
  OnePointHistory accepted;
  auto status = PrepareOnePointLaw44History(reference, parameters, failure.parameters[parent],
      values, old_shell.stamp(), accepted);
  if (status != Status::kSuccess) return status;
  OnePointForceTrial candidate;
  status = EvaluateOnePointLaw44Force(reference, parameters, failure.parameters[parent],
      accepted, interval, candidate);
  if (status != Status::kSuccess) return status;
  const ShellBatchOnePointSectionState next{
      candidate.point, candidate.proposed_history.plastic_work_j()};
  if (!ValidOnePointState(next, parameters, interval.base_time + interval.dt)) {
    return Status::kNonfiniteResult;
  }
  ForceTrial force;
  force.proposed_history = candidate.proposed_history.shell();
  force.kinematics = candidate.kinematics;
  force.diagnostics = candidate.diagnostics;
  for (unsigned n = 0; n < 3; ++n) {
    force.internal_force[n] = candidate.internal_force[n];
    force.internal_couple[n] = candidate.internal_couple[n];
  }
  // Infallible private trial writes. The existing T3 participant publishes
  // both slabs only after the common owner commit; no NIP3 row is accessed.
  point.section[1u - slab][parent] = next;
  output = force;
  return Status::kSuccess;
}
} // namespace tl::fea::t3::batch_detail
