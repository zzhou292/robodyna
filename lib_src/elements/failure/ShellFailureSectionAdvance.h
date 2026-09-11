#pragma once
#include "ShellFailureValues.h"
#include "ShellFailureArenaLayout.h"
#include "../ShellMixedSectionArenaLayout.h"
#include "../ShellBatchSectionAdvance.h"

namespace tl::fea::shell_batch_plasticity_detail {
// Both native failure adapters expose the same saved/current section contract.
// Stage the diagnostic/work update before publishing any family trial fields.
template<class NativeTrial, class ForceTrial>
TL_SHELL_SECTION_HD bool PublishFailureSection(const ShellBatchSectionState& old,
    double old_thickness, const NativeTrial& next, MixedDeviceStorage& mixed,
    FailureDeviceStorage& failure, unsigned target, std::size_t parent,
    ForceTrial& output) noexcept {
  ShellBatchSectionState section;
  if (!ProposedPlasticSection(old, next.section.history.saved, next.section.current.diagnostics,
                             old_thickness, next.force.kinematics.area, section)) return false;
  const auto sidecar = FailureState(next.section.history);
  output = next.force;
  mixed.plastic.section[target][parent] = section;
  failure.state[target][parent] = sidecar;
  return true;
}
} // namespace tl::fea::shell_batch_plasticity_detail
