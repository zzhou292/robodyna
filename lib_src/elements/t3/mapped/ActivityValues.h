// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Result.h"
#include "../T3OnePointHistory.h"
#include "../../ShellLayeredSectionValues.h"
#include "../../one_point/ShellOnePointValues.h"
#include "../../failure/ShellFailureValues.h"
#include <cstdint>

namespace tl::fea::t3::mapped {
enum class ActivityPhase : std::uint32_t { OnePoint, Mixed, Failure, Force, PointIdentity };
enum class ActivityError : std::uint32_t {
  None, PointState, ElasticSection, PlasticSection, PointUnavailable,
  UnsupportedLaw, ReservedFailure, FailureState, ForceResult, PointIdentity
};
inline constexpr std::uint32_t ActivityKeyStride = 16;
inline constexpr std::uint32_t NoActivityFailure = UINT32_MAX;
TL_T3_HD inline std::uint32_t ActivityKey(std::uint32_t parent, ActivityError error) noexcept {
  return parent * ActivityKeyStride + static_cast<std::uint32_t>(error);
}

// These leaves perform only the existing readback predicates. Their caller
// owns complete source admission and the original global phase order.
TL_T3_HD inline ActivityError CheckPointActivity(ShellSectionLaw law,
    const ShellBatchOnePointSectionState& point, const sections::PointParameters& parameters,
    double time) noexcept {
  if (law != ShellSectionLaw::Law44Nip1) return ActivityError::None;
  return shell_batch_plasticity_detail::ValidOnePointState(point, parameters, time)
      ? ActivityError::None : ActivityError::PointState;
}
TL_T3_HD inline ActivityError CheckMixedActivity(ShellSectionLaw law,
    const ShellBatchSectionState& plastic, const sections::ShellLayeredLaw1History& elastic,
    bool has_one_point, bool execution) noexcept {
  using shell_batch_plasticity_detail::FiniteSection;
  if (law == ShellSectionLaw::LayeredLaw1Nip3)
    return FiniteSection(elastic) ? ActivityError::None : ActivityError::ElasticSection;
  if (law == ShellSectionLaw::LayeredLaw44Nip3)
    return FiniteSection(plastic) ? ActivityError::None : ActivityError::PlasticSection;
  if (law == ShellSectionLaw::Law44Nip1)
    return has_one_point ? ActivityError::None : ActivityError::PointUnavailable;
  return execution && (law == ShellSectionLaw::RigidSkin || law == ShellSectionLaw::GlobalLaw1Npt0)
      ? ActivityError::None : ActivityError::UnsupportedLaw;
}
TL_T3_HD inline ActivityError CheckFailureActivity(ShellSectionLaw law, ShellFailurePolicy policy,
    const ShellBatchFailureState& failure, const ShellBatchSectionState& plastic,
    bool has_one_point, double time) noexcept {
  using namespace shell_batch_plasticity_detail;
  if (law == ShellSectionLaw::Law44Nip1) {
    return policy == ShellFailurePolicy::ConstantAllPoints && has_one_point &&
        ValidFailureEncoding(failure) && ValidFailureState(failure, ShellFailurePolicy::None, nullptr, time)
        ? ActivityError::None : ActivityError::ReservedFailure;
  }
  const auto* section = law == ShellSectionLaw::LayeredLaw44Nip3 ? &plastic : nullptr;
  return failure.policy() == policy && ValidFailureEncoding(failure) &&
      ValidFailureState(failure, policy, section, time)
      ? ActivityError::None : ActivityError::FailureState;
}
TL_T3_HD inline ActivityError CheckForceActivity(const ReferenceData& reference,
    const ForceTrial& force, ShellSectionLaw law, double time, std::uint64_t epoch) noexcept {
  return ValidResult(reference, force, time, epoch, law == ShellSectionLaw::RigidSkin)
      ? ActivityError::None : ActivityError::ForceResult;
}
TL_T3_HD inline ActivityError CheckPointIdentity(const ReferenceData& reference,
    const ForceTrial& force, const ShellBatchOnePointSectionState& point,
    const sections::PointParameters& parameters, ShellSectionLaw law,
    double time, std::uint64_t epoch) noexcept {
  if (law != ShellSectionLaw::Law44Nip1) return ActivityError::None;
  const auto& history = force.proposed_history;
  return history.matches_reference(reference) && history.stamp().time == time &&
      history.stamp().sample_index == epoch &&
      detail::SameHistoryBits(history.data().thickness, point.point.reported_thickness_m) &&
      one_point_detail::ValidValues({history.data(), point.point.saved, point.point.failure.history,
          point.cumulative_plastic_work_J}, parameters, time)
      ? ActivityError::None : ActivityError::PointIdentity;
}
} // namespace tl::fea::t3::mapped
