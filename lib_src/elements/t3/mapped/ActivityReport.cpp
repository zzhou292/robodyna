// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ActivityQuery.h"
namespace tl::fea::t3::mapped {
BatchReport ActivityErrorReport(ActivityPhase phase, std::uint32_t key, std::size_t parents) noexcept {
  if (key == NoActivityFailure) return {BatchStatus::Success,"OK"};
  const auto parent = key / ActivityKeyStride;
  const auto error = static_cast<ActivityError>(key % ActivityKeyStride);
  if (parent < parents) {
    if (phase == ActivityPhase::OnePoint && error == ActivityError::PointState)
      return {BatchStatus::NonfiniteResult,"One-point saved/current/failure state is invalid"};
    if (phase == ActivityPhase::Mixed) {
      if (error == ActivityError::ElasticSection) return {BatchStatus::NonfiniteResult,"Nonfinite elastic section history"};
      if (error == ActivityError::PlasticSection) return {BatchStatus::NonfiniteResult,"Nonfinite plastic section history"};
      if (error == ActivityError::PointUnavailable) return {BatchStatus::InvalidInput,"One-point history is unavailable"};
      if (error == ActivityError::UnsupportedLaw) return {BatchStatus::InvalidInput,"Unsupported section readback law"};
    }
    if (phase == ActivityPhase::Failure) {
      if (error == ActivityError::ReservedFailure)
        return {BatchStatus::NonfiniteResult,"One-point row acquired a three-point failure state"};
      if (error == ActivityError::FailureState)
        return {BatchStatus::NonfiniteResult,"Failure sidecar state disagrees with its declared policy/saved section"};
    }
    if (phase == ActivityPhase::Force && error == ActivityError::ForceResult)
      return {BatchStatus::NonfiniteResult,"Mapped T3 force cache differs from its source/endpoint role",parent};
    if (phase == ActivityPhase::PointIdentity && error == ActivityError::PointIdentity)
      return {BatchStatus::NonfiniteResult,"One-point state differs from its actual shell slab/history identity",parent};
  }
  return {BatchStatus::NonfiniteResult,"T3 compact activity packet encoding is invalid"};
}
} // namespace tl::fea::t3::mapped
