// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Scratch18.h"

namespace tl::fea::solids::batch_detail {
template<class Traits> struct ExtendedScratch {
  typename Traits::ForceScratch force;
  typename Traits::Cache cache;
};
template<class Traits>
TL_BRICK_HD inline int CaptureExtendedState(ExtendedScratch<Traits>& scratch,
    State<Traits>& output) noexcept {
  const auto& trial = Traits::TrialValue(scratch.force);
  if (!Traits::Capture(trial, scratch.cache)) return -1;
  output.history = trial.proposed_history;
  output.cache = scratch.cache;
  return 0;
}
template<class Traits>
TL_BRICK_HD inline int InitializeExtendedState(const typename Traits::Parent& parent,
    const typename Traits::Material& material, solid18::Vec3 velocity,
    ExtendedScratch<Traits>& scratch, State<Traits>& output) noexcept {
  const auto status = Traits::InitializeScratch(parent, material, velocity, scratch.force);
  return status == Traits::success ? CaptureExtendedState(scratch, output) : int(status);
}
template<class Traits>
TL_BRICK_HD inline int UpdateExtendedState(const typename Traits::Parent& parent,
    const typename Traits::Material& material, const State<Traits>& accepted,
    const typename Traits::Interval& interval, ExtendedScratch<Traits>& scratch,
    State<Traits>& output) noexcept {
  const auto status = Traits::EvaluateScratch(parent, material, accepted.history, interval, scratch.force);
  return status == Traits::success ? CaptureExtendedState(scratch, output) : int(status);
}
} // namespace tl::fea::solids::batch_detail
