// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Traits18.h"
#include "Traits24.h"
#include "Traits6z.h"
#include "Traits18Law44.h"
#include "Traits18Law90.h"

namespace tl::fea::solids::batch_detail {
// Closed family traits adapt existing value APIs only. No mechanics formulas or
// independent sample counter are stored in this common transaction helper.
template<class Traits> struct State {
  typename Traits::History history;
  typename Traits::Cache cache;
};
template<class Traits>
TL_BRICK_HD inline int InitializeState(const typename Traits::Parent& parent,
    const typename Traits::Material& material, solid18::Vec3 velocity,
    State<Traits>& output) noexcept {
  typename Traits::Trial trial;
  const auto status = Traits::Initialize(parent, material, velocity, trial);
  if (status != Traits::success) return int(status);
  State<Traits> next;
  if (!Traits::Capture(trial, next.cache)) return -1;
  next.history = trial.proposed_history;
  output = next;
  return 0;
}
template<class Traits>
TL_BRICK_HD inline int UpdateState(const typename Traits::Parent& parent,
    const typename Traits::Material& material, const State<Traits>& accepted,
    const typename Traits::Interval& interval, State<Traits>& output) noexcept {
  typename Traits::Trial trial;
  const auto status = Traits::Evaluate(parent, material, accepted.history, interval, trial);
  if (status != Traits::success) return int(status);
  State<Traits> next;
  if (!Traits::Capture(trial, next.cache)) return -1;
  next.history = trial.proposed_history;
  output = next;
  return 0;
}
} // namespace tl::fea::solids::batch_detail
