// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "State.h"
#include "../../solid18/Solid18ForceScratch.h"

namespace tl::fea::solids::batch_detail {
// Shared by arena admission and kernel launch. Each live worker reuses one
// private workspace for its grid-stride sequence; no parent shares it in flight.
inline constexpr unsigned candidate_blocks = 64;
inline constexpr unsigned candidate_threads = 64;
inline constexpr std::size_t candidate_workers = candidate_blocks * candidate_threads;
inline constexpr std::size_t Scratch18Count(std::size_t parents) noexcept {
  return parents < candidate_workers ? parents : candidate_workers;
}
struct Scratch18 {
  solid18::detail::ForceScratch force;
  Traits18::Cache cache;
};
TL_SOLID18_HD inline int CaptureState18(Scratch18& scratch,
    State<Traits18>& output) noexcept {
  if (!Traits18::Capture(scratch.force.trial,scratch.cache)) return -1;
  output.history = scratch.force.trial.proposed_history;
  output.cache = scratch.cache;
  return 0;
}
TL_SOLID18_HD inline int InitializeState18(const Parent18& parent,
    const solid18::Material& material, solid18::Vec3 velocity,
    Scratch18& scratch, State<Traits18>& output) noexcept {
  const auto status = solid18::detail::InitializeForceScratch(parent.reference,
      material,velocity,scratch.force);
  if (status != solid18::Status::Success) return int(status);
  return CaptureState18(scratch,output);
}
TL_SOLID18_HD inline int UpdateState18(const Parent18& parent,
    const solid18::Material& material, const State<Traits18>& accepted,
    const solid18::PrescribedInterval& interval, Scratch18& scratch,
    State<Traits18>& output) noexcept {
  const auto status = solid18::detail::EvaluateForceScratch(parent.reference,
      accepted.history,interval,material,scratch.force);
  if (status != solid18::Status::Success) return int(status);
  return CaptureState18(scratch,output);
}
} // namespace tl::fea::solids::batch_detail
