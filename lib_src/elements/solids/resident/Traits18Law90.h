// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"
#include "../Model.h"
#include "../../solid18/total_strain/Force.h"

namespace tl::fea::solids::batch_detail {
struct Traits18Law90 {
  using Parent = Parent18Law90;
  using Material = solid18::total_strain::Material;
  using History = solid18::total_strain::History;
  using Interval = solid18::PrescribedInterval;
  using Trial = solid18::total_strain::ForceTrial;
  using Cache = Cache18Law90;
  using Result = Result18Law90;
  using Status = solid18::Status;
  using ForceScratch = solid18::total_strain::ForceScratch;
  static constexpr Family family = Family::Solid18Law90;
  static constexpr unsigned nodes = 8;
  static constexpr Status success = Status::Success;
  TL_SOLID18_HD static Status InitializeScratch(const Parent& parent, const Material& material,
      solid18::Vec3 velocity, ForceScratch& scratch) noexcept {
    return solid18::total_strain::InitializeForce90Scratch(parent.reference, material, velocity, scratch);
  }
  TL_SOLID18_HD static Status EvaluateScratch(const Parent& parent, const Material& material,
      const History& history, const Interval& interval, ForceScratch& scratch) noexcept {
    return solid18::total_strain::EvaluateForce90Scratch(parent.reference, history, interval, material, scratch);
  }
  TL_SOLID18_HD static const Trial& TrialValue(const ForceScratch& scratch) noexcept {
    return scratch.staged;
  }
  TL_SOLID18_HD static Interval Phase(double base, double dt, std::uint64_t epoch) noexcept {
    Interval result;
    result.base_time_s = base; result.dt_s = dt; result.sample_index = epoch + 1;
    return result;
  }
  TL_SOLID18_HD static void Node(Interval& interval, unsigned n,
      solid18::Vec3 position, solid18::Vec3 velocity) noexcept {
    interval.position_endpoint_m[n] = position;
    interval.velocity_midpoint_m_s[n] = velocity;
  }
  TL_SOLID18_HD static bool Capture(const Trial& trial, Cache& output) noexcept {
    Cache next;
    if (!trial.proposed_history.prepared() || !stiffness_detail::Prepare(
        trial.diagnostics.raw_stiffness_n_m, trial.diagnostics.minimum_unscaled_dt_s,
        .25, next.stiffness)) return false;
    for (unsigned n = 0; n < nodes; ++n) next.rhs_force_n[n] = trial.rhs_force_n[n];
    next.diagnostics = trial.diagnostics;
    output = next;
    return true;
  }
  TL_SOLID18_HD static Result Read(const History& history, const Cache& cache) noexcept {
    return {history.data(), history.stamp(), cache};
  }
};
} // namespace tl::fea::solids::batch_detail
