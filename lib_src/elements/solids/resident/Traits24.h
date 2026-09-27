// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"
#include "../Model.h"
#include "../../solid24/Solid24Force.h"
#include "controlled/History.h"

namespace tl::fea::solids::batch_detail {
struct Traits24 {
  using Parent = Parent24;
  using Material = solid24::Material;
  using History = controlled::History24;
  using Interval = solid24::PrescribedInterval;
  using Trial = solid24::ForceTrial;
  using Cache = Cache24;
  using Result = Result24;
  using Status = solid24::ForceStatus;
  static constexpr Family family = Family::Solid24;
  static constexpr unsigned nodes = 8;
  static constexpr Status success = Status::Success;
  TL_BRICK_HD static Status Initialize(const Parent& parent, const Material& material,
      solid24::Vec3 velocity, Trial& output) noexcept {
    return solid24::InitializeForce(parent.reference, material, velocity, output);
  }
  TL_BRICK_HD static Status Evaluate(const Parent& parent, const Material& material,
      const History& accepted, const Interval& interval, Trial& output) noexcept {
    const auto* history=accepted.legacy();
    if(!history)return Status::UnsupportedProfile;
    return solid24::EvaluateForce(parent.reference, *history, interval, material, output);
  }
  TL_BRICK_HD static Interval Phase(double base, double dt, std::uint64_t epoch) noexcept {
    Interval result;
    result.base_time_s = base;
    result.dt_s = dt;
    result.sample_index = epoch + 1;
    return result;
  }
  TL_BRICK_HD static void Node(Interval& interval, unsigned n,
      solid24::Vec3 position, solid24::Vec3 velocity) noexcept {
    interval.position_m[n] = position;
    interval.velocity_m_s[n] = velocity;
  }
  TL_BRICK_HD static bool Capture(const Trial& trial, Cache& output) noexcept {
    Cache next;
    if (!PrepareNodalStiffness(trial, next.stiffness)) return false;
    for (unsigned n = 0; n < nodes; ++n) next.rhs_force_n[n] = trial.rhs_force_n[n];
    next.diagnostics = trial.diagnostics;
    output = next;
    return true;
  }
  TL_BRICK_HD static Result Read(const History& history, const Cache& cache) noexcept {
    return {history.legacy()->values(), history.stamp(), cache};
  }
};
} // namespace tl::fea::solids::batch_detail
