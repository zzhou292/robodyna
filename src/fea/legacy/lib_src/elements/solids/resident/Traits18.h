// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"
#include "../Model.h"
#include "../../solid18/Solid18Force.h"

namespace tl::fea::solids::batch_detail {
struct Traits18 {
  using Parent = Parent18;
  using Material = solid18::Material;
  using History = solid18::History;
  using Interval = solid18::PrescribedInterval;
  using Trial = solid18::ForceTrial;
  using Cache = Cache18;
  using Result = Result18;
  using Status = solid18::Status;
  static constexpr Family family = Family::Solid18;
  static constexpr unsigned nodes = 8;
  static constexpr Status success = Status::Success;
  TL_SOLID18_HD static Status Initialize(const Parent& parent, const Material& material,
      solid18::Vec3 velocity, Trial& output) noexcept {
    return solid18::InitializeForce(parent.reference, material, velocity, output);
  }
  TL_SOLID18_HD static Status Evaluate(const Parent& parent, const Material& material,
      const History& accepted, const Interval& interval, Trial& output) noexcept {
    return solid18::EvaluateForce(parent.reference, accepted, interval, material, output);
  }
  TL_SOLID18_HD static Interval Phase(double base, double dt, std::uint64_t epoch) noexcept {
    Interval result;
    result.base_time_s = base;
    result.dt_s = dt;
    result.sample_index = epoch + 1;
    return result;
  }
  TL_SOLID18_HD static void Node(Interval& interval, unsigned n,
      solid18::Vec3 position, solid18::Vec3 velocity) noexcept {
    interval.position_endpoint_m[n] = position;
    interval.velocity_midpoint_m_s[n] = velocity;
  }
  TL_SOLID18_HD static bool Capture(const Trial& trial, Cache& output) noexcept {
    Cache next;
    if (!PrepareNodalStiffness(trial, next.stiffness)) return false;
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
