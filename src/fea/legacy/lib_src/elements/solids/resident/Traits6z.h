// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"
#include "../Model.h"
#include "../../solid6z/Solid6zForce.h"

namespace tl::fea::solids::batch_detail {
struct Traits6z {
  using Parent = Parent6z;
  using Material = solid6z::Material;
  using History = solid6z::History;
  using Interval = solid6z::PrescribedInterval;
  using Trial = solid6z::ForceTrial;
  using Cache = Cache6z;
  using Result = Result6z;
  using Status = solid6z::Status;
  static constexpr Family family = Family::Solid6z;
  static constexpr unsigned nodes = 6;
  static constexpr Status success = Status::Success;
  TL_BRICK_HD static Status Initialize(const Parent& parent, const Material& material,
      solid6z::Vec3 velocity, Trial& output) noexcept {
    return solid6z::InitializeForce(parent.reference, material, parent.profile, velocity, output);
  }
  TL_BRICK_HD static Status Evaluate(const Parent& parent, const Material& material,
      const History& accepted, const Interval& interval, Trial& output) noexcept {
    return solid6z::EvaluateForce(parent.reference, accepted, interval, material, parent.profile, output);
  }
  TL_BRICK_HD static Interval Phase(double base, double dt, std::uint64_t epoch) noexcept {
    Interval result;
    result.base_time_s = base;
    result.dt_s = dt;
    result.sample_index = epoch; // This existing family labels the base sample.
    return result;
  }
  TL_BRICK_HD static void Node(Interval& interval, unsigned n,
      solid6z::Vec3 position, solid6z::Vec3 velocity) noexcept {
    interval.position_endpoint_m[n] = position;
    interval.velocity_midpoint_m_s[n] = velocity;
  }
  TL_BRICK_HD static bool Capture(const Trial& trial, Cache& output) noexcept {
    Cache next;
    if (!PrepareNodalStiffness(trial, next.stiffness)) return false;
    for (unsigned n = 0; n < nodes; ++n) next.rhs_force_n[n] = trial.rhs_force_n[n];
    next.material = trial.material;
    next.stabilization = trial.stabilization;
    next.total_internal_work_increment_j = trial.total_internal_work_increment_j;
    output = next;
    return true;
  }
  TL_BRICK_HD static Result Read(const History& history, const Cache& cache) noexcept {
    return {history.data(), history.stamp(), cache};
  }
};
} // namespace tl::fea::solids::batch_detail
