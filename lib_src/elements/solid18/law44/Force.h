// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "History.h"
#include "PointResponse.h"
#include "ForceAssembly.h"
#include "lib_src/elements/solid18/Solid18ForceValues.h"

namespace tl::fea::solid18::law44 {
namespace detail {
// Private unpublished staging; the public entry publishes only after every IP.
TL_SOLID18_HD inline Status CalculateForceStaged(const Reference& reference,
    const History& accepted, const PrescribedInterval& interval, const Material& material,
    ForceTrial& trial, HistoryValues& next, StartupGeometry& geometry_scratch) noexcept {
  const auto status = GeometryValues(reference,interval,trial.geometry,geometry_scratch);
  if (status != Status::Success) return status;
  trial.diagnostics = {};
  auto& diagnostics = trial.diagnostics;
  diagnostics.native_degeneracy = NativeDegeneracy(reference);
  diagnostics.caller_degeneracy = diagnostics.native_degeneracy > 0 ? diagnostics.native_degeneracy+10 : 0;
  diagnostics.center_divergence_per_s = CenterDivergence(trial.geometry);
  if (!tl::math::Finite(diagnostics.center_divergence_per_s)) return Status::NonfiniteResult;
  diagnostics.minimum_unscaled_dt_s = 1e30;
  next = accepted.data();
  next.global = {};
  for (unsigned n = 0; n < 7; ++n) {
    const auto& x = trial.geometry.local_position_m[n];
    const auto& last = trial.geometry.local_position_m[7];
    next.saved_local_position_m[n] = {x.x-last.x,x.y-last.y,x.z-last.z};
  }
  Vec3 local_force[8]{};
  double length = 1e30;
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        const unsigned ip = r+2*s+4*t;
        const auto& geometry = trial.geometry.point[ip];
        length = ::fmin(length,128*geometry.current_volume_m3*
                        trial.geometry.inverse_center_face_scale_per_m2/1.0);
        const auto point_status = PointResponse(material,accepted.data().point[ip],geometry,
            trial.geometry.local_velocity_m_s,diagnostics.center_divergence_per_s,
            diagnostics.caller_degeneracy,interval.dt_s,length,next.point[ip],trial.point[ip]);
        if (point_status != Status::Success) return point_status;
        AccumulatePointForce(geometry,next.point[ip],diagnostics.caller_degeneracy,local_force);
        AccumulateGlobal(reference,trial.geometry,ip,next.point[ip],trial.point[ip],next.global,diagnostics);
      }
    }
  }
  AccumulateCenterPressure(trial.geometry,diagnostics.caller_degeneracy,diagnostics.mean_pressure_pa,local_force);
  for (unsigned n = 0; n < 8; ++n) {
    const Vec3 force = solid18::detail::WorldForce(trial.geometry.frame,local_force[n]);
    if (!solid18::detail::Finite(force)) return Status::NonfiniteResult;
    trial.rhs_force_n[reference.source_slot(n)] = force;
  }
  if (!solid18::detail::Positive(diagnostics.minimum_unscaled_dt_s) ||
      !solid18::detail::Positive(diagnostics.raw_stiffness_n_m) ||
      !tl::math::Finite(diagnostics.mean_pressure_pa) ||
      !tl::math::Finite(diagnostics.internal_work_increment_j) ||
      !tl::math::Finite(diagnostics.plastic_work_increment_j)) return Status::NonfiniteResult;
  return HistoryWriter::Prepare(reference,material,next,
      {interval.base_time_s+interval.dt_s,interval.sample_index},trial.proposed_history);
}
}
// Active positive-Jacobian prescribed recurrence, with exact source/material
// identity. The caller owns acceptance and immutable curve lifetime.
TL_SOLID18_HD inline Status EvaluateForce(const Reference& reference, const History& accepted,
    const PrescribedInterval& interval, const Material& material, ForceTrial& output) noexcept {
  if (!detail::ValidMaterial(reference,material) || !detail::ValidInterval(accepted,interval) ||
      !detail::SameReference(reference,accepted.reference()) ||
      !detail::SameMaterial(material,accepted.material()) ||
      !detail::ValidHistory(reference,material,accepted.data())) return Status::InvalidInput;
  ForceTrial trial;
  HistoryValues next;
  StartupGeometry scratch;
  const auto status = detail::CalculateForceStaged(reference,accepted,interval,material,trial,next,scratch);
  if (status == Status::Success) output = trial;
  return status;
}
}  // namespace tl::fea::solid18::law44
