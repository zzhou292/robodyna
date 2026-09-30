// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected eight-point S8EFORC3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18History.h"
#include "Solid18CurrentGeometry.h"
#include "Solid18Selection.h"
#include "Solid18SelectiveShear.h"
#include "Solid18PointResponse.h"
#include "Solid18ForceWork.h"

namespace tl::fea::solid18 {
namespace detail {
// Shared selected force arithmetic. Initialization is entered only by the
// constructor below, with generated virgin history and exact reference fields.
TL_SOLID18_HD inline Status CalculateForceStaged(const Reference& reference,
    const History& accepted, const PrescribedInterval& interval,
    const Material& material, ForceTrial& trial, HistoryValues& next,
    StartupGeometry& geometry_scratch, bool initialization) noexcept {
  trial.diagnostics = {};
  Status status = detail::SelectAcceptedPoint(material,accepted.data(),trial.diagnostics);
  if (status != Status::Success) return status;
  status = detail::CurrentGeometryValues(reference,interval,trial.geometry,geometry_scratch);
  if (status != Status::Success) return status;
  detail::SelectiveDerivatives(trial.diagnostics.selective_poisson_ratio,trial.geometry);
  next = accepted.data();
  next.global = {}; // S8ZZERO3: accepted global values already consumed by selection.
  for (unsigned n = 0; n < 7; ++n) {
    const auto& x = trial.geometry.local_position_m[n];
    const auto& last = trial.geometry.local_position_m[7];
    next.saved_local_position_m[n] = {x.x-last.x,x.y-last.y,x.z-last.z};
  }
  Vec3 local_force[8]{};
  double length = 1e30;
  trial.diagnostics.minimum_unscaled_dt_s = 1e30;
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        const unsigned ip = r+2*s+4*t;
        const auto& geometry = trial.geometry.point[ip];
        // S8EDERI_2 updates the running minimum before this point's MQVISCB.
        length = ::fmin(length,128*geometry.current_volume_m3*
                        trial.geometry.inverse_center_face_scale_per_m2/1.0);
        status = detail::PointResponse(material,accepted.data().point[ip],geometry,
            trial.geometry.local_velocity_m_s,interval.dt_s,length,
            next.point[ip],trial.point[ip],initialization);
        if (status != Status::Success) return status;
        detail::AccumulateForce(geometry,next.point[ip],local_force);
        detail::AccumulateGlobal(reference,trial.geometry,ip,next.point[ip],
            trial.point[ip],next.global,trial.diagnostics);
      }
    }
  }
  for (unsigned n = 0; n < 8; ++n) {
    const Vec3 force = detail::WorldForce(trial.geometry.frame,local_force[n]);
    if (!detail::Finite(force)) return Status::NonfiniteResult;
    trial.rhs_force_n[reference.source_slot(n)] = force;
  }
  if (!detail::Positive(trial.diagnostics.minimum_unscaled_dt_s) ||
      !detail::Positive(trial.diagnostics.raw_stiffness_n_m) ||
      !tl::math::Finite(trial.diagnostics.internal_work_increment_j) ||
      !tl::math::Finite(trial.diagnostics.plastic_work_increment_j))
    return Status::NonfiniteResult;
  const HistoryStamp stamp{interval.base_time_s+interval.dt_s,interval.sample_index};
  status = HistoryWriter::Prepare(reference,material,next,stamp,trial.proposed_history);
  if (status != Status::Success) return status;
  return Status::Success;
}
TL_SOLID18_HD inline Status CalculateForce(const Reference& reference,
    const History& accepted, const PrescribedInterval& interval,
    const Material& material, ForceTrial& output, bool initialization) noexcept {
  ForceTrial trial;
  HistoryValues next;
  StartupGeometry geometry_scratch;
  const auto status = CalculateForceStaged(reference,accepted,interval,material,
      trial,next,geometry_scratch,initialization);
  if (status != Status::Success) return status;
  output = trial;
  return Status::Success;
}
} // namespace detail
// Pure active positive-Jacobian recurrence. All borrowed inputs are consumed
// before publication; this is not a state owner or a zero-duration update API.
TL_SOLID18_HD inline Status EvaluateForce(const Reference& reference,
    const History& accepted, const PrescribedInterval& interval,
    const Material& material, ForceTrial& output) noexcept {
  if (!detail::ValidMaterial(reference,material) || !detail::ValidInterval(accepted,interval) ||
      !detail::SameReference(reference,accepted.reference()) ||
      !detail::SameMaterial(material,accepted.material()) ||
      !detail::ValidHistory(reference,accepted.data())) return Status::InvalidInput;
  return detail::CalculateForce(reference,accepted,interval,material,output,false);
}
// Native TT0/DT1=0 construction. Initial material/rate/Q and cache fields are
// evaluated once; sample0 is not a completed physical interval.
TL_SOLID18_HD inline Status InitializeForce(const Reference& reference,
    const Material& material, Vec3 uniform_velocity_m_s, ForceTrial& output) noexcept {
  if (!detail::Finite(uniform_velocity_m_s)) return Status::InvalidInput;
  History virgin;
  const Status status=InitializeHistory(reference,material,virgin);
  if (status!=Status::Success) return status;
  PrescribedInterval initial;
  for (unsigned n=0; n<8; ++n) {
    initial.position_endpoint_m[n]=reference.input().position_m[n];
    initial.velocity_midpoint_m_s[n]=uniform_velocity_m_s;
  }
  return detail::CalculateForce(reference,virgin,initial,material,output,true);
}
}  // namespace tl::fea::solid18
