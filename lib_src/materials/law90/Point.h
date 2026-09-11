// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "PointChecks.h"
#include "PointHistoryUpdate.h"
#include "PointStress.h"

namespace tl::material::law90 {
namespace point_detail {
TL_LAW90_HD inline PointStatus Evaluate(const PreparedMaterial& material,
    const PointHistory& history, const PointKinematics& input, PointResult& output) noexcept {
  Kinematics kinematics;
  PointStatus status = ResolveKinematics(input, kinematics);
  if (status != PointStatus::Ok) return status;
  PointResult next;
  next.history = history;
  CurveResponse response;
  status = CurvePass(material, kinematics, next.history, response);
  if (status != PointStatus::Ok) return status;
  const bool unloading = UpdateLoading(kinematics, response, next);
  // The second native VINTER2 visit is required even with one immutable curve.
  const bool explicit_unloading_curve = material.reader().loading_flag == 1;
  status = CurvePass(material, kinematics, next.history, response, explicit_unloading_curve);
  if (status != PointStatus::Ok) return status;
  next.tangent_factor = response.minimum_slope/material.updated().young_pa;
  // SIGEPS90's UVAR1/2/4/6/7 path/damage block belongs only to IFLAG2.
  // IFLAG1 carries those slots unchanged while common rate/modulus state evolves.
  if (!explicit_unloading_curve)
    UpdatePath(material, kinematics, unloading, response, next.history);
  CauchyStress(material, kinematics, response, unloading, next);
  UpdateModulus(material, kinematics, response, next);
  status = CheckResult(material, next);
  if (status != PointStatus::Ok) return status;
  output = next;
  return PointStatus::Ok;
}
} // namespace point_detail

// Virgin TIME0 path: no prior history is accepted, and no interval is counted.
TL_LAW90_HD inline PointStatus InitializePointSI(const PreparedMaterial& material,
    const PointKinematics& input, PointResult& output) noexcept {
  if (!material.initialized()) return PointStatus::InvalidMaterial;
  PointHistory virgin;
  virgin.unloading_factor = 1;
  virgin.effective_modulus_pa = material.updated().young_pa;
  return point_detail::Evaluate(material, virgin, input, output);
}

// TIME>0 path: the caller owns the clock. No history reset or synthetic dt.
TL_LAW90_HD inline PointStatus UpdatePointSI(const PreparedMaterial& material,
    const PointHistory& history, const PointKinematics& input, double time_s,
    PointResult& output) noexcept {
  if (!material.initialized()) return PointStatus::InvalidMaterial;
  if (!tl::math::Finite(time_s) || time_s <= 0) return PointStatus::InvalidInput;
  const PointStatus status = point_detail::CheckHistory(material, history);
  if (status != PointStatus::Ok) return status;
  return point_detail::Evaluate(material, history, input, output);
}
} // namespace tl::material::law90
