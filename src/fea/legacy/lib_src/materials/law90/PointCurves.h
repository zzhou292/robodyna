// SPDX-License-Identifier: AGPL-3.0-or-later
// Engine SIGEPS90:326–354,405–440 and515–553, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "PointKinematics.h"
#include "Curve.h"
#include "Prepare.h"

namespace tl::material::law90::point_detail {
struct CurveResponse {
  double stress[3]{};
  double quasistatic_energy = 0;
  double minimum_slope = 1e20;
  double stress_norm = 0;
};
TL_LAW90_HD inline PointStatus CurvePass(const PreparedMaterial& material,
    const Kinematics& kinematics, PointHistory& history, CurveResponse& response,
    bool explicit_unloading_curve = false) noexcept {
  response = {};
  for (unsigned k = 0; k < 3; ++k) {
    CurveResult curve;
    const Status status = LookupCurve(material, kinematics.compression[k], history.cursor[k], curve);
    if (status == Status::InvalidCursor) return PointStatus::InvalidCursor;
    if (status != Status::Ok) return PointStatus::InvalidCurve;
    history.cursor[k] = curve.cursor;
    response.stress[k] = material.reader().curve_scale * curve.stress_pa;
    if (kinematics.compression[k] < 0)
      response.stress[k] = material.updated().young_pa * kinematics.compression[k];
    response.quasistatic_energy = response.quasistatic_energy +
        .5 * kinematics.compression[k] * response.stress[k];
    const double slope = material.reader().curve_scale * curve.slope_pa;
    if (!tl::math::Finite(slope) || slope < 0) return PointStatus::InvalidCurve;
    // IFLAG1 retains the donor MAX from the finite EP20 initial value.
    // It is not the IFLAG2 minimum and must not be "corrected" to one.
    response.minimum_slope = explicit_unloading_curve
        ? detail::Maximum(response.minimum_slope, slope)
        : detail::Minimum(response.minimum_slope, slope);
  }
  response.stress_norm = ::sqrt(response.stress[0]*response.stress[0] +
      response.stress[1]*response.stress[1] + response.stress[2]*response.stress[2]);
  if (!tl::math::Finite(response.quasistatic_energy) || !tl::math::Finite(response.stress_norm))
    return PointStatus::NonfiniteResult;
  return PointStatus::Ok;
}
} // namespace tl::material::law90::point_detail
