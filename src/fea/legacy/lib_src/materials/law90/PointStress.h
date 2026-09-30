// SPDX-License-Identifier: AGPL-3.0-or-later
// Engine SIGEPS90:765–805, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "PointCurves.h"

namespace tl::material::law90::point_detail {
TL_LAW90_HD inline void CauchyStress(const PreparedMaterial& material,
    const Kinematics& kinematics, const CurveResponse& response, bool unloading,
    PointResult& result) noexcept {
  double cutoff = material.reader().tension_cutoff_pa;
  if (unloading) cutoff = result.history.unloading_factor * cutoff;
  double stress[3];
  for (unsigned k = 0; k < 3; ++k) stress[k] = detail::Minimum(cutoff, -response.stress[k]);
  stress[0] = stress[0]/kinematics.stretch[1]/kinematics.stretch[2];
  stress[1] = stress[1]/kinematics.stretch[0]/kinematics.stretch[2];
  stress[2] = stress[2]/kinematics.stretch[0]/kinematics.stretch[1];
  const double* v = kinematics.spectrum.vectors.v;
  double* s = result.cauchy_stress_pa;
  s[0] = v[0]*v[0]*stress[0] + v[1]*v[1]*stress[1] + v[2]*v[2]*stress[2];
  s[1] = v[4]*v[4]*stress[1] + v[5]*v[5]*stress[2] + v[3]*v[3]*stress[0];
  s[2] = v[8]*v[8]*stress[2] + v[6]*v[6]*stress[0] + v[7]*v[7]*stress[1];
  s[3] = v[0]*v[3]*stress[0] + v[1]*v[4]*stress[1] + v[2]*v[5]*stress[2];
  s[4] = v[4]*v[7]*stress[1] + v[5]*v[8]*stress[2] + v[3]*v[6]*stress[0];
  s[5] = v[8]*v[2]*stress[2] + v[6]*v[0]*stress[0] + v[7]*v[1]*stress[1];
}
} // namespace tl::material::law90::point_detail
