// SPDX-License-Identifier: AGPL-3.0-or-later
// S8EFINT3/SRROTA3 values, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18ForceTypes.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline void PointStressVolume(const double (&cauchy)[6],
    double pressure, double volume, double (&stress)[6]) noexcept {
  for (unsigned k=0; k<3; ++k) stress[k]=(cauchy[k]+0.0-pressure)*volume;
  for (unsigned k=3; k<6; ++k) stress[k]=(cauchy[k]+0.0)*volume;
}
TL_SOLID18_HD inline void AccumulateSelectedShearForce(const PointDerivatives& geometry,
    const double (&stress)[6], Vec3 (&force)[8]) noexcept {
  const auto& p=geometry.regular_per_m;
  const auto& s=geometry.shear_per_m;
  for (unsigned n=0; n<8; ++n) {
    force[n].x=force[n].x-(stress[0]*p[0][n]+stress[3]*s[0][n]+stress[5]*s[2][n]);
    force[n].y=force[n].y-(stress[1]*p[1][n]+stress[3]*s[1][n]+stress[4]*s[4][n]);
    force[n].z=force[n].z-(stress[2]*p[2][n]+stress[5]*s[3][n]+stress[4]*s[5][n]);
  }
}
TL_SOLID18_HD inline void AccumulateCrossForce(const PointDerivatives& geometry,
    const double (&stress)[6], Vec3 (&force)[8]) noexcept {
  const auto& b=geometry.cross_per_m;
  for (unsigned n=0; n<8; ++n) {
    force[n].x=force[n].x-(stress[1]*b[0][n]+stress[2]*b[2][n]);
    force[n].y=force[n].y-(stress[0]*b[1][n]+stress[2]*b[4][n]);
    force[n].z=force[n].z-(stress[0]*b[3][n]+stress[1]*b[5][n]);
  }
}
TL_SOLID18_HD inline Vec3 WorldForce(const Matrix3& frame, const Vec3& force) noexcept {
  return {frame.v[0]*force.x+frame.v[1]*force.y+frame.v[2]*force.z,
          frame.v[3]*force.x+frame.v[4]*force.y+frame.v[5]*force.z,
          frame.v[6]*force.x+frame.v[7]*force.y+frame.v[8]*force.z};
}
} // namespace tl::fea::solid18::detail
