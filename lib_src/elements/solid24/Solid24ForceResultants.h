// SPDX-License-Identifier: AGPL-3.0-or-later
// Shared selected SFINT3/SRROTA3 expressions, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid24ForceTypes.h"

namespace tl::fea::solid24::force_detail {
TL_BRICK_HD inline void MaterialForces(const ForceGeometry& g,
    const tl::material::law42::CallerHistory& history,Vec3 (&force)[8]) noexcept {
  double s[6];
  for (unsigned k=0; k<3; ++k)
    s[k]=(history.stress_pa[k]+0.0-history.bulk_pressure_pa)*g.current.volume_m3;
  for (unsigned k=3; k<6; ++k) s[k]=(history.stress_pa[k]+0.0)*g.current.volume_m3;
  constexpr unsigned opposite[]{6,7,4,5};
  const auto& p=g.derivative_per_m;
  for (unsigned n=0; n<4; ++n) {
    const double x=s[0]*p[0][n]+s[3]*p[1][n]+s[5]*p[2][n];
    const double y=s[1]*p[1][n]+s[3]*p[0][n]+s[4]*p[2][n];
    const double z=s[2]*p[2][n]+s[5]*p[0][n]+s[4]*p[1][n];
    force[n].x=force[n].x-x; force[opposite[n]].x=force[opposite[n]].x+x;
    force[n].y=force[n].y-y; force[opposite[n]].y=force[opposite[n]].y+y;
    force[n].z=force[n].z-z; force[opposite[n]].z=force[opposite[n]].z+z;
  }
}
TL_BRICK_HD inline ForceStatus RotateAndMapForces(const Reference& reference,
    const Matrix3& frame,const Vec3 (&local_force)[8],Vec3 (&source_world)[8]) noexcept {
  const auto& f=frame.v;
  for (unsigned n=0; n<8; ++n) {
    const auto& a=local_force[n];
    const Vec3 world{f[0]*a.x+f[1]*a.y+f[2]*a.z,
                     f[3]*a.x+f[4]*a.y+f[5]*a.z,
                     f[6]*a.x+f[7]*a.y+f[8]*a.z};
    if (!tl::fea::solid_common::Finite(world)) return ForceStatus::NonfiniteResult;
    source_world[reference.source_slot(n)]=world;
  }
  return ForceStatus::Success;
}
} // namespace tl::fea::solid24::force_detail
