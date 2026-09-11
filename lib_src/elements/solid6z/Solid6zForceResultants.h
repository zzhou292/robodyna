// SPDX-License-Identifier: AGPL-3.0-or-later
// S6ZFINT3 / S6ZRROTA3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid6zForceTypes.h"

namespace tl::fea::solid6z::force_detail {
TL_BRICK_HD inline bool MaterialForces(const CurrentGeometry& geometry,
    const tl::material::law42::CallerResult& material, Vec3 (&output)[6]) noexcept {
  const auto& stress = material.history.stress_pa;
  const double pressure = material.history.bulk_pressure_pa;
  const double volume = geometry.current_volume_m3;
  double s[6];
  for (unsigned k = 0; k < 3; ++k) s[k] = (stress[k]+0.0-pressure)*volume;
  for (unsigned k = 3; k < 6; ++k) s[k] = (stress[k]+0.0)*volume;
  for (unsigned n = 0; n < 6; ++n) {
    const double px = geometry.point_gradient_per_m[0][n];
    const double py = geometry.point_gradient_per_m[1][n];
    const double pz = geometry.point_gradient_per_m[2][n];
    output[n].x = 0.0-(s[0]*px+s[3]*py+s[5]*pz);
    output[n].y = 0.0-(s[1]*py+s[4]*pz+s[3]*px);
    output[n].z = 0.0-(s[2]*pz+s[5]*px+s[4]*py);
    if (!solid_common::Finite(output[n])) return false;
  }
  return true;
}
TL_BRICK_HD inline bool Project(const Reference& reference, const Matrix3& frame,
    const Vec3 (&local)[6], Vec3 (&world)[6]) noexcept {
  for (unsigned n = 0; n < 6; ++n) {
    const auto& f = local[n];
    Vec3 force;
    force.x = frame.v[0]*f.x+frame.v[1]*f.y+frame.v[2]*f.z;
    force.y = frame.v[3]*f.x+frame.v[4]*f.y+frame.v[5]*f.z;
    force.z = frame.v[6]*f.x+frame.v[7]*f.y+frame.v[8]*f.z;
    if (!solid_common::Finite(force)) return false;
    world[reference.source_slot(n)] = force;
  }
  return true;
}
} // namespace tl::fea::solid6z::force_detail
