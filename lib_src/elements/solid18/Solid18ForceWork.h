// SPDX-License-Identifier: AGPL-3.0-or-later
// S8EFINT3/S8EFMOY3/SRROTA3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18ForceTypes.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline void AccumulateForce(const PointDerivatives& geometry,
    const PointHistory& history, Vec3 (&force)[8]) noexcept {
  double stress[6];
  for (unsigned k = 0; k < 3; ++k) {
    stress[k] = (history.material.point.stress_pa[k]+0.0-history.bulk_pressure_pa)*
                geometry.current_volume_m3;
  }
  for (unsigned k = 3; k < 6; ++k) {
    stress[k] = (history.material.point.stress_pa[k]+0.0)*geometry.current_volume_m3;
  }
  const auto& p = geometry.regular_per_m;
  const auto& s = geometry.shear_per_m;
  const auto& b = geometry.cross_per_m;
  // Native regular/shear subtraction precedes the separate cross-term pass.
  for (unsigned n = 0; n < 8; ++n) {
    force[n].x = force[n].x-(stress[0]*p[0][n]+stress[3]*s[0][n]+stress[5]*s[2][n]);
    force[n].y = force[n].y-(stress[1]*p[1][n]+stress[3]*s[1][n]+stress[4]*s[4][n]);
    force[n].z = force[n].z-(stress[2]*p[2][n]+stress[5]*s[3][n]+stress[4]*s[5][n]);
  }
  for (unsigned n = 0; n < 8; ++n) {
    force[n].x = force[n].x-(stress[1]*b[0][n]+stress[2]*b[2][n]);
    force[n].y = force[n].y-(stress[0]*b[1][n]+stress[2]*b[4][n]);
    force[n].z = force[n].z-(stress[0]*b[3][n]+stress[1]*b[5][n]);
  }
}

TL_SOLID18_HD inline void AccumulateGlobal(const Reference& reference,
    const CurrentGeometry& geometry, unsigned ip, const PointHistory& point,
    const PointObservation& observation, GlobalHistory& global,
    ForceDiagnostics& diagnostics) noexcept {
  const double volume = geometry.point[ip].current_volume_m3;
  const double factor = 1.0*volume/geometry.center_volume_m3;
  const double density_factor = volume/geometry.center_volume_m3;
  for (unsigned k = 0; k < 6; ++k) {
    global.stress_pa[k] = global.stress_pa[k]+factor*point.material.point.stress_pa[k];
  }
  global.density_kg_m3 = global.density_kg_m3+density_factor*point.density_kg_m3;
  global.internal_energy_density_j_m3 = global.internal_energy_density_j_m3+
      point.material.internal_energy_density_j_m3*point.storage_volume_m3/
      reference.geometry().center_volume_m3;
  global.plastic_work_j = global.plastic_work_j+point.material.plastic_work_j;
  global.bulk_pressure_pa = global.bulk_pressure_pa+factor*point.bulk_pressure_pa;
  diagnostics.raw_stiffness_n_m = diagnostics.raw_stiffness_n_m+observation.raw_stiffness_n_m;
  global.plastic_strain = global.plastic_strain+factor*point.material.point.plastic_strain;
  diagnostics.minimum_unscaled_dt_s = ::fmin(diagnostics.minimum_unscaled_dt_s,
                                            observation.unscaled_element_dt_s);
  diagnostics.internal_work_increment_j += observation.material.internal_work_j;
  diagnostics.plastic_work_increment_j += observation.material.plastic_work_increment_j;
}

TL_SOLID18_HD inline Vec3 WorldForce(const Matrix3& frame, const Vec3& force) noexcept {
  return {frame.v[0]*force.x+frame.v[1]*force.y+frame.v[2]*force.z,
          frame.v[3]*force.x+frame.v[4]*force.y+frame.v[5]*force.z,
          frame.v[6]*force.x+frame.v[7]*force.y+frame.v[8]*force.z};
}
}  // namespace tl::fea::solid18::detail
