// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S8EFINT3 I_SH0/ICP1, S8EFMOY3 and S8ZFINTP3.
#pragma once
#include "ForceTypes.h"

namespace tl::fea::solid18::law44::detail {
// S8EFINT3 and S8EFMOY3 use ZEP3, defined as THREE/TEN in CONSTANT_MOD.
// This selected pressure coefficient is distinct from native THIRD.
inline constexpr double native_pressure_fraction = 3.0/10.0;
TL_SOLID18_HD inline void AccumulatePointForce(const PointDerivatives& geometry,
    const PointHistory& history, unsigned degeneracy, Vec3 (&force)[8]) noexcept {
  const auto& sig = history.material.stress_pa;
  double pressure = native_pressure_fraction*(sig[0]+sig[1]+sig[2]+0.0+0.0+0.0);
  if (degeneracy > 10) pressure = history.bulk_pressure_pa;
  double stress[6];
  for (unsigned k = 0; k < 3; ++k) stress[k] = (sig[k]+0.0-pressure)*geometry.current_volume_m3;
  for (unsigned k = 3; k < 6; ++k) stress[k] = (sig[k]+0.0)*geometry.current_volume_m3;
  const auto& p = geometry.regular_per_m;
  for (unsigned n = 0; n < 8; ++n) {
    force[n].x = force[n].x-(stress[0]*p[0][n]+stress[3]*p[1][n]+stress[5]*p[2][n]);
    force[n].y = force[n].y-(stress[1]*p[1][n]+stress[3]*p[0][n]+stress[4]*p[2][n]);
    force[n].z = force[n].z-(stress[2]*p[2][n]+stress[5]*p[0][n]+stress[4]*p[1][n]);
  }
}
TL_SOLID18_HD inline void AccumulateGlobal(const Reference& reference, const CurrentGeometry& geometry,
    unsigned ip, const PointHistory& point_history, const PointObservation& observation,
    GlobalHistory& global, ForceDiagnostics& diagnostics) noexcept {
  const double volume = geometry.point[ip].current_volume_m3;
  const double factor = 1.0*volume/geometry.center_volume_m3;
  const double density_factor = volume/geometry.center_volume_m3;
  const auto& stress = point_history.material.stress_pa;
  for (unsigned k = 0; k < 6; ++k) global.stress_pa[k] = global.stress_pa[k]+factor*stress[k];
  global.density_kg_m3 = global.density_kg_m3+density_factor*point_history.density_kg_m3;
  global.energy_density_j_m3 = global.energy_density_j_m3+
      point_history.energy_density_j_m3*point_history.storage_volume_m3/reference.geometry().center_volume_m3;
  global.plastic_work_j = global.plastic_work_j+point_history.plastic_work_j;
  global.bulk_pressure_pa = global.bulk_pressure_pa+factor*point_history.bulk_pressure_pa;
  diagnostics.raw_stiffness_n_m = diagnostics.raw_stiffness_n_m+observation.raw_stiffness_n_m;
  global.plastic_strain = global.plastic_strain+factor*point_history.material.plastic_strain;
  global.filtered_rate_per_s = global.filtered_rate_per_s+factor*point_history.material.filtered_rate_per_s;
  const double mean = native_pressure_fraction*(stress[0]+stress[1]+stress[2]+0.0+0.0+0.0);
  diagnostics.mean_pressure_pa = diagnostics.mean_pressure_pa+factor*(mean-point_history.bulk_pressure_pa);
  diagnostics.minimum_unscaled_dt_s = ::fmin(diagnostics.minimum_unscaled_dt_s,observation.unscaled_element_dt_s);
  diagnostics.internal_work_increment_j += observation.internal_work_j;
  diagnostics.plastic_work_increment_j += observation.plastic_work_increment_j;
}
TL_SOLID18_HD inline void AccumulateCenterPressure(const CurrentGeometry& geometry,
    unsigned degeneracy, double pressure, Vec3 (&force)[8]) noexcept {
  if (degeneracy > 10) return;
  const double sp = pressure*geometry.center_volume_m3;
  constexpr unsigned opposite[]{6,7,4,5};
  for (unsigned n = 0; n < 4; ++n) {
    const double sx = sp*geometry.center_gradient_per_m[0][n];
    const double sy = sp*geometry.center_gradient_per_m[1][n];
    const double sz = sp*geometry.center_gradient_per_m[2][n];
    force[n].x = force[n].x-sx;
    force[opposite[n]].x = force[opposite[n]].x+sx;
    force[n].y = force[n].y-sy;
    force[opposite[n]].y = force[opposite[n]].y+sy;
    force[n].z = force[n].z-sz;
    force[opposite[n]].z = force[opposite[n]].z+sz;
  }
}
}  // namespace tl::fea::solid18::law44::detail
