// SPDX-License-Identifier: AGPL-3.0-or-later
// S8EFINT3/S8EFMOY3/SRROTA3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18ForceValues.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline void AccumulateForce(const PointDerivatives& geometry,
    const PointHistory& history, Vec3 (&force)[8]) noexcept {
  double stress[6];
  PointStressVolume(history.material.point.stress_pa,history.bulk_pressure_pa,
                    geometry.current_volume_m3,stress);
  AccumulateSelectedShearForce(geometry,stress,force);
  AccumulateCrossForce(geometry,stress,force);
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

}  // namespace tl::fea::solid18::detail
