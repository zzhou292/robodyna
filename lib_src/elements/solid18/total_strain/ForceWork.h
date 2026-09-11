// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S8EFMOY3, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "ForceTypes.h"
namespace tl::fea::solid18::total_strain::force_detail {
TL_SOLID18_HD inline void AccumulateGlobal(const Reference& reference,
    const CurrentGeometry& geometry,unsigned ip,const PointObservation& observation,
    GlobalHistory& global,ForceDiagnostics& diagnostics) noexcept {
  const auto& point=observation.material.history;
  const double volume=geometry.point[ip].current_volume_m3;
  const double factor=1.0*volume/geometry.center_volume_m3;
  const double density_factor=volume/geometry.center_volume_m3;
  for(unsigned k=0;k<6;++k)global.stress_pa[k]=global.stress_pa[k]+factor*point.stress_pa[k];
  global.density_kg_m3=global.density_kg_m3+density_factor*point.density_kg_m3;
  global.internal_energy_density_j_m3=global.internal_energy_density_j_m3+
      point.internal_energy_density_j_m3*observation.storage_volume_m3/
      reference.geometry().center_volume_m3;
  global.bulk_pressure_pa=global.bulk_pressure_pa+factor*point.bulk_pressure_pa;
  diagnostics.raw_stiffness_n_m=diagnostics.raw_stiffness_n_m+observation.material.raw_stiffness_n_m;
  global.scalar_rate_per_s=global.scalar_rate_per_s+factor*point.scalar_rate_per_s;
  diagnostics.minimum_unscaled_dt_s=::fmin(diagnostics.minimum_unscaled_dt_s,
      observation.material.unscaled_element_dt_s);
  diagnostics.internal_work_increment_j=diagnostics.internal_work_increment_j+
      observation.material.internal_work_j;
}
} // namespace tl::fea::solid18::total_strain::force_detail
