// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "PointTypes.h"

namespace tl::material::law90 {
// Independent six-stress caller history plus the complete ten UVAR/three
// cursors. EPSD is a strain-rate output, never a J2 plastic-strain field.
struct CallerHistory {
  PointHistory point{};
  double stress_pa[6]{};
  double density_kg_m3=0;
  double internal_energy_density_j_m3=0;
  double bulk_pressure_pa=0;
  double scalar_rate_per_s=0;
};
struct CallerInput {
  double selected_b_minus_identity[6]{}; // XX/YY/ZZ/XY/YZ/XZ tensor shear.
  double engineering_rate_per_s[6]{};
  double endpoint_time_s=0, dt_s=0;
  double current_volume_m3=0, storage_volume_m3=0, characteristic_length_m=0;
};
struct CallerResult {
  CallerHistory history{};
  PointResult point{};
  double volume_increment_m3=0, average_volume_m3=0;
  double density_compression=0; // MMAIN AMU=rho/rho0-1, not a point-law input.
  double internal_work_j=0, unscaled_element_dt_s=0, raw_stiffness_n_m=0;
};
} // namespace tl::material::law90
