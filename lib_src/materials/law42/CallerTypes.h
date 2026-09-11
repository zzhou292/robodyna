// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"

namespace tl::material::law42 {
// Selected active ISMSTR10 solid caller. All dimensional values use SI.
struct CallerHistory {
  double stress_pa[6]{};
  double density_kg_m3 = 0;
  double internal_energy_density_j_m3 = 0;
  double bulk_pressure_pa = 0;
};
struct CallerInput {
  double displacement_gradient[9]{}; // Row-major, in the current material frame.
  double engineering_rate_per_s[6]{};
  double dt_s = 0;
  double current_volume_m3 = 0;
  double storage_volume_m3 = 0;
  double characteristic_length_m = 0;
};
struct CallerResult {
  CallerHistory history;
  Result point;
  double total_strain[6]{};
  double volume_increment_m3 = 0;
  double average_volume_m3 = 0;
  double internal_work_j = 0;
  double unscaled_element_dt_s = 0;
  double raw_stiffness_n_m = 0;
};
} // namespace tl::material::law42
