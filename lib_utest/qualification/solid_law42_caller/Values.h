// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law42/Caller.h"
#include <vector>
#include <cstring>
namespace law42_caller_test {
namespace law=tl::material::law42;
inline std::vector<double> Values(const law::CallerResult& r) {
  std::vector<double> values;
  for (double x : r.history.stress_pa) values.push_back(x);
  values.insert(values.end(), {r.history.density_kg_m3,
      r.history.internal_energy_density_j_m3, r.history.bulk_pressure_pa});
  for (double x : r.point.stress_pa) values.push_back(x);
  values.insert(values.end(), {r.point.maximum_principal_stress_pa,
      r.point.minimum_principal_stress_pa, r.point.active, r.point.relative_volume,
      r.point.sound_speed_m_s, r.point.hourglass_tangent_factor,
      r.point.material_viscosity_pa_s});
  for (double x : r.total_strain) values.push_back(x);
  values.insert(values.end(), {r.volume_increment_m3, r.average_volume_m3,
      r.internal_work_j, r.unscaled_element_dt_s, r.raw_stiffness_n_m});
  return values;
}
inline bool Same(const law::CallerResult& a, const law::CallerResult& b) {
  const auto x = Values(a), y = Values(b);
  return std::memcmp(x.data(), y.data(), x.size()*sizeof(double)) == 0;
}

}
