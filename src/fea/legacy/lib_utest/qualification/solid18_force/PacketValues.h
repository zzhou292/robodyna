// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "lib_utest/qualification/solid_law36_point/TestSupport.h"
#include <iomanip>

namespace solid18_force_test {
inline NativeResult Values(const s::ForceTrial& trial) {
  NativeResult values;
  values.status = 0;
  const auto& data = trial.proposed_history.data();
  for (unsigned ip = 0; ip < 8; ++ip) {
    const auto& point = data.point[ip];
    const auto state = law36_test::Values(point.material.point);
    auto begin = values.next.point.begin()+20*ip;
    std::copy(state.begin(),state.end(),begin);
    begin[14] = point.material.internal_energy_density_j_m3;
    begin[15] = point.material.plastic_work_j;
    begin[16] = point.density_kg_m3;
    begin[17] = point.storage_volume_m3;
    begin[18] = point.initial_volume_m3;
    begin[19] = point.bulk_pressure_pa;
    const auto& observation = trial.point[ip];
    const auto material = law36_test::Values(observation.material);
    auto observed = values.observation.begin()+38*ip;
    std::copy(material.begin(),material.end(),observed);
    std::copy_n(observation.engineering_rate_per_s,6,observed+26);
    observed[32] = observation.selective_volume_increment;
    observed[33] = observation.storage_volume_factor;
    observed[34] = observation.volume_increment_m3;
    observed[35] = observation.bulk_pressure_pa;
    observed[36] = observation.unscaled_element_dt_s;
    observed[37] = observation.raw_stiffness_n_m;
  }
  const auto& global = data.global;
  std::copy_n(global.stress_pa,6,values.next.global.begin());
  values.next.global[6] = global.plastic_strain;
  values.next.global[7] = global.density_kg_m3;
  values.next.global[8] = global.internal_energy_density_j_m3;
  values.next.global[9] = global.plastic_work_j;
  values.next.global[10] = global.bulk_pressure_pa;
  for (unsigned n = 0; n < 7; ++n) {
    const auto& x = data.saved_local_position_m[n];
    values.next.saved[3*n] = x.x;
    values.next.saved[3*n+1] = x.y;
    values.next.saved[3*n+2] = x.z;
  }
  for (unsigned n = 0; n < 8; ++n) {
    const auto& force = trial.rhs_force_n[n];
    values.force[3*n] = force.x;
    values.force[3*n+1] = force.y;
    values.force[3*n+2] = force.z;
  }
  const auto& g = trial.geometry;
  unsigned i = 0;
  for (double value : g.frame.v) values.geometry[i++] = value;
  const auto put = [&](s::Vec3 v) {
    values.geometry[i++] = v.x;
    values.geometry[i++] = v.y;
    values.geometry[i++] = v.z;
  };
  for (const auto& v : g.local_position_m) put(v);
  for (const auto& v : g.local_velocity_m_s) put(v);
  for (const auto& axis : g.center_gradient_per_m) {
    for (double value : axis) values.geometry[i++] = value;
  }
  values.geometry[i++] = g.center_volume_m3;
  values.geometry[i++] = g.inverse_center_face_scale_per_m2;
  for (const auto& point : g.point) {
    for (const auto& axis : point.regular_per_m) {
      for (double value : axis) values.geometry[i++] = value;
    }
    for (const auto& channel : point.shear_per_m) {
      for (double value : channel) values.geometry[i++] = value;
    }
    for (const auto& channel : point.cross_per_m) {
      for (double value : channel) values.geometry[i++] = value;
    }
    for (double value : point.inverse_scaled_jacobian_per_m.v) values.geometry[i++] = value;
    values.geometry[i++] = point.current_volume_m3;
  }
  assert(i == values.geometry.size());
  const auto& d = trial.diagnostics;
  values.diagnostics = {d.selection_factor,d.selective_poisson_ratio,
      static_cast<double>(d.selected_point),d.minimum_unscaled_dt_s,d.raw_stiffness_n_m,
      d.internal_work_increment_j,d.plastic_work_increment_j};
  return values;
}

// Scale only same-dimensional named groups to accommodate cancellation. There
// is no absolute floor, unit conversion, source-ID or phase tolerance here.
inline bool Range(const double* a, const double* b, unsigned count, const char* name,
                  double relative = 3e-11) {
  double scale = 0;
  for (unsigned i = 0; i < count; ++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i])) {
      ADD_FAILURE() << name << " nonfinite channel " << i;
      return false;
    }
    scale = std::max({scale,std::abs(a[i]),std::abs(b[i])});
  }
  const double roundoff = 256*std::numeric_limits<double>::epsilon()*scale;
  for (unsigned i = 0; i < count; ++i) {
    if (std::abs(a[i]-b[i]) > relative*std::max(std::abs(a[i]),std::abs(b[i]))+roundoff) {
      ADD_FAILURE() << name << " channel " << i << " actual " << std::setprecision(17)
                    << a[i] << " native " << b[i] << " group scale " << scale;
      return false;
    }
  }
  return true;
}
bool Agree(const s::ForceTrial&, const NativeResult&);
} // namespace solid18_force_test
