// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cassert>
namespace rear_force_test {
NativeResult Values(const law::ForceTrial& trial) {
  NativeResult values;
  values.status = 0;
  const auto& data = trial.proposed_history.data();
  for (unsigned ip = 0; ip < 8; ++ip) {
    const auto& p = data.point[ip];
    auto state = values.next.point.begin()+20*ip;
    std::copy_n(p.material.stress_pa,6,state);
    std::copy_n(p.material.engineering_strain,6,state+6);
    state[12] = p.material.plastic_strain;
    state[13] = p.material.filtered_rate_per_s;
    values.next.cursor[ip] = p.material.curve_cursor;
    state[14] = p.energy_density_j_m3;
    state[15] = p.plastic_work_j;
    state[16] = p.density_kg_m3;
    state[17] = p.storage_volume_m3;
    state[18] = p.initial_volume_m3;
    state[19] = p.bulk_pressure_pa;
    const auto& o = trial.point[ip];
    const auto material = law44_solid_test::Values(o.material);
    auto observed = values.observation.begin()+38*ip;
    std::copy_n(material.begin(),19,observed);
    observed[19] = o.plastic_work_increment_j/trial.geometry.point[ip].current_volume_m3;
    observed[20] = p.energy_density_j_m3;
    observed[21] = p.plastic_work_j;
    observed[22] = o.internal_work_j;
    observed[23] = o.plastic_work_increment_j;
    observed[24] = o.relative_density;
    observed[25] = o.average_volume_m3;
    std::copy_n(o.engineering_rate_per_s,6,observed+26);
    observed[32] = o.selective_volume_increment;
    observed[33] = o.storage_volume_factor;
    observed[34] = o.volume_increment_m3;
    observed[35] = o.bulk_pressure_pa;
    observed[36] = o.unscaled_element_dt_s;
    observed[37] = o.raw_stiffness_n_m;
  }
  const auto& gbl = data.global;
  std::copy_n(gbl.stress_pa,6,values.next.global.begin());
  values.next.global[6] = gbl.plastic_strain;
  values.next.global[7] = gbl.density_kg_m3;
  values.next.global[8] = gbl.energy_density_j_m3;
  values.next.global[9] = gbl.plastic_work_j;
  values.next.global[10] = gbl.bulk_pressure_pa;
  values.next.global[11] = gbl.filtered_rate_per_s;
  for (unsigned n = 0; n < 7; ++n) {
    const auto& x = data.saved_local_position_m[n];
    values.next.saved[3*n] = x.x;
    values.next.saved[3*n+1] = x.y;
    values.next.saved[3*n+2] = x.z;
  }
  for (unsigned n = 0; n < 8; ++n) {
    const auto& f = trial.rhs_force_n[n];
    values.force[3*n] = f.x;
    values.force[3*n+1] = f.y;
    values.force[3*n+2] = f.z;
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
  values.diagnostics = {double(d.native_degeneracy),double(d.caller_degeneracy),
      d.center_divergence_per_s,d.mean_pressure_pa,d.minimum_unscaled_dt_s,
      d.raw_stiffness_n_m,d.internal_work_increment_j,d.plastic_work_increment_j};
  return values;
}
}
