// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid6z/Solid6zForce.h"
#include <array>
namespace solid6z_force_test {
struct Values {
  double geometry[98]{},material[33]{},history[21]{},forces[54]{},stabilization[28]{};
};
TL_BRICK_HD inline Values Pack(const tl::fea::solid6z::ForceTrial& r) {
  Values v;
  const auto& g = r.geometry;
  unsigned i = 0;
  for (unsigned row = 0; row < 3; ++row) {
    for (unsigned col = 0; col < 3; ++col) v.geometry[i++] = g.frame.v[3*row+col];
  }
  for (const auto& x : g.local_position_m) { v.geometry[i++] = x.x; v.geometry[i++] = x.y; v.geometry[i++] = x.z; }
  for (const auto& x : g.local_velocity_m_s) { v.geometry[i++] = x.x; v.geometry[i++] = x.y; v.geometry[i++] = x.z; }
  for (const auto& p : g.point_gradient_per_m) for (double x : p) v.geometry[i++] = x;
  for (double x : g.world_displacement_gradient) v.geometry[i++] = x;
  for (double x : g.material_displacement_gradient) v.geometry[i++] = x;
  for (double x : g.velocity_gradient_per_s) v.geometry[i++] = x;
  for (double x : g.engineering_rate_per_s) v.geometry[i++] = x;
  v.geometry[i++] = g.current_volume_m3;
  v.geometry[i++] = g.characteristic_length_m;
  const auto& m = r.material;
  i = 0;
  for (double x : m.history.stress_pa) v.material[i++] = x;
  v.material[i++] = m.history.density_kg_m3;
  v.material[i++] = m.history.internal_energy_density_j_m3;
  v.material[i++] = m.history.bulk_pressure_pa;
  for (double x : m.point.stress_pa) v.material[i++] = x;
  v.material[i++] = m.point.maximum_principal_stress_pa;
  v.material[i++] = m.point.minimum_principal_stress_pa;
  v.material[i++] = m.point.active;
  v.material[i++] = m.point.relative_volume;
  v.material[i++] = m.point.sound_speed_m_s;
  v.material[i++] = m.point.hourglass_tangent_factor;
  v.material[i++] = m.point.material_viscosity_pa_s;
  for (double x : m.total_strain) v.material[i++] = x;
  v.material[i++] = m.volume_increment_m3;
  v.material[i++] = m.average_volume_m3;
  v.material[i++] = m.internal_work_j;
  v.material[i++] = m.unscaled_element_dt_s;
  v.material[i++] = m.raw_stiffness_n_m;
  const auto& h = r.proposed_history.data();
  i = 0;
  for (double x : h.material.stress_pa) v.history[i++] = x;
  v.history[i++] = h.material.density_kg_m3;
  v.history[i++] = h.material.internal_energy_density_j_m3;
  v.history[i++] = h.material.bulk_pressure_pa;
  for (const auto& row : h.hourglass_stress_pa) for (double x : row) v.history[i++] = x;
  i = 0;
  for (const auto& f : r.material_local_force_n) { v.forces[i++] = f.x; v.forces[i++] = f.y; v.forces[i++] = f.z; }
  for (const auto& f : r.stabilized_local_force_n) { v.forces[i++] = f.x; v.forces[i++] = f.y; v.forces[i++] = f.z; }
  for (const auto& f : r.rhs_force_n) { v.forces[i++] = f.x; v.forces[i++] = f.y; v.forces[i++] = f.z; }
  i = 0;
  for (const auto& row : r.stabilization.modal_velocity_m_s) for (double x : row) v.stabilization[i++] = x;
  for (const auto& row : r.stabilization.modal_force_n) for (double x : row) v.stabilization[i++] = x;
  v.stabilization[i++] = r.stabilization.effective_shear_modulus_pa;
  v.stabilization[i++] = r.stabilization.damping_kg_m_s;
  v.stabilization[i++] = m.history.internal_energy_density_j_m3+r.stabilization.first_work_j/
      ::fmax(1e-20,r.proposed_history.reference().geometry().volume_m3);
  v.stabilization[i++] = h.material.internal_energy_density_j_m3;
  return v;
}
} // namespace solid6z_force_test
