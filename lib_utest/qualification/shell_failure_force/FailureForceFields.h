#pragma once
#include "FailureForceFixture.h"

namespace failure_force_test {
inline void Append(std::vector<double>& v,const sec::ShellLayeredJ2History& h) {
  for(const auto& p:h.point) {Append(v,p.stress);Append(v,p.plastic_strain);Append(v,p.filtered_rate_per_s);}
}
inline void Append(std::vector<double>& v,const sec::ShellLayeredJ2Diagnostics& d) {
  Append(v,d.plastic_work_density_increment);Append(v,d.maximum_plastic_strain);Append(v,d.mean_plastic_strain);
  Append(v,d.minimum_tangent_ratio);Append(v,d.mean_tangent_ratio);Append(v,d.mean_yield_before_pa);Append(v,d.last_point_yield_before_pa);
}
inline std::vector<double> Values(const sec::ShellLayeredJ2FailureResult& r) {
  std::vector<double> v;Append(v,r.history.saved);
  for(const auto& p:r.history.failure) {Append(v,p.damage);Append(v,p.failure_time_s);Append(v,p.point_active);}
  for(const auto& p:r.history.current_force_point)Append(v,p.stress);
  Append(v,r.history.element_active);Append(v,r.current.history);Append(v,r.current.material_stress);
  Append(v,r.current.bending_stress);Append(v,r.current.reported_thickness);Append(v,r.current.diagnostics);
  Append(v,r.constitutive_increment);Append(v,r.caller_failure_increment);Append(v,r.removed_now);return v;
}
inline void Append(std::vector<double>& v,const q::Kinematics& k) {
  Append(v,k.frame.v);Append(v,k.area);Append(v,k.reciprocal_area);Append(v,k.characteristic_length);Append(v,k.nodal_factors);
  Append(v,k.raw_warpage_abs);Append(v,k.effective_warpage);Append(v,k.planar);Append(v,k.local_position);Append(v,k.local_normals);
  Append(v,k.projection_inverse);Append(v,k.projection_columns);Append(v,k.projected_omega);Append(v,k.regular_rate);Append(v,k.hourglass_rate);
  Append(v,k.base_time);Append(v,k.dt);Append(v,k.sample_index);
}
inline void Append(std::vector<double>& v,const t::Kinematics& k) {
  Append(v,k.frame.v);Append(v,k.local_position);Append(v,k.derivative);Append(v,k.raw_rate);Append(v,k.normalized_rate);
  Append(v,k.corrected_velocity_difference);Append(v,k.area);Append(v,k.characteristic_length);Append(v,k.area_scale);
  Append(v,k.base_time);Append(v,k.position_time);Append(v,k.velocity_time);Append(v,k.dt);Append(v,k.sample_index);Append(v,k.valid);
}
template<class Trial> std::vector<double> ForceValues(const Trial& t) {
  auto v=Values(t.proposed_history.data());Append(v,t.kinematics);Append(v,t.internal_force);Append(v,t.internal_couple);
  Append(v,t.proposed_history.stamp().time);Append(v,t.proposed_history.stamp().sample_index);return v;
}
} // namespace failure_force_test
