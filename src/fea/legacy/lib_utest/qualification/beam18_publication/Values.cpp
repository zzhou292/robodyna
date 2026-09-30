// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace beam18_publication_test {
std::vector<std::uint64_t> BeamValues(const std::vector<b::Result>& values) {
  std::vector<std::uint64_t> output;
  auto scalar=[&](double v) { output.push_back(existing::Bits(v)); };
  auto vector=[&](b::Vec3 v) { scalar(v.x); scalar(v.y); scalar(v.z); };
  for (const auto& v:values) {
    output.push_back(v.stamp.sample_index); scalar(v.stamp.time_s);
    for (unsigned p=0;p<4;++p) {
      const auto& h=v.history.point[p]; const auto& r=v.point[p];
      for (double s:h.stress_pa) scalar(s);
      for (double e:v.history.total_strain[p]) scalar(e);
      scalar(h.plastic_strain); output.push_back(h.curve_cursor);
      for (double s:r.history.stress_pa) scalar(s);
      scalar(r.history.plastic_strain); output.push_back(r.history.curve_cursor);
      scalar(r.plastic_increment); scalar(r.yield_stress_pa); scalar(r.tangent_factor);
    }
    vector(v.history.section_seed); vector(v.history.section_force_n); vector(v.history.section_moment_nm);
    scalar(v.history.filtered_neutral_rate_per_s);
    for (double e:v.history.internal_energy_j) scalar(e);
    scalar(v.history.plastic_work_j);
    for (auto f:v.rhs_force_n) vector(f);
    for (auto m:v.rhs_couple_nm) vector(m);
    for (auto axis:v.geometry.axis) vector(axis);
    scalar(v.geometry.length_m);
    const auto& r=v.rate;
    for (double value:{r.axial,r.shear_y,r.shear_z,r.curvature_x,r.curvature_y,r.curvature_z}) scalar(value);
    const auto& d=v.diagnostics;
    scalar(d.translation_stiffness_n_m); scalar(d.rotation_stiffness_nm); scalar(d.minimum_unscaled_dt_s);
    for (double e:d.internal_work_increment_j) scalar(e);
    scalar(d.plastic_work_increment_j); vector(d.damped_section_force_n); vector(d.damped_section_moment_nm);
  }
  return output;
}
} // namespace beam18_publication_test
