// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include <vector>

namespace qbat_resident_test {
// Named active fields only. Object padding is not an accepted-state channel;
// retain double bits, including signed zero, for retry/rollback comparisons.
inline std::vector<std::uint64_t> ResultValues(const qb::BatchResult& row) {
  std::vector<std::uint64_t> values;
  values.reserve(400);
  const auto scalar=[&](double value) {
    std::uint64_t bits;
    std::memcpy(&bits,&value,sizeof(bits));
    values.push_back(bits);
  };
  const auto vector=[&](qb::Vec3 value) {
    scalar(value.x);
    scalar(value.y);
    scalar(value.z);
  };
  for(double value:StateValues(row.history)) scalar(value);
  scalar(row.stamp.time);
  values.push_back(row.stamp.sample_index);
  const auto& kinematics=row.kinematics;
  for(double value:qbat_test::Values(kinematics.geometry)) scalar(value);
  for(auto value:kinematics.corrected_velocity) vector(value);
  for(double value:kinematics.local_spin) scalar(value);
  for(const auto& point:kinematics.rate) for(double value:point) scalar(value);
  for(const auto& point:kinematics.strain_increment) for(double value:point) scalar(value);
  for(double value:kinematics.equivalent_rate_per_s) scalar(value);
  scalar(kinematics.characteristic_length_m);
  for(double value:kinematics.nodal_factor) scalar(value);
  for(const auto& point:row.point) {
    const auto& material=point.material;
    for(double value:material.history.stress) scalar(value);
    scalar(material.history.plastic_strain);
    scalar(material.history.filtered_rate_per_s);
    scalar(material.plastic_increment);
    scalar(material.tangent_ratio);
    scalar(material.elastic_thickness_strain);
    scalar(material.plastic_thickness_strain);
    scalar(material.yield_before_pa);
    scalar(material.equivalent_stress_pa);
    scalar(material.plastic_work_density);
    scalar(point.thickness_before_m);
    scalar(point.thickness_material_m);
    scalar(point.thickness_after_m);
    scalar(point.force_volume_m3);
    values.push_back(point.failed_now);
  }
  for(auto value:row.internal_force_n) vector(value);
  for(auto value:row.internal_couple_nm) vector(value);
  const auto& diagnostics=row.diagnostics;
  scalar(diagnostics.membrane_viscosity);
  scalar(diagnostics.numerical_viscosity);
  scalar(diagnostics.sound_speed_m_s);
  scalar(diagnostics.viscosity_timestep_factor);
  scalar(diagnostics.unscaled_element_dt_s);
  scalar(diagnostics.translation_stiffness_n_m);
  scalar(diagnostics.rotation_stiffness_nm);
  for(double value:diagnostics.internal_work_increment_j) scalar(value);
  scalar(diagnostics.plastic_work_increment_j);
  scalar(diagnostics.numerical_viscous_work_increment_j);
  values.push_back(diagnostics.removed_now);
  return values;
}
} // namespace qbat_resident_test
