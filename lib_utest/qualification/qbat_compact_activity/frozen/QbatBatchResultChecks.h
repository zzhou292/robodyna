// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatBatchArena.h"
#include "QbatForceChecks.h"

namespace tl::fea::qbat::batch_detail {
TL_QBAT_HD inline bool ValidBool(const bool& value) noexcept {
  const auto* bytes=reinterpret_cast<const unsigned char*>(&value);
  return sizeof(bool)==1&&bytes[0]<=1;
}
TL_QBAT_HD inline bool FinitePoint(const PointResult& value) noexcept {
  const double scalars[]{value.history.plastic_strain,value.history.filtered_rate_per_s,
      value.plastic_increment,value.tangent_ratio,value.elastic_thickness_strain,
      value.plastic_thickness_strain,value.yield_before_pa,value.equivalent_stress_pa,
      value.plastic_work_density};
  return detail::FiniteValues(value.history.stress)&&detail::FiniteValues(scalars);
}
TL_QBAT_HD inline bool FiniteKinematics(const Kinematics& value,bool completed) noexcept {
  if(completed&&!detail::Finite(value.geometry)) return false;
  // Startup has a known zero cache; geometry has not been force-evaluated.
  if(!completed) {
    for(double x:value.geometry.frame.v) if(!tl::math::Finite(x)) return false;
    const double scalars[]{value.geometry.area_m2,value.geometry.reciprocal_area_per_m2,
        value.geometry.actual_warpage_m};
    if(!detail::FiniteValues(scalars)) return false;
    for(const auto& x:value.geometry.centered_projected_position_m) if(!detail::Finite(x)) return false;
    if(!detail::FiniteValues(value.geometry.native_vcore)||
        !detail::FiniteValues(value.geometry.assumed_shear_per_m)) return false;
    for(const auto& point:value.geometry.point) {
      const double scalars[]{point.jacobian_m2,point.hx_per_m,point.hy_per_m};
      if(!detail::FiniteValues(scalars)||!detail::FiniteValues(point.membrane_b_per_m)) return false;
    }
  }
  for(const auto& x:value.corrected_velocity) if(!detail::Finite(x)) return false;
  if(!detail::FiniteValues(value.local_spin)||!detail::FiniteValues(value.equivalent_rate_per_s)||
      !detail::FiniteValues(value.nodal_factor)||!tl::math::Finite(value.characteristic_length_m)) return false;
  for(unsigned point=0;point<4;++point) {
    if(!detail::FiniteValues(value.rate[point])||!detail::FiniteValues(value.strain_increment[point])||
        value.equivalent_rate_per_s[point]<0) return false;
  }
  return !completed||detail::Positive(value.characteristic_length_m);
}
// This validates every named numerical field before staged host publication.
// Padding is deliberately not a semantic channel. Immutable curve/reference
// inputs come from the retained complete catalog, never device readback bytes.
TL_QBAT_HD inline bool ValidResult(const BatchResult& value,const Material& material,
    double time,std::uint64_t epoch) noexcept {
  if(!ValidBool(value.history.element_active)||!ValidBool(value.diagnostics.removed_now)) return false;
  for(unsigned point=0;point<4;++point) {
    const auto& saved=value.history.point[point];
    if(!ValidBool(saved.surface_active)||!ValidBool(saved.failure.point_active)||
        !ValidBool(value.point[point].failed_now)) return false;
  }
  if(value.stamp.time!=time||value.stamp.sample_index!=epoch||
      !detail::ValidHistory(value.history,material,time)||!FiniteKinematics(value.kinematics,epoch!=0)) return false;
  const auto& diagnostics=value.diagnostics;
  const double scalars[]{diagnostics.membrane_viscosity,diagnostics.numerical_viscosity,
      diagnostics.sound_speed_m_s,diagnostics.viscosity_timestep_factor,
      diagnostics.unscaled_element_dt_s,diagnostics.translation_stiffness_n_m,
      diagnostics.rotation_stiffness_nm,diagnostics.plastic_work_increment_j,
      diagnostics.numerical_viscous_work_increment_j};
  if(!detail::FiniteValues(scalars)||!detail::FiniteValues(diagnostics.internal_work_increment_j)||
      diagnostics.rotation_stiffness_nm!=0) return false;
  if(epoch&&(!detail::Positive(diagnostics.sound_speed_m_s)||
      !detail::Positive(diagnostics.viscosity_timestep_factor)||
      !detail::Positive(diagnostics.unscaled_element_dt_s)||diagnostics.translation_stiffness_n_m<0)) return false;
  for(unsigned point=0;point<4;++point) {
    const auto& current=value.point[point];
    const double thickness[]{current.thickness_before_m,current.thickness_material_m,
        current.thickness_after_m,current.force_volume_m3};
    if(!FinitePoint(current.material)||!detail::FiniteValues(thickness)||
        !detail::Finite(value.internal_force_n[point])||!detail::Finite(value.internal_couple_nm[point])) return false;
    const auto couple=value.internal_couple_nm[point];
    if(couple.x!=0||couple.y!=0||couple.z!=0) return false;
    if(!value.history.element_active) {
      const auto force=value.internal_force_n[point];
      if(force.x!=0||force.y!=0||force.z!=0) return false;
    }
    if(epoch) {
      for(double x:thickness) if(x<=0) return false;
      if(current.material.history.plastic_strain!=value.history.point[point].material.plastic_strain||
          current.material.history.filtered_rate_per_s!=value.history.point[point].material.filtered_rate_per_s||
          (point&&current.thickness_before_m!=value.point[point-1].thickness_after_m)) return false;
    }
  }
  return !epoch||value.point[3].thickness_after_m==value.history.thickness_m;
}
} // namespace tl::fea::qbat::batch_detail
