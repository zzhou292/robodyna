// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "UnitTypes.h"
#include "BeforeDistortion.h"
namespace tl::fea::solid24::controlled_hourglass::units_detail {
using Factors=tlfea::contact::radioss_type25::coefficient_detail::UnitFactors;
TL_BRICK_HD inline bool Make(UnitScale units,Factors& f) noexcept {
  if(!((units.length_m==1&&units.mass_kg==1&&units.time_s==1)||
       (units.length_m==.001&&units.mass_kg==1000&&units.time_s==1)))return false;
  return tlfea::contact::radioss_type25::coefficient_detail::Make(units,f);
}
TL_BRICK_HD inline Vec3 Multiply(Vec3 v,double f)noexcept{return {v.x*f,v.y*f,v.z*f};}
TL_BRICK_HD inline Vec3 Divide(Vec3 v,double f)noexcept{return {v.x/f,v.y/f,v.z/f};}
TL_BRICK_HD inline tl::material::law42::CallerHistory ToNative(
    const tl::material::law42::CallerHistory& value,const Factors& f) noexcept {
  auto r=value;for(double& v:r.stress_pa)v/=f.pressure;
  r.density_kg_m3/=f.base.mass/f.volume;r.internal_energy_density_j_m3/=f.pressure;
  r.bulk_pressure_pa/=f.pressure;return r;
}
TL_BRICK_HD inline void HistoryToSi(tl::material::law42::CallerHistory& r,const Factors& f)noexcept {
  for(double& v:r.stress_pa)v*=f.pressure;r.density_kg_m3*=f.base.mass/f.volume;
  r.internal_energy_density_j_m3*=f.pressure;r.bulk_pressure_pa*=f.pressure;
}
TL_BRICK_HD inline void MaterialToSi(tl::material::law42::CallerResult& r,const Factors& f)noexcept {
  HistoryToSi(r.history,f);for(double& v:r.point.stress_pa)v*=f.pressure;
  r.point.maximum_principal_stress_pa*=f.pressure;r.point.minimum_principal_stress_pa*=f.pressure;
  r.point.sound_speed_m_s*=f.base.velocity;r.point.material_viscosity_pa_s*=f.pressure*f.base.time;
  r.volume_increment_m3*=f.volume;r.average_volume_m3*=f.volume;r.internal_work_j*=f.base.energy;
  r.unscaled_element_dt_s*=f.base.time;r.raw_stiffness_n_m*=f.base.stiffness;
}
TL_BRICK_HD inline void GeometryToSi(ForceGeometry& g,const Factors& f)noexcept {
  for(auto& v:g.current.local_position_m)v=Multiply(v,f.base.length);
  for(auto& v:g.local_velocity_m_s)v=Multiply(v,f.base.velocity);
  g.current.volume_m3*=f.volume;g.current.characteristic_length_m*=f.base.length;
  for(auto& row:g.derivative_per_m)for(double& v:row)v/=f.base.length;
  for(double& v:g.jacobian_diagonal_m)v*=f.base.length;
  for(double& v:g.engineering_rate_per_s)v/=f.base.time;
}
TL_BRICK_HD inline void StageToSi(BeforeDistortionResult& r,const Factors& f)noexcept {
  auto& h=r.hourglass;
  for(auto& row:h.proposed_state.force_n)for(double& v:row)v*=f.base.force;
  for(auto& v:h.local_force_n)v=Multiply(v,f.base.force);
  h.internal_energy_density_j_m3*=f.pressure;h.raw_stiffness_n_m*=f.base.stiffness;h.work_j*=f.base.energy;
  for(auto& row:h.modal_velocity_m_s)for(double& v:row)v*=f.base.velocity;
  for(auto& row:h.modal_force_n)for(double& v:row)v*=f.base.force;
  for(auto& v:r.local_force_after_material_n)v=Multiply(v,f.base.force);
  for(auto& v:r.world_native_force_before_distortion_n)v=Multiply(v,f.base.force);
  for(auto& v:r.world_force_before_distortion_n)v=Multiply(v,f.base.force);
  HistoryToSi(r.proposed_values.material,f);
  for(auto& row:r.proposed_values.controlled_hourglass.force_n)for(double& v:row)v*=f.base.force;
  r.raw_stiffness_before_distortion_n_m*=f.base.stiffness;
}
TL_BRICK_HD inline bool Finite(const WorkingResult& r)noexcept {
  using tl::math::Finite;const auto& g=r.geometry;const auto& m=r.material;
  for(double v:g.current.frame.v)if(!Finite(v))return false;
  for(const auto& v:g.current.local_position_m)if(!solid_common::Finite(v))return false;
  for(const auto& v:g.local_velocity_m_s)if(!solid_common::Finite(v))return false;
  for(const auto& row:g.derivative_per_m)for(double v:row)if(!Finite(v))return false;
  for(const auto& row:g.hourglass_projection)for(double v:row)if(!Finite(v))return false;
  for(double v:g.jacobian_diagonal_m)if(!Finite(v))return false;
  for(double v:g.material_displacement_gradient)if(!Finite(v))return false;
  for(double v:g.engineering_rate_per_s)if(!Finite(v))return false;
  if(!Finite(g.current.volume_m3)||!Finite(g.current.characteristic_length_m))return false;
  for(double v:m.history.stress_pa)if(!Finite(v))return false;
  for(double v:m.point.stress_pa)if(!Finite(v))return false;
  for(double v:m.total_strain)if(!Finite(v))return false;
  const double a[]{m.history.density_kg_m3,m.history.internal_energy_density_j_m3,m.history.bulk_pressure_pa,
    m.point.maximum_principal_stress_pa,m.point.minimum_principal_stress_pa,m.point.active,
    m.point.relative_volume,m.point.sound_speed_m_s,m.point.hourglass_tangent_factor,m.point.material_viscosity_pa_s,
    m.volume_increment_m3,m.average_volume_m3,m.internal_work_j,m.unscaled_element_dt_s,m.raw_stiffness_n_m};
  for(double v:a)if(!Finite(v))return false;
  if(!hg::detail::FiniteResult(r.stage.hourglass)||!Finite(r.stage.raw_stiffness_before_distortion_n_m))return false;
  for(const auto& v:r.stage.local_force_after_material_n)if(!solid_common::Finite(v))return false;
  for(const auto& v:r.stage.world_native_force_before_distortion_n)if(!solid_common::Finite(v))return false;
  for(const auto& v:r.stage.world_force_before_distortion_n)if(!solid_common::Finite(v))return false;
  return true;
}
} // namespace tl::fea::solid24::controlled_hourglass::units_detail
