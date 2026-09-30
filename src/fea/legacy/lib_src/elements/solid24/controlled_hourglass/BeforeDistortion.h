// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SZHOUR_CTL -> SFINT3 -> SRROTA3, a62b27e6 OpenRadioss.
#pragma once
#include "Types.h"
#include "lib_src/elements/solid24/Solid24ForceHistory.h"
#include "lib_src/elements/solid24/Solid24ForceResultants.h"
#include "lib_src/elements/solid_common/controlled_hourglass/Response.h"

namespace tl::fea::solid24::controlled_hourglass {
TL_BRICK_HD inline ForceStatus EvaluateBeforeDistortion(const Reference& reference,
    const Material& material,const ForceGeometry& geometry,
    const tl::material::law42::CallerResult& material_result,double dt_s,
    const hg::State& accepted_controlled_hourglass,BeforeDistortionResult& output) noexcept {
  if(!reference.prepared()||!reference.reference_jacobian()||
     reference.input().profile.working_length!=WorkingLengthUnit::Metre||
     material.poisson_ratio<0||material.poisson_ratio>hg::MaximumPoissonRatio)
    return ForceStatus::UnsupportedProfile;
  if(!force_detail::ValidMaterial(reference,material)||material_result.point.active!=1)
    return ForceStatus::InvalidInput;
  for(double value:geometry.current.frame.v)if(!tl::math::Finite(value))return ForceStatus::InvalidInput;
  for(const auto& row:geometry.derivative_per_m)for(double value:row)
    if(!tl::math::Finite(value))return ForceStatus::InvalidInput;
  for(double value:material_result.history.stress_pa)
    if(!tl::math::Finite(value))return ForceStatus::InvalidInput;
  if(!tl::math::Finite(material_result.history.bulk_pressure_pa))return ForceStatus::InvalidInput;
  hg::Input input;
  input.mu_pa=material.mu_pa;input.poisson_ratio=material.poisson_ratio;
  input.density_kg_m3=material_result.history.density_kg_m3;
  input.material_sound_speed_m_s=material_result.point.sound_speed_m_s;
  input.dt_s=dt_s;input.current_volume_m3=geometry.current.volume_m3;
  input.reference_volume_m3=reference.geometry().volume_m3;
  for(unsigned n=0;n<8;++n)input.local_velocity_m_s[n]=geometry.local_velocity_m_s[n];
  for(unsigned n=0;n<4;++n)for(unsigned h=0;h<3;++h)
    input.projection[n][h]=geometry.hourglass_projection[n][h];
  input.internal_energy_density_j_m3=material_result.history.internal_energy_density_j_m3;
  input.raw_stiffness_n_m=material_result.raw_stiffness_n_m;
  BeforeDistortionResult next;
  const auto status=hg::EvaluateLaw42(input,accepted_controlled_hourglass,next.hourglass);
  if(status==hg::Status::UnsupportedProfile)return ForceStatus::UnsupportedProfile;
  if(status==hg::Status::InvalidInput)return ForceStatus::InvalidInput;
  if(status!=hg::Status::Success)return ForceStatus::NonfiniteResult;
  for(unsigned n=0;n<8;++n)next.local_force_after_material_n[n]=next.hourglass.local_force_n[n];
  force_detail::MaterialForces(geometry,material_result.history,next.local_force_after_material_n);
  const auto rotation=force_detail::RotateAndMapForces(reference,geometry.current.frame,
      next.local_force_after_material_n,next.world_force_before_distortion_n);
  if(rotation!=ForceStatus::Success)return rotation;
  for(unsigned n=0;n<8;++n) {
    if(!tl::fea::solid_common::Finite(next.local_force_after_material_n[n]))return ForceStatus::NonfiniteResult;
    next.world_native_force_before_distortion_n[n]=next.world_force_before_distortion_n[reference.source_slot(n)];
  }
  next.proposed_values.material=material_result.history;
  next.proposed_values.material.internal_energy_density_j_m3=next.hourglass.internal_energy_density_j_m3;
  next.proposed_values.controlled_hourglass=next.hourglass.proposed_state;
  next.raw_stiffness_before_distortion_n_m=next.hourglass.raw_stiffness_n_m;
  output=next;return ForceStatus::Success;
}
} // namespace tl::fea::solid24::controlled_hourglass
