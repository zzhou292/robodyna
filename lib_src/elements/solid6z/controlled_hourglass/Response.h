// SPDX-License-Identifier: AGPL-3.0-or-later
// S6ZFINT3 -> S6CHOUR_CTL -> S6ZRROTA3; native geometric distortion disabled.
#pragma once
#include "Projection.h"
#include "lib_src/elements/solid6z/Solid6zForceResultants.h"
#include "lib_src/elements/solid6z/Solid6zForceChecks.h"
#include "lib_src/elements/solid_common/controlled_hourglass/Response.h"
namespace tl::fea::solid6z::controlled_hourglass {
TL_BRICK_HD inline Status Evaluate(const Reference& reference,const Material& material,
    const CurrentGeometry& geometry,const tl::material::law42::CallerResult& point,
    double dt,const hg::State& accepted,Result& output)noexcept {
  if(!reference.prepared()||material.poisson_ratio<0||material.poisson_ratio>hg::MaximumPoissonRatio)
    return Status::UnsupportedProfile;
  Material checked;if(tl::material::law42::Prepare(material.mu_pa,material.poisson_ratio,material.density_kg_m3,
      material.tension_cutoff_pa,checked)!=tl::material::law42::Status::Ok||!force_detail::Same(checked,material)||
      material.density_kg_m3!=reference.input().density_kg_m3||point.point.active!=1)return Status::InvalidInput;
  Result next;auto status=detail::Projection(geometry.local_position_m,next.native_projection);
  if(status!=Status::Success)return status;
  if(!force_detail::MaterialForces(geometry,point,next.material_local_force_n))return Status::NonfiniteResult;
  hg::Input input;input.mu_pa=material.mu_pa;input.poisson_ratio=material.poisson_ratio;
  input.density_kg_m3=point.history.density_kg_m3;input.material_sound_speed_m_s=point.point.sound_speed_m_s;
  input.current_volume_m3=geometry.current_volume_m3;input.reference_volume_m3=reference.geometry().volume_m3;
  input.dt_s=dt;input.internal_energy_density_j_m3=point.history.internal_energy_density_j_m3;
  input.raw_stiffness_n_m=point.raw_stiffness_n_m;
  constexpr unsigned expand[]{0,1,2,2,3,4,5,5},first[]{0,1,2,4,5,6};
  for(unsigned n=0;n<8;++n)input.local_velocity_m_s[n]=geometry.local_velocity_m_s[expand[n]];
  for(unsigned n=0;n<6;++n)input.incoming_local_force_n[first[n]]=next.material_local_force_n[n];
  for(unsigned n=0;n<4;++n)for(unsigned h=0;h<3;++h)input.projection[n][h]=next.native_projection[n][h];
  const auto hg_status=hg::EvaluateLaw42(input,accepted,next.expanded_hourglass);
  if(hg_status==hg::Status::UnsupportedProfile)return Status::UnsupportedProfile;
  if(hg_status==hg::Status::InvalidInput)return Status::InvalidInput;
  if(hg_status!=hg::Status::Success)return Status::NonfiniteResult;
  const auto& f=next.expanded_hourglass.local_force_n;
  for(unsigned k=0;k<3;++k){
    using solid_common::Component;using solid_common::SetComponent;
    SetComponent(next.local_force_n[0],k,Component(f[0],k));SetComponent(next.local_force_n[1],k,Component(f[1],k));
    SetComponent(next.local_force_n[2],k,Component(f[2],k)+Component(f[3],k));
    SetComponent(next.local_force_n[3],k,Component(f[4],k));SetComponent(next.local_force_n[4],k,Component(f[5],k));
    SetComponent(next.local_force_n[5],k,Component(f[6],k)+Component(f[7],k));
  }
  if(!force_detail::Project(reference,geometry.frame,next.local_force_n,next.world_force_n))return Status::NonfiniteResult;
  for(unsigned n=0;n<6;++n)next.world_native_force_n[n]=next.world_force_n[reference.source_slot(n)];
  next.proposed_values.material=point.history;
  next.proposed_values.material.internal_energy_density_j_m3=next.expanded_hourglass.internal_energy_density_j_m3;
  next.proposed_values.controlled_hourglass=next.expanded_hourglass.proposed_state;
  next.raw_stin_n_m=next.expanded_hourglass.raw_stiffness_n_m;
  output=next;return Status::Success;
}
} // namespace tl::fea::solid6z::controlled_hourglass
