// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SHOUR_CTL, OpenRadioss (C) 2026 Siemens, revision a62b27e6.
#pragma once
#include "Modes.h"
#include "lib_src/elements/solid_common/PhysicalHourglassModes.h"

namespace tl::fea::solid_common::controlled_hourglass {
namespace detail {
TL_BRICK_HD inline bool FiniteInput(const Input& in,const State& state) noexcept {
  if(!tl::math::Finite(in.mu_pa)||!tl::math::Finite(in.poisson_ratio)||
     !tl::math::Finite(in.density_kg_m3)||!tl::math::Finite(in.material_sound_speed_m_s)||
     !tl::math::Finite(in.dt_s)||!tl::math::Finite(in.current_volume_m3)||
     !tl::math::Finite(in.reference_volume_m3)||!tl::math::Finite(in.internal_energy_density_j_m3)||
     !tl::math::Finite(in.raw_stiffness_n_m))return false;
  for(unsigned n=0;n<8;++n)
    if(!Finite(in.local_velocity_m_s[n])||!Finite(in.incoming_local_force_n[n]))return false;
  for(const auto& row:in.projection)for(double v:row)if(!tl::math::Finite(v))return false;
  for(const auto& row:state.force_n)for(double v:row)if(!tl::math::Finite(v))return false;
  return true;
}
TL_BRICK_HD inline bool FiniteResult(const Result& value) noexcept {
  if(!tl::math::Finite(value.internal_energy_density_j_m3)||
     !tl::math::Finite(value.raw_stiffness_n_m)||!tl::math::Finite(value.work_j))return false;
  for(const auto& v:value.local_force_n)if(!Finite(v))return false;
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)
    if(!tl::math::Finite(value.proposed_state.force_n[k][h])||
       !tl::math::Finite(value.modal_velocity_m_s[k][h])||
       !tl::math::Finite(value.modal_force_n[k][h]))return false;
  return true;
}
} // namespace detail
TL_BRICK_HD inline Status EvaluateLaw42(const Input& input,const State& state,Result& output) noexcept {
  if(!detail::FiniteInput(input,state)||input.mu_pa<=0||input.density_kg_m3<=0||
     input.material_sound_speed_m_s<=0||input.current_volume_m3<=0||
     input.reference_volume_m3<=0||input.dt_s<0||input.raw_stiffness_n_m<0)
    return Status::InvalidInput;
  if(input.poisson_ratio<0||input.poisson_ratio>MaximumPoissonRatio)return Status::UnsupportedProfile;
  // HM_READ_MAT42 GS loop/PARMAT and HM_READ_MAT LAW42 PM20/22 assignments.
  double gs=0;gs=gs+input.mu_pa*2.0;
  const double e0=gs*(1.0+input.poisson_ratio);
  const double g0=.5*gs;
  const double c1=(1.0/3.0)*e0/(1.0-2.0*input.poisson_ratio);
  const double lamgt=input.material_sound_speed_m_s*input.material_sound_speed_m_s*input.density_kg_m3;
  const double lamg=c1+(4.0/3.0)*g0;
  if(!tl::math::Finite(g0)||!tl::math::Finite(c1)||!tl::math::Finite(lamg)||!tl::math::Finite(lamgt))return Status::NonfiniteResult;
  const double f_gt=::fmax(1.0,(lamgt-c1)/g0/(4.0/3.0));
  double f_et=::fmax(1.0,lamgt/lamg);
  const double sfac=::fmin(4.0,::sqrt(f_gt));
  f_et=sfac*f_et;
  const double f_sti=sfac*1.0;
  // Native bare 0.3 is default REAL, widened to WP by multiplication.
  const double stif=static_cast<double>(0.3f)*1.0*lamg;
  const double fvl=.25*(1.0/10.0)*(1.0/10.0); // DN=ZEP1, EM01.
  const double length=1.0*::pow(input.current_volume_m3,1.0/3.0);
  const double caq=stif*input.dt_s;
  const double edt=f_et*caq*length;
  const double fcl=fvl*input.density_kg_m3*input.material_sound_speed_m_s*length*length;
  if(!tl::math::Finite(f_et)||!tl::math::Finite(edt)||!tl::math::Finite(fcl))return Status::NonfiniteResult;
  Result next;
  next.raw_stiffness_n_m=f_sti*input.raw_stiffness_n_m;
  double projection[8][3];
  detail::Modes(input,projection,next.modal_velocity_m_s);
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h) {
    next.proposed_state.force_n[k][h]=state.force_n[k][h]*1.0;
    next.proposed_state.force_n[k][h]=next.proposed_state.force_n[k][h]+edt*next.modal_velocity_m_s[k][h];
    next.modal_force_n[k][h]=(next.proposed_state.force_n[k][h]+fcl*next.modal_velocity_m_s[k][h])*8.0;
  }
  detail::Forces(input,projection,next.modal_force_n,next.local_force_n);
  next.work_j=input.dt_s*ModePower(next.modal_force_n,next.modal_velocity_m_s);
  next.internal_energy_density_j_m3=input.internal_energy_density_j_m3+next.work_j/input.reference_volume_m3;
  if(!detail::FiniteResult(next))return Status::NonfiniteResult;
  output=next;
  return Status::Success;
}
} // namespace tl::fea::solid_common::controlled_hourglass
