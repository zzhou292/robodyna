// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Prepare.h"
#include "Stress.h"
namespace tl::material::law42 {
TL_LAW42_HD inline Status Update(const Parameters& p,const Input& input,Result& output) noexcept {
  Parameters checked;
  if(Prepare(p.mu_pa,p.poisson_ratio,p.density_kg_m3,p.tension_cutoff_pa,checked)!=Status::Ok||
     checked.bulk_pa!=p.bulk_pa) return Status::InvalidParameters;
  if(!tl::math::Finite(input.density_kg_m3)||input.density_kg_m3<=0||
     (input.active!=0&&input.active!=1)) return Status::InvalidInput;
  double strain[6];
  for(unsigned k=0;k<6;++k) {
    if(!tl::math::Finite(input.total_strain[k])) return Status::InvalidInput;
    strain[k]=input.total_strain[k]*(k<3?1:0.5);
  }
  if(!detail::PositiveStretchTensor(strain)) return Status::InvalidStretch;
  tl::math::SymmetricSpectrum3 spectrum;
  if(!tl::math::SymmetricEigen3(strain,spectrum)) return Status::NonfiniteResult;
  double stretch[3];
  for(unsigned k=0;k<3;++k) {
    if(spectrum.value[k]+1<=0) return Status::InvalidStretch;
    stretch[k]=::sqrt(spectrum.value[k]+1);
  }
  Result next;
  next.active=input.active;
  next.relative_volume=stretch[0]*stretch[1]*stretch[2];
  if(!tl::math::Finite(next.relative_volume)||next.relative_volume<=0)
    return Status::NonfiniteResult;
  const double gs=p.mu_pa*2;
  double pressure_factor=1;
  if(p.bulk_pa>24*gs) {
    const double nu1=40*(0.5-(3*p.bulk_pa-gs)/(6*p.bulk_pa+gs));
    const double minimum=detail::Minimum(stretch[0],detail::Minimum(stretch[1],stretch[2]));
    if(minimum<0.2) pressure_factor=detail::Maximum(1,nu1/detail::Maximum(1e-20,minimum));
  }
  const double bulk=pressure_factor*p.bulk_pa;
  const double volume_factor=::exp((-1.0/3.0)*::log(next.relative_volume));
  double power[3],derivative[3];
  for(unsigned k=0;k<3;++k) {
    power[k]=::exp(2*::log(stretch[k]*volume_factor));
    derivative[k]=p.mu_pa*power[k];
  }
  const double mean_power=(1.0/3.0)*(power[0]+power[1]+power[2]);
  double maximum_tangent=0;
  for(unsigned k=0;k<3;++k)
    maximum_tangent=detail::Maximum(maximum_tangent,gs*(power[k]+mean_power));
  // SIGEPS42's unsuffixed 0.81 is first rounded as a Fortran default REAL.
  constexpr double native_factor=static_cast<double>(0.81f);
  next.hourglass_tangent_factor=detail::Maximum(1,(native_factor*0.5/gs)*maximum_tangent);
  const double gtmax=gs*next.hourglass_tangent_factor;
  const double volumetric_derivative=bulk*(next.relative_volume-1);
  const double mean=(derivative[0]+derivative[1]+derivative[2])*(1.0/3.0);
  double principal[3];
  for(unsigned k=0;k<3;++k) {
    principal[k]=(derivative[k]-(mean-next.relative_volume*volumetric_derivative))*(1/next.relative_volume);
    const double biot=next.relative_volume*principal[k]/stretch[k];
    if(!tl::math::Finite(principal[k])||!tl::math::Finite(biot)) return Status::NonfiniteResult;
    if(biot>p.tension_cutoff_pa) next.active=0;
  }
  if(next.active==0) for(double& x:principal) x=0;
  detail::RotatePrincipal(spectrum.vectors,principal,next.stress_pa);
  next.maximum_principal_stress_pa=detail::Maximum(principal[0],detail::Maximum(principal[1],principal[2]));
  next.minimum_principal_stress_pa=detail::Minimum(principal[0],detail::Minimum(principal[1],principal[2]));
  next.sound_speed_m_s=::sqrt(((2.0/3.0)*gtmax+bulk)/input.density_kg_m3);
  for(double x:next.stress_pa) if(!tl::math::Finite(x)) return Status::NonfiniteResult;
  if(!tl::math::Finite(next.sound_speed_m_s)||!tl::math::Finite(next.hourglass_tangent_factor)||
     !tl::math::Finite(next.maximum_principal_stress_pa)||!tl::math::Finite(next.minimum_principal_stress_pa))
    return Status::NonfiniteResult;
  output=next;
  return Status::Ok;
}
} // namespace tl::material::law42
