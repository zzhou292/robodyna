// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid_common/distortion/UnitResponse.h"
extern "C" void distortion_force_native(int,const double*,const int*,const double*,const double*,const double*,const double*,double*,int*);
namespace distortion_force_test {
inline tl::fea::solid_common::distortion::ForceResult NativePrepared(
    const tl::fea::solid_common::distortion::PreparedForceValues& value) {
  namespace d=tl::fea::solid_common::distortion;
  namespace b=tl::fea::solid_common;
  const auto& q=value.parameters;const auto& in=value.input;
  double p[10]{q.control_stiffness,q.damping,q.length,q.damping_coefficient,q.quadratic_limit};
  int flags[2]{q.buckling_flag},contacts[2]{};
  double x[48]{},v[48]{},base[52]{},result[52]{},work[52]{};
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k) {
    x[3*n+k]=b::Component(in.position[n],k);v[3*n+k]=b::Component(in.velocity[n],k);
    base[3*n+k]=b::Component(in.incoming_force[n],k);
  }
  base[24]=in.raw_stiffness;base[25]=in.distortion_energy;
  distortion_force_native(1,p,flags,x,v,base,&in.dt,result,contacts);
  base[25]=0;int ignored[2];distortion_force_native(1,p,flags,x,v,base,&in.dt,work,ignored);
  const double force=value.units.mass_kg*value.units.length_m/(value.units.time_s*value.units.time_s);
  const double energy=force*value.units.length_m,stiffness=value.units.mass_kg/(value.units.time_s*value.units.time_s);
  d::ForceResult out;
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)b::SetComponent(out.force_n[n],k,result[3*n+k]*force);
  out.raw_stiffness_n_m=result[24]*stiffness;out.distortion_energy_j=result[25]*energy;
  out.distortion_work_increment_j=work[25]*energy;out.center_contacts=contacts[0];out.corner_contacts=contacts[1];return out;
}
} // namespace distortion_force_test
