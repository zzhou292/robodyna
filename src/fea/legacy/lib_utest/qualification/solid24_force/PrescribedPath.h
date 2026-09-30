// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"

namespace heph_test {
inline s::Vec3 Scale(s::Vec3 x,double factor) {
  return {factor*x.x,factor*x.y,factor*x.z};
}
inline s::Vec3 Subtract(s::Vec3 a,s::Vec3 b) {
  return {a.x-b.x,a.y-b.y,a.z-b.z};
}
inline s::PrescribedInterval Path(const s::Reference& reference,unsigned step,double dt=1e-6) {
  s::PrescribedInterval interval;
  interval.base_time_s=(step-1)*dt;interval.dt_s=dt;interval.sample_index=step;
  const auto center=Scale(b::Add(reference.input().position_m[0],reference.input().position_m[6]),.5);
  const double length=reference.geometry().characteristic_length_m;
  const auto position=[&](unsigned source,unsigned index) {
    const double phase=double(index)*.23;
    const double deformation=.018*std::sin(phase);
    const double spin=double(index)*.015,c=std::cos(spin),sn=std::sin(spin);
    const auto p=Subtract(reference.input().position_m[source],center);
    double x=(1+deformation)*p.x+.4*deformation*p.y;
    double y=(1-.2*deformation)*p.y-.3*deformation*p.z;
    const double z=(1+.3*deformation)*p.z;
    constexpr double mode[]{1,1,-1,-1,-1,-1,1,1};
    unsigned native=0;
    while(reference.source_slot(native)!=source)++native;
    x+=.001*length*std::sin(phase)*mode[native];
    y+=.0007*length*std::sin(.7*phase)*mode[(native+1)%8];
    return s::Vec3{center.x+c*x-sn*y+.0001*length*index,
                   center.y+sn*x+c*y-.0002*length*index,center.z+z};
  };
  for(unsigned n=0;n<8;++n) {
    interval.position_m[n]=position(n,step);
    interval.velocity_m_s[n]=Scale(Subtract(interval.position_m[n],position(n,step-1)),1/dt);
  }
  return interval;
}
}
