// SPDX-License-Identifier: AGPL-3.0-or-later
// Identical penalty/work/scatter expressions in SFOR_N2S4 and SFOR_NS2S4.
#pragma once
#include "ForceTypes.h"
namespace tl::fea::solid_common::distortion::native::detail {
TL_BRICK_HD inline void Penalty(const Parameters& p,const ForceInput& in,
    const unsigned (&face)[4],Vec3 secondary_velocity,Vec3 normal,double penetration,
    double penref,const double (&h)[4],Vec3& secondary_force,Work& w) noexcept {
  const double ratio=penetration/penref;
  const double pendr=ratio*ratio;
  const double fac=::fmin(p.quadratic_limit,100.0*pendr);
  const double fn=(fac+1.0)*p.control_stiffness*penetration;
  const double fkt=1.0+3.0*fac;
  w.stiffness=::fmax(w.stiffness,fkt*p.control_stiffness);
  Vec3 relative;
  for(unsigned k=0;k<3;++k)
    SetComponent(relative,k,Component(secondary_velocity,k)-h[0]*Component(in.velocity[face[0]],k)
      -h[1]*Component(in.velocity[face[1]],k)-h[2]*Component(in.velocity[face[2]],k)
      -h[3]*Component(in.velocity[face[3]],k));
  const double dn=Dot(normal,relative)*in.dt;
  const double work=fn*dn;w.energy=w.energy-work;w.increment=w.increment-work;
  const Vec3 force{normal.x*fn,normal.y*fn,normal.z*fn};
  secondary_force.x=secondary_force.x-force.x;
  secondary_force.y=secondary_force.y-force.y;
  secondary_force.z=secondary_force.z-force.z;
  for(unsigned n=0;n<4;++n) {
    auto& f=w.force[face[n]];
    f.x=f.x+force.x*h[n];f.y=f.y+force.y*h[n];f.z=f.z+force.z*h[n];
  }
}
} // namespace tl::fea::solid_common::distortion::native::detail
