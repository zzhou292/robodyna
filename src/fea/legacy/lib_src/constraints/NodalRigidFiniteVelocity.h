// SPDX-License-Identifier: AGPL-3.0-or-later
// VELROT_EXPLICIT adapted from OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "NodalRigidGroupStepMath.h"
#if defined(__CUDACC__)
#define TL_RIGID_FINITE_HD __host__ __device__
#else
#define TL_RIGID_FINITE_HD
#endif
namespace tl::fea::rigid::two_member_detail {
TL_RIGID_FINITE_HD inline Vec3 Cross(Vec3 x,Vec3 y) {
  return {x.y*y.z-y.y*x.z,-x.x*y.z+y.x*x.z,x.x*y.y-y.x*x.y};
}
// The donor threshold is 1e-8 in its length squared. All supplied values here
// are SI; length_to_m preserves the original source-unit branch. This function
// is private arithmetic, with preflight and finite publication in its caller.
TL_RIGID_FINITE_HD inline Vec3 FiniteVelocity(Vec3 w,Vec3 arm,double dt,double length_to_m) {
  auto vs=Cross(w,arm);
  const Vec3 angle{w.x*dt,w.y*dt,w.z*dt};
  const double angle2=angle.x*angle.x+angle.y*angle.y+angle.z*angle.z;
  const double vs2=(vs.x*vs.x+vs.y*vs.y+vs.z*vs.z)*dt*dt;
  // Do not let an overflowed norm normalize a finite vector to zero and then
  // manufacture a finite apparent displacement. The caller rejects this value.
  if(!tl::math::Finite(angle2)||!tl::math::Finite(vs2)) return {angle2,vs2,0};
  const double threshold=(1e-8*length_to_m)*length_to_m;
  if(angle2>1e-6&&vs2>threshold) {
    const double a=::sqrt(angle2),iz=1/(a>1e-20?a:1e-20),ix=dt/::sqrt(vs2);
    const Vec3 z{iz*angle.x,iz*angle.y,iz*angle.z},x{ix*vs.x,ix*vs.y,ix*vs.z};
    const auto y=Cross(z,x);
    const Vec3 local{x.x*arm.x+x.y*arm.y+x.z*arm.z,
      y.x*arm.x+y.y*arm.y+y.z*arm.z,z.x*arm.x+z.y*arm.y+z.z*arm.z};
    const double c=::cos(a),s=::sin(a);
    // Retain the full native products, including zero entries and reduction order.
    const Vec3 rotated{c*local.x+(-s)*local.y+0*local.z,
      s*local.x+c*local.y+0*local.z,0*local.x+0*local.y+1*local.z};
    const Vec3 world{x.x*rotated.x+y.x*rotated.y+z.x*rotated.z,
      x.y*rotated.x+y.y*rotated.y+z.y*rotated.z,
      x.z*rotated.x+y.z*rotated.y+z.z*rotated.z};
    vs={(world.x-arm.x)/dt,(world.y-arm.y)/dt,(world.z-arm.z)/dt};
  }
  return vs;
}
} // namespace tl::fea::rigid::two_member_detail
#undef TL_RIGID_FINITE_HD
