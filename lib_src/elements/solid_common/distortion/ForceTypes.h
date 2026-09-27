// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/elements/solid_common/BrickFrame.h"
namespace tl::fea::solid_common::distortion::native {
// All numeric values use the explicitly selected native length/mass/time system.
// FLD is the native formula operand, not an assumed SI damping coefficient.
struct Parameters {
  double length=0,damping=0,control_stiffness=0,damping_coefficient=0,quadratic_limit=0;
  int buckling_flag=0;
};
struct ForceInput {
  Vec3 position[8]{},velocity[8]{},incoming_force[8]{}; // Native slots, world frame.
  double dt=0,raw_stiffness=0,distortion_energy=0;
};
struct DampingActivity {int flag=0;int triggers_native_batch=0;};
struct ForceResult {
  Vec3 force[8]{};
  double raw_stiffness=0,distortion_energy=0,distortion_work_increment=0;
  int damping_applied=0,center_contacts=0,corner_contacts=0;
};
namespace detail {
struct Work {
  Vec3 force[8]{},center_force{};
  double stiffness=0,energy=0,increment=0;
  int center_contacts=0,corner_contacts=0;
};
TL_BRICK_HD inline Vec3 Difference(const Vec3& a,const Vec3& b) noexcept {
  return {a.x-b.x,a.y-b.y,a.z-b.z};
}
TL_BRICK_HD inline double Distance1(const Vec3& a,const Vec3& b) noexcept {
  return ::fabs(a.x-b.x)+::fabs(a.y-b.y)+::fabs(a.z-b.z);
}
TL_BRICK_HD inline Vec3 Mean8(const Vec3 (&x)[8]) noexcept {
  Vec3 r;
  for(unsigned k=0;k<3;++k) {
    double sum=Component(x[0],k);
    for(unsigned n=1;n<8;++n)sum=sum+Component(x[n],k);
    SetComponent(r,k,(1.0/8.0)*sum);
  }
  return r;
}
TL_BRICK_HD inline bool Valid(const Parameters& p,const ForceInput& in) noexcept {
  if(!tl::math::Finite(p.length)||p.length<=0||
     !tl::math::Finite(p.damping)||p.damping<0||
     !tl::math::Finite(p.control_stiffness)||p.control_stiffness<0||
     !tl::math::Finite(p.damping_coefficient)||p.damping_coefficient<0||
     !tl::math::Finite(p.quadratic_limit)||p.quadratic_limit<0||
     (p.buckling_flag!=0&&p.buckling_flag!=1)||!tl::math::Finite(in.dt)||in.dt<0||
     !tl::math::Finite(in.raw_stiffness)||in.raw_stiffness<0||
     !tl::math::Finite(in.distortion_energy))return false;
  for(unsigned n=0;n<8;++n)
    if(!Finite(in.position[n])||!Finite(in.velocity[n])||!Finite(in.incoming_force[n]))return false;
  return true;
}
TL_BRICK_HD inline bool FiniteWork(const Work& w) noexcept {
  if(!tl::math::Finite(w.stiffness)||!tl::math::Finite(w.energy)||
     !tl::math::Finite(w.increment)||!Finite(w.center_force))return false;
  for(const auto& f:w.force)if(!Finite(f))return false;
  return true;
}
} // namespace detail
} // namespace tl::fea::solid_common::distortion::native
