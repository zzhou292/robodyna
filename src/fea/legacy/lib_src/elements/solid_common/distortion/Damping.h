// SPDX-License-Identifier: AGPL-3.0-or-later
// SFOR_VISN8, OpenRadioss a62b27e6 (C) 2026 Siemens.
#pragma once
#include "ForceTypes.h"
namespace tl::fea::solid_common::distortion::native {
// Integer OR of triggers_native_batch over the ORIGINAL native NEL packet is
// required before evaluating any row. A CUDA block is not a native batch.
TL_BRICK_HD inline Status ClassifyDamping(const Parameters& p,const ForceInput& in,
                                          DampingActivity& output) noexcept {
  if(!detail::Valid(p,in))return Status::InvalidInput;
  const Vec3 mean=detail::Mean8(in.velocity);
  const double vc2=Dot(mean,mean);
  if(!tl::math::Finite(vc2))return Status::NonfiniteResult;
  DampingActivity next{p.buckling_flag,0};
  if(!(vc2<1e-20||p.control_stiffness==0)) {
    double v2max=Dot(in.velocity[0],in.velocity[0]);
    for(unsigned n=1;n<8;++n) {
      const double v2=Dot(in.velocity[n],in.velocity[n]);
      if(!tl::math::Finite(v2))return Status::NonfiniteResult;
      v2max=::fmax(v2max,v2);
    }
    if(!tl::math::Finite(v2max))return Status::NonfiniteResult;
    if(v2max>(10.0*10.0)*vc2)next.flag=1;
    if(next.flag>0)next.triggers_native_batch=1;
  }
  output=next;
  return Status::Success;
}
namespace detail {
TL_BRICK_HD inline void Damping(const Parameters& p,const ForceInput& in,
                                 const Vec3& mean,Work& w) noexcept {
  // FOR_T is zero at the source call. Keep component/node order in work sum.
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)
    SetComponent(w.force[n],k,Component(w.force[n],k)-p.damping*
      (Component(in.velocity[n],k)-Component(mean,k)));
  w.stiffness=(1.0+2.0*p.damping_coefficient)*w.stiffness;
  double power=Component(w.force[0],0)*(Component(in.velocity[0],0)-Component(mean,0));
  for(unsigned i=1;i<24;++i) {
    const unsigned n=i/3,k=i%3;
    power=power+Component(w.force[n],k)*(Component(in.velocity[n],k)-Component(mean,k));
  }
  const double work=in.dt*power;
  w.energy=w.energy-work;w.increment=w.increment-work;
}
} // namespace detail
} // namespace tl::fea::solid_common::distortion::native
