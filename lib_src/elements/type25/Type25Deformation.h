// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete donor hashes: lib_utest/qualification/type25/native/source-manifest.json.
// GNU AGPL v3 or later; see LICENSE.md alongside.
#pragma once
#include "Type25Frame.h"

#if defined(__CUDACC__)
#define TL_TYPE25_HD __host__ __device__
#else
#define TL_TYPE25_HD
#endif

namespace tl::fea::type25::detail {
TL_TYPE25_HD inline bool ValidHistory(const History& h) {
  if(!tl::math::Unit(h.transverse_axis)||!tl::math::Finite(h.displacement_m)||!tl::math::Finite(h.rotation_rad)||
     !tl::math::Finite(h.local_force_N)||!tl::math::Finite(h.local_couple_Nm)||
     !Nonnegative(h.failure_criterion)||h.failure_criterion>1||(!h.active&&h.failure_criterion!=1))return false;
  for(unsigned i=0;i<4;++i)if(!tl::math::Finite(h.internal_work_J[i]))return false;
  return true;
}
TL_TYPE25_HD inline double RotationIncrement(double old,Vec3 dw,Vec3 axis) {
  // R6DEF3 preserves the left-associated old + three products.
  return old+dw.x*axis.x+dw.y*axis.y+dw.z*axis.z;
}
TL_TYPE25_HD inline Status Deformation(const Units& u,const Reference& reference,const History& accepted,
    const Frame& frame,const EndpointKinematics (&nodes)[2],double dt,History& next) {
  const auto ym=tl::math::Column(frame.midpoint_axes,1),zm=tl::math::Column(frame.midpoint_axes,2);
  const auto dv=tl::math::Subtract(nodes[1].velocity,nodes[0].velocity);
  const auto sumw=tl::math::Add(nodes[1].angular_velocity,nodes[0].angular_velocity);
  const double half_dt=.5*dt;
  const double epxy=tl::math::Dot(dv,ym)*half_dt,epxz=tl::math::Dot(dv,zm)*half_dt;
  const double denominator=::fmax(frame.midpoint_length_m,u.shear_length_floor);
  const double ry=half_dt*tl::math::Dot(sumw,ym)+2*::atan(epxz/denominator);
  const double rz=half_dt*tl::math::Dot(sumw,zm)-2*::atan(epxy/denominator);
  next.displacement_m={frame.length_m-reference.length_m,
      accepted.displacement_m.y-rz*frame.midpoint_length_m,
      accepted.displacement_m.z+ry*frame.midpoint_length_m};
  const auto dw=tl::math::Scale(tl::math::Subtract(nodes[1].angular_velocity,nodes[0].angular_velocity),dt);
  next.rotation_rad={RotationIncrement(accepted.rotation_rad.x,dw,tl::math::Column(frame.midpoint_axes,0)),
      RotationIncrement(accepted.rotation_rad.y,dw,ym),RotationIncrement(accepted.rotation_rad.z,dw,zm)};
  next.transverse_axis=tl::math::Column(frame.axes,1);
  return tl::math::Finite(next.displacement_m)&&tl::math::Finite(next.rotation_rad)?Status::Success:Status::NonfiniteResult;
}
} // namespace tl::fea::type25::detail

#undef TL_TYPE25_HD
