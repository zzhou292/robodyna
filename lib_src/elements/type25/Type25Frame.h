// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete donor hashes: lib_utest/qualification/type25/native/source-manifest.json.
// GNU AGPL v3 or later; see LICENSE.md alongside.
#pragma once
#include "Type25Property.h"

#if defined(__CUDACC__)
#define TL_TYPE25_HD __host__ __device__
#else
#define TL_TYPE25_HD
#endif

namespace tl::fea::type25 {
// R4BUF3 no-N3, finite-length frame branch. The native zero/noise-length branch
// cannot establish a qualified orthonormal frame; reject that geometry.
TL_TYPE25_HD inline Status InitializeReference(SourceUnits units,const Vec3 (&position)[2],
    FrameSeed seed,Reference& output) {
  detail::Units u;
  if(!detail::ResolveUnits(units,u)||!tl::math::fixed3::Finite(position[0])||!tl::math::fixed3::Finite(position[1])||
     !tl::math::fixed3::Unit(seed.x)||!tl::math::fixed3::Unit(seed.y)||::fabs(tl::math::fixed3::Dot(seed.x,seed.y))>1e-12)return Status::InvalidInput;
  const auto chord=tl::math::fixed3::Subtract(position[1],position[0]);
  const double length=tl::math::fixed3::Norm(chord);
  if(!detail::Positive(length))return Status::DegenerateGeometry;
  if(length<=u.length_floor)return Status::DegenerateGeometry;
  auto cross=tl::math::fixed3::Cross(chord,seed.y);
  if(::sqrt(tl::math::fixed3::Dot(cross,cross)/tl::math::fixed3::Dot(seed.y,seed.y))/length<1e-5)
    cross=tl::math::fixed3::Cross(chord,seed.x);
  const auto transverse=tl::math::fixed3::Cross(cross,chord);
  const double norm=tl::math::fixed3::Norm(transverse);
  if(!detail::Positive(norm))return Status::DegenerateGeometry;
  const auto y=tl::math::fixed3::Divide(transverse,norm);
  const auto x=tl::math::fixed3::Divide(chord,length);
  if(!tl::math::fixed3::Orthonormal(tl::math::fixed3::Columns(x,y,tl::math::fixed3::Cross(x,y))))return Status::DegenerateGeometry;
  output={{position[0],position[1]},y,length};return Status::Success;
}

namespace detail {
TL_TYPE25_HD inline bool ValidReference(const Reference& r) {
  if(!tl::math::fixed3::Finite(r.position[0])||!tl::math::fixed3::Finite(r.position[1])||
     !Positive(r.length_m)||!tl::math::fixed3::Unit(r.transverse_axis))return false;
  const auto chord=tl::math::fixed3::Subtract(r.position[1],r.position[0]);
  return tl::math::fixed3::Norm(chord)==r.length_m&&
      ::fabs(tl::math::fixed3::Dot(tl::math::fixed3::Divide(chord,r.length_m),r.transverse_axis))<=1e-12;
}
TL_TYPE25_HD inline bool ValidKinematics(const EndpointKinematics& a) {
  return tl::math::fixed3::Finite(a.position)&&tl::math::fixed3::Finite(a.velocity)&&tl::math::fixed3::Finite(a.angular_velocity);
}
TL_TYPE25_HD inline bool FinishFrame(Vec3 x,Vec3 y,Vec3 z,double cy,double sz,Matrix3& out) {
  y=tl::math::fixed3::Add(tl::math::fixed3::Scale(y,cy),tl::math::fixed3::Scale(z,sz));
  y=tl::math::fixed3::Divide(y,::fmax(1e-15,tl::math::fixed3::Norm(y)));
  z=tl::math::fixed3::Cross(x,y);
  z=tl::math::fixed3::Divide(z,::fmax(1e-15,tl::math::fixed3::Norm(z)));
  const auto axes=tl::math::fixed3::Columns(x,y,z);
  if(!tl::math::fixed3::Orthonormal(axes))return false;
  out=axes;return true;
}
} // namespace detail

// R4EVEC3: current chord and chord backtracked by half dt with the actual
// midpoint velocity; previous transverse direction transported and twisted by
// the mean axial spin. The midpoint frame is used for deformation increments;
// the endpoint frame is used for force/couple scatter. No nodal quaternion fit.
TL_TYPE25_HD inline Status AdvanceFrame(SourceUnits units,Vec3 accepted_transverse,
    const EndpointKinematics (&nodes)[2],double dt,Frame& output) {
  detail::Units u;
  if(!detail::ResolveUnits(units,u)||!detail::Positive(dt)||!tl::math::fixed3::Unit(accepted_transverse)||
     !detail::ValidKinematics(nodes[0])||!detail::ValidKinematics(nodes[1]))return Status::InvalidInput;
  const double half_dt=.5*dt;
  const auto chord=tl::math::fixed3::Subtract(nodes[1].position,nodes[0].position);
  const auto middle=tl::math::fixed3::Subtract(chord,tl::math::fixed3::Scale(tl::math::fixed3::Subtract(nodes[1].velocity,nodes[0].velocity),half_dt));
  Frame next;next.length_m=tl::math::fixed3::Norm(chord);next.midpoint_length_m=tl::math::fixed3::Norm(middle);
  if(!tl::math::fixed3::Finite(next.length_m)||!tl::math::fixed3::Finite(next.midpoint_length_m))return Status::NonfiniteResult;
  if(next.length_m<=u.length_floor||next.midpoint_length_m<=u.length_floor)return Status::DegenerateGeometry;
  const auto x=tl::math::fixed3::Divide(chord,next.length_m),xm=tl::math::fixed3::Divide(middle,next.midpoint_length_m);
  const auto z=tl::math::fixed3::Cross(x,accepted_transverse),zm=tl::math::fixed3::Cross(xm,accepted_transverse);
  const auto y=tl::math::fixed3::Cross(z,x),ym=tl::math::fixed3::Cross(zm,xm);
  const double w1=tl::math::fixed3::Dot(xm,nodes[0].angular_velocity),w2=tl::math::fixed3::Dot(xm,nodes[1].angular_velocity);
  const double angle=(w1+w2)/2*half_dt;
  if(!tl::math::fixed3::Finite(angle))return Status::NonfiniteResult;
  const double c=::cos(angle),s=::sin(angle);
  if(!detail::FinishFrame(x,y,z,(2*c*c-1)/::fmax(1e-15,tl::math::fixed3::Norm(y)),
       (2*c*s)/::fmax(1e-15,tl::math::fixed3::Norm(z)),next.axes)||
     !detail::FinishFrame(xm,ym,zm,c/::fmax(1e-15,tl::math::fixed3::Norm(ym)),
       s/::fmax(1e-15,tl::math::fixed3::Norm(zm)),next.midpoint_axes))return Status::DegenerateGeometry;
  output=next;return Status::Success;
}
} // namespace tl::fea::type25

#undef TL_TYPE25_HD
