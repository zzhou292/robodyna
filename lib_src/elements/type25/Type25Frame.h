// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete donor hashes: lib_utest/qualification/type25/native/source-manifest.json.
// GNU AGPL v3 or later; see LICENSE.md alongside.
#pragma once
#include "Type25Property.h"
#include "../spring/SpringFrame.h"

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
  spring::FrameValues next;
  const auto status=spring::AdvanceFrame(accepted_transverse,nodes,dt,u.length_floor,next);
  if(status==spring::Status::NonfiniteResult)return Status::NonfiniteResult;
  if(status!=spring::Status::Success)return Status::DegenerateGeometry;
  output={next.axes,next.midpoint_axes,next.length,next.midpoint_length};return Status::Success;
}
} // namespace tl::fea::type25

#undef TL_TYPE25_HD
