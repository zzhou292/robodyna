// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from OpenRadioss, Copyright (C) 2026 Siemens; see ../LICENSE.md.
// Shared original REAL4 arithmetic for Starter, fixed-ready and current NORMP.
#pragma once
#include "../GeometryTypes.h"
#include <cmath>
namespace tlfea::contact::radioss_type25::normal_math {
TL_MATH_HOST_DEVICE inline bool Finite(StoredNormal a) noexcept {
  return tl::math::Finite(double(a.x))&&tl::math::Finite(double(a.y))&&tl::math::Finite(double(a.z));
}
TL_MATH_HOST_DEVICE inline StoredNormal Add(StoredNormal a,StoredNormal b) noexcept {return {a.x+b.x,a.y+b.y,a.z+b.z};}
TL_MATH_HOST_DEVICE inline StoredNormal Subtract(StoredNormal a,StoredNormal b) noexcept {return {a.x-b.x,a.y-b.y,a.z-b.z};}
TL_MATH_HOST_DEVICE inline StoredNormal Negate(StoredNormal a) noexcept {return {-a.x,-a.y,-a.z};}
TL_MATH_HOST_DEVICE inline StoredNormal Cross(StoredNormal a,StoredNormal b) noexcept {
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
TL_MATH_HOST_DEVICE inline StoredNormal Normalize(StoredNormal a,float floor) noexcept {
  const float length=::sqrtf(a.x*a.x+a.y*a.y+a.z*a.z);
  // Same operand/tie order as std::max(floor,length), without a host-only
  // standard-library call in the shared CUDA leaf. No FMA or fast reciprocal.
  const float inverse=1.0f/(floor<length?length:floor);
  return {a.x*inverse,a.y*inverse,a.z*inverse};
}
TL_MATH_HOST_DEVICE inline bool Zero(StoredNormal a) noexcept {return a.x==0&&a.y==0&&a.z==0;}
TL_MATH_HOST_DEVICE inline constexpr float ReadyFloor() noexcept {return 1.0e-30f;}
struct PrimaryResult {bool valid=false;unsigned bad_corner=4;};
TL_MATH_HOST_DEVICE inline PrimaryResult Primary(const Vector (&points)[4],bool quad,
    float floor,StoredNormal (&normal)[4]) noexcept {
  StoredNormal corner[4];
  for(unsigned k=0;k<4;++k) {
    corner[k]={static_cast<float>(points[k].x),static_cast<float>(points[k].y),static_cast<float>(points[k].z)};
    if(!Finite(corner[k]))return {false,k};
  }
  StoredNormal center=corner[2];
  if(quad)center={static_cast<float>(0.25*(corner[0].x+corner[1].x+corner[2].x+corner[3].x)),
      static_cast<float>(0.25*(corner[0].y+corner[1].y+corner[2].y+corner[3].y)),
      static_cast<float>(0.25*(corner[0].z+corner[1].z+corner[2].z+corner[3].z))};
  StoredNormal edge[4];
  for(unsigned k=0;k<4;++k)edge[k]=Subtract(corner[k],center);
  for(unsigned k=0;k<4;++k)normal[k]=Normalize(Cross(edge[k],edge[(k+1)%4]),floor);
  if(quad) {for(const auto n:normal)if(!Finite(n))return {};}
  else if(!Finite(normal[0]))return {};
  return {true,4};
}
TL_MATH_HOST_DEVICE inline StoredNormal FreeEdge(Vector a,Vector b,StoredNormal normal,float floor) noexcept {
  // X is MYREAL8 here: subtract first, THEN assign the differences to REAL4.
  const StoredNormal delta{static_cast<float>(b.x-a.x),static_cast<float>(b.y-a.y),static_cast<float>(b.z-a.z)};
  return Normalize(Cross(delta,normal),floor);
}
} // namespace tlfea::contact::radioss_type25::normal_math
