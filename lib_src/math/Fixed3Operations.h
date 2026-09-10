// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixed3.h"
#include "Quaternion.h"

#if defined(__CUDACC__)
#define TL_FIXED3_HD __host__ __device__
#else
#define TL_FIXED3_HD
#endif

namespace tl::math {
// Shared scalar order used by TL's existing rigid and shell frame helpers.
TL_FIXED3_HD inline bool Finite(Vec3 v) { return Finite(v.x)&&Finite(v.y)&&Finite(v.z); }
TL_FIXED3_HD inline Vec3 Add(Vec3 a,Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
TL_FIXED3_HD inline Vec3 Subtract(Vec3 a,Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
TL_FIXED3_HD inline Vec3 Scale(Vec3 v,double s) { return {v.x*s,v.y*s,v.z*s}; }
TL_FIXED3_HD inline double Dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
TL_FIXED3_HD inline Vec3 Cross(Vec3 a,Vec3 b) {
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
TL_FIXED3_HD inline double Norm(Vec3 v) { return ::sqrt(Dot(v,v)); }
TL_FIXED3_HD inline Vec3 Divide(Vec3 v,double s) { return {v.x/s,v.y/s,v.z/s}; }
TL_FIXED3_HD inline Vec3 Column(const Matrix3& m,unsigned a) { return {m.v[a],m.v[3+a],m.v[6+a]}; }
TL_FIXED3_HD inline Matrix3 Columns(Vec3 x,Vec3 y,Vec3 z) {
  return {{x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z}};
}
TL_FIXED3_HD inline Vec3 ToLocal(const Matrix3& m,Vec3 v) {
  return {Dot(Column(m,0),v),Dot(Column(m,1),v),Dot(Column(m,2),v)};
}
TL_FIXED3_HD inline Vec3 ToWorld(const Matrix3& m,Vec3 v) {
  return {m.v[0]*v.x+m.v[1]*v.y+m.v[2]*v.z,
          m.v[3]*v.x+m.v[4]*v.y+m.v[5]*v.z,
          m.v[6]*v.x+m.v[7]*v.y+m.v[8]*v.z};
}
TL_FIXED3_HD inline bool Unit(Vec3 v,double tolerance=1e-12) {
  return Finite(v)&&::fabs(Dot(v,v)-1)<=tolerance;
}
TL_FIXED3_HD inline bool Orthonormal(const Matrix3& m,double tolerance=1e-12) {
  const auto x=Column(m,0),y=Column(m,1),z=Column(m,2);
  return Unit(x,tolerance)&&Unit(y,tolerance)&&Unit(z,tolerance)&&
    ::fabs(Dot(x,y))<=tolerance&&::fabs(Dot(x,z))<=tolerance&&
    ::fabs(Dot(y,z))<=tolerance&&::fabs(Dot(Cross(x,y),z)-1)<=tolerance;
}
} // namespace tl::math

#undef TL_FIXED3_HD
