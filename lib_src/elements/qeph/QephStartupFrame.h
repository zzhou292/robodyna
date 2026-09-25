// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete selected CNEVECI and starter CLSKEW3 IREP0 arithmetic, one cell.
// Source ranges and expression-preserving transformations are recorded in
// lib_utest/qualification/qeph/source-manifest.json; LICENSE.md is alongside.
#pragma once

#include "QephData.h"
#include "lib_src/math/Quaternion.h"  // Shared binary64 finite predicate only.
#include <cfloat>
#include <cmath>

#if defined(__CUDACC__)
#define TL_QEPH_STARTUP_HD __host__ __device__
#else
#define TL_QEPH_STARTUP_HD
#endif

namespace tl::fea::qeph::detail {
TL_QEPH_STARTUP_HD inline bool Positive(double value) {
  return tl::math::Finite(value)&&value>0;
}
TL_QEPH_STARTUP_HD inline bool ValidProjectionLength(double value) {
  // Both directions and the area conversion must be representable. Actual
  // consumed packet conversions are checked again before their publication.
  if(!Positive(value)) return false;
  const double reciprocal=1/value;
  return Positive(reciprocal)&&Positive(value*value)&&Positive(reciprocal*reciprocal);
}
TL_QEPH_STARTUP_HD inline bool Finite(Vec3 value) {
  return tl::math::Finite(value.x)&&tl::math::Finite(value.y)&&tl::math::Finite(value.z);
}
TL_QEPH_STARTUP_HD inline bool Proper(const Matrix3& frame) {
  for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) {
    const double dot=frame.v[i]*frame.v[j]+frame.v[3+i]*frame.v[3+j]+frame.v[6+i]*frame.v[6+j];
    if (!tl::math::Finite(dot)||::fabs(dot-(i==j?1.:0.))>1e-10) return false;
  }
  const double cx=frame.v[3]*frame.v[7]-frame.v[6]*frame.v[4];
  const double cy=frame.v[6]*frame.v[1]-frame.v[0]*frame.v[7];
  const double cz=frame.v[0]*frame.v[4]-frame.v[3]*frame.v[1];
  return cx*frame.v[2]+cy*frame.v[5]+cz*frame.v[8]>0;
}

TL_QEPH_STARTUP_HD inline Status StartupFrame(const Vec3 x[4],Matrix3& frame,double& area) {
  // CNEVECI: preserve differences and R/S expression order. X21 is unused by
  // the selected source; its dead store alone is omitted in this scalar port.
  const double x31=x[2].x-x[0].x,y31=x[2].y-x[0].y,z31=x[2].z-x[0].z;
  const double x42=x[3].x-x[1].x,y42=x[3].y-x[1].y,z42=x[3].z-x[1].z;
  const double rx=x31-x42,ry=y31-y42,rz=z31-z42;
  const double sx=x31+x42,sy=y31+y42,sz=z31+z42;
  // Starter CLSKEW3 IREP0; no rsqrt, FMA or alternate IREP frame is substituted.
  double e3x=ry*sz-rz*sy,e3y=rz*sx-rx*sz,e3z=rx*sy-ry*sx;
  double determinant=::sqrt(e3x*e3x+e3y*e3y+e3z*e3z);
  if (!tl::math::Finite(determinant)) return Status::kNonfiniteResult;
  // Exclude a 64epsilon band around the source floor in addition to DET>EM20.
  // This is a conservative measured-arithmetic domain, not an exact geometry
  // certificate or equivalence to the host oracle's extended-precision guard.
  if (!(determinant>1e-20*(1+64*DBL_EPSILON))) return Status::kUnsupportedGeometry;
  determinant=::fmax(1e-20,determinant);  // Original MAX retained on admitted domain.
  const double cc=1/determinant;
  e3x=e3x*cc; e3y=e3y*cc; e3z=e3z*cc;
  double c1=::sqrt(rx*rx+ry*ry+rz*rz);
  const double c2=::sqrt(sx*sx+sy*sy+sz*sz);
  double e1x=rx*c2+(sy*e3z-sz*e3y)*c1;
  double e1y=ry*c2+(sz*e3x-sx*e3z)*c1;
  double e1z=rz*c2+(sx*e3y-sy*e3x)*c1;
  c1=::sqrt(e1x*e1x+e1y*e1y+e1z*e1z);
  if (c1!=0) c1=1/c1;
  e1x=e1x*c1; e1y=e1y*c1; e1z=e1z*c1;
  const double e2x=e3y*e1z-e3z*e1y;
  const double e2y=e3z*e1x-e3x*e1z;
  const double e2z=e3x*e1y-e3y*e1x;
  const Matrix3 candidate{{e1x,e2x,e3x,e1y,e2y,e3y,e1z,e2z,e3z}};
  if (!Proper(candidate)) return Status::kNonfiniteResult;
  frame=candidate; area=.25*determinant;
  return Status::kSuccess;
}

TL_QEPH_STARTUP_HD inline bool ConvexProjection(const Vec3 x[4],const Matrix3& frame) {
  Vec3 p[4]{}; double scale=0;
  for (unsigned i=1;i<4;++i) {
    p[i]={x[i].x-x[0].x,x[i].y-x[0].y,x[i].z-x[0].z};
    const double length=::sqrt(p[i].x*p[i].x+p[i].y*p[i].y+p[i].z*p[i].z);
    if (!tl::math::Finite(length)) return false;
    scale=::fmax(scale,length);
  }
  if (!Positive(scale)) return false;
  const double inverse=1/scale;
  for (auto& point:p) { point.x*=inverse; point.y*=inverse; point.z*=inverse; }
  for (unsigned i=0;i<4;++i) {
    const auto& a=p[i]; const auto& b=p[(i+1)%4]; const auto& c=p[(i+2)%4];
    const double ux=b.x-a.x,uy=b.y-a.y,uz=b.z-a.z;
    const double vx=c.x-b.x,vy=c.y-b.y,vz=c.z-b.z;
    const double turn=(uy*vz-uz*vy)*frame.v[2]+(uz*vx-ux*vz)*frame.v[5]+(ux*vy-uy*vx)*frame.v[8];
    if (!(turn>128*DBL_EPSILON)) return false;
  }
  return true;
}
}  // namespace tl::fea::qeph::detail
#undef TL_QEPH_STARTUP_HD
