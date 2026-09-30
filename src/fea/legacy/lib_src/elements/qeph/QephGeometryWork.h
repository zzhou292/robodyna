// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Private coherent CZCORC1/CZCORP5/CZDEF workspace; see source manifest.
#pragma once
#include "QephKinematicsData.h"
#include "QephStartupFrame.h"

#if defined(__CUDACC__)
#define TL_QEPH_HD __host__ __device__
#else
#define TL_QEPH_HD
#endif

namespace tl::fea::qeph::detail {
// No accepted history or clock. Later force/material leaves consume these
// exact intermediates rather than reconstructing them from public output.
struct GeometryWork {
  Kinematics values;
  Vec3 v13{},v24{},vhi{};
  double x13=0,x24=0,y13=0,y24=0;
  double mx13=0,mx23=0,mx34=0,my13=0,my23=0,my34=0;
  double l13=0,l24=0,lm=0;
  double py1=0,px2=0,py2=0;
};
TL_QEPH_HD inline Vec3 Project(const Matrix3& f,Vec3 x) {
  return {f.v[0]*x.x+f.v[3]*x.y+f.v[6]*x.z,
          f.v[1]*x.x+f.v[4]*x.y+f.v[7]*x.z,
          f.v[2]*x.x+f.v[5]*x.y+f.v[8]*x.z};
}
TL_QEPH_HD inline double Dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
TL_QEPH_HD inline Vec3 Difference(Vec3 a,Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
TL_QEPH_HD inline double Component(Vec3 v,unsigned axis) { return axis==0?v.x:axis==1?v.y:v.z; }
TL_QEPH_HD inline double FourProducts(const Vec3* a,const Vec3* b,unsigned i,unsigned j) {
  // Used only for explicit fixed-size projection matrix sums, keeping the
  // native left-to-right order. No pointer arithmetic across Vec3 members.
  return Component(a[0],i)*Component(b[0],j)+Component(a[1],i)*Component(b[1],j)+
         Component(a[2],i)*Component(b[2],j)+Component(a[3],i)*Component(b[3],j);
}
} // namespace tl::fea::qeph::detail
