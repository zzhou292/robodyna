// SPDX-License-Identifier: AGPL-3.0-or-later
// Complete selected CZFINTCE arithmetic, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QephHistoryData.h"
#include "QephMaterial.h"

namespace tl::fea::qeph::detail {
// Native antisymmetric contributions in slots0/1, symmetric in2/3.
// These are not nodal/world forces until the final CZPROJN operation.
struct LocalForceWork { Vec3 force[4]{}; double couple[4][2]{}; };
TL_QEPH_HD inline void ElasticForces(const GeometryWork& g,const MaterialWork& m,
    const HistoryValues& h,LocalForceWork& w) {
  const double* s=h.stress; const double* b=h.bending_stress;
  auto* f=w.force; auto& c=w.couple;
  const double x13s8=g.x13*b[2],x24s8=g.x24*b[2];
  const double y13s8=g.y13*b[2],y24s8=g.y24*b[2];
  const double s1=(g.my34*g.mx23-g.my23*g.mx34)*m.thickness;
  const double s42s=s1*s[4],s52s=s1*s[3];
  f[0].x=m.thickness*(g.y24*s[0]-g.x24*s[2]);
  f[0].y=m.thickness*(-g.x24*s[1]+g.y24*s[2]);
  f[0].z=m.thickness*(-g.x24*s[3]+g.y24*s[4]);
  c[0][0]=m.thickness2*(g.x24*b[1]-y24s8)-g.my13*f[0].z;
  c[0][1]=m.thickness2*(g.y24*b[0]-x24s8)+g.mx13*f[0].z;
  c[2][0]=-s52s; c[2][1]=s42s;
  f[1].x=m.thickness*(-g.y13*s[0]+g.x13*s[2]);
  f[1].y=m.thickness*(g.x13*s[1]-g.y13*s[2]);
  f[1].z=m.thickness*(g.x13*s[3]-g.y13*s[4]);
  c[1][0]=m.thickness2*(-g.x13*b[1]+y13s8)+g.my13*f[1].z;
  c[1][1]=m.thickness2*(-g.y13*b[0]+x13s8)-g.mx13*f[1].z;
  c[3][0]=c[2][0]; c[3][1]=c[2][1];
  const double c2=m.thickness2*g.values.effective_warpage*4.*g.values.reciprocal_area;
  f[0].x=f[0].x+c2*(g.x13*b[0]+y13s8);
  f[0].y=f[0].y+c2*(g.y13*b[1]+x13s8);
  f[1].x=f[1].x-c2*(g.x24*b[0]+y24s8);
  f[1].y=f[1].y-c2*(g.y24*b[1]+x24s8);
}
} // namespace tl::fea::qeph::detail
