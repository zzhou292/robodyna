// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Engine CLSKEW3 IREP0 and CZCORC1 current geometry, selected explicit path.
// This is not the starter CLSKEW3 normalization. See the source manifest.
#pragma once
#include "QephGeometryWork.h"
#include <cfloat>

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline Status CurrentFrame(const Vec3 (&x)[4],Matrix3& f,double& area) {
  // Retain original sum/difference order; replacing this with initial edge
  // differences changes the binary64 current-frame contract under translation.
  const double rx=x[1].x+x[2].x-x[0].x-x[3].x,sx=x[2].x+x[3].x-x[0].x-x[1].x;
  const double ry=x[1].y+x[2].y-x[0].y-x[3].y,sy=x[2].y+x[3].y-x[0].y-x[1].y;
  const double rz=x[1].z+x[2].z-x[0].z-x[3].z,sz=x[2].z+x[3].z-x[0].z-x[1].z;
  double e3x=ry*sz-rz*sy,e3y=rz*sx-rx*sz,e3z=rx*sy-ry*sx;
  double det=::sqrt(e3x*e3x+e3y*e3y+e3z*e3z);
  if (!tl::math::Finite(det)) return Status::kNonfiniteResult;
  if (!(det>1e-20*(1+64*DBL_EPSILON))) return Status::kUnsupportedGeometry;
  det=::fmax(1e-20,det);
  const double cc=::fmax(1/det,1e-20);
  e3x=e3x*cc; e3y=e3y*cc; e3z=e3z*cc;
  const double c1c1=rx*rx+ry*ry+rz*rz,c2c2=sx*sx+sy*sy+sz*sz;
  double c1_1=0,c2_1=0;
  if (c1c1!=0) { c2_1=::sqrt(c2c2/::fmax(1e-20,c1c1)); c1_1=1; }
  else if(c2c2!=0) { c2_1=1; c1_1=::sqrt(c1c1/::fmax(1e-20,c2c2)); }
  double e1x=rx*c2_1+(sy*e3z-sz*e3y)*c1_1;
  double e1y=ry*c2_1+(sz*e3x-sx*e3z)*c1_1;
  double e1z=rz*c2_1+(sx*e3y-sy*e3x)*c1_1;
  double c1=::sqrt(e1x*e1x+e1y*e1y+e1z*e1z);
  if(c1!=0) c1=1/::fmax(1e-20,c1);
  e1x=e1x*c1; e1y=e1y*c1; e1z=e1z*c1;
  const double e2x=e3y*e1z-e3z*e1y,e2y=e3z*e1x-e3x*e1z,e2z=e3x*e1y-e3y*e1x;
  f={{e1x,e2x,e3x,e1y,e2y,e3y,e1z,e2z,e3z}};
  area=.25*det;
  if (!Proper(f)||!Positive(area)) return Status::kNonfiniteResult;
  return ConvexProjection(x,f)?Status::kSuccess:Status::kUnsupportedGeometry;
}

TL_QEPH_HD inline Status CurrentGeometry(const PrescribedInterval& interval,GeometryWork& g) {
  const auto& x=interval.position_endpoint; auto& o=g.values;
  const auto status=CurrentFrame(x,o.frame,o.area);
  if(status!=Status::kSuccess) return status;
  o.reciprocal_area=::fmax(1/o.area,1e-20);
  const Vec3 centroid{.25*(x[2].x+x[3].x+x[0].x+x[1].x),
                      .25*(x[2].y+x[3].y+x[0].y+x[1].y),
                      .25*(x[2].z+x[3].z+x[0].z+x[1].z)};
  const auto p1=Project(o.frame,Difference(x[0],centroid));
  const auto p2=Project(o.frame,Difference(x[1],x[0]));
  const auto p3=Project(o.frame,Difference(x[2],x[0]));
  const auto p4=Project(o.frame,Difference(x[3],x[0]));
  o.effective_warpage=p1.z; o.raw_warpage_abs=::fabs(p1.z);
  const double cx=.25*(p2.x+p3.x+p4.x),cy=.25*(p2.y+p3.y+p4.y);
  auto& p=o.local_position;
  p[0]={-cx,-cy,0}; p[1]={p2.x-cx,p2.y-cy,0};
  p[2]={p3.x-cx,p3.y-cy,0}; p[3]={p4.x-cx,p4.y-cy,0};
  g.x13=(p[0].x-p[2].x)*.5; g.x24=(p[1].x-p[3].x)*.5;
  g.y13=(p[0].y-p[2].y)*.5; g.y24=(p[1].y-p[3].y)*.5;
  g.mx13=(p[0].x+p[2].x)*.5; g.mx23=(p[1].x+p[2].x)*.5; g.mx34=(p[2].x+p[3].x)*.5;
  g.my13=(p[0].y+p[2].y)*.5; g.my23=(p[1].y+p[2].y)*.5; g.my34=(p[2].y+p[3].y)*.5;
  g.py1=-g.x24; g.px2=-g.y13; g.py2=g.x13;
  g.l13=g.x13*g.x13+g.y13*g.y13; g.l24=g.x24*g.x24+g.y24*g.y24;
  double c1=p[1].x*p[3].y-p[1].y*p[3].x,c2=p[0].x*p[2].y-p[0].y*p[2].x;
  const double hs=::fmax(::fabs(c1),::fabs(c2))*o.reciprocal_area;
  const double rx=p2.x+p3.x-p4.x,ry=p2.y+p3.y-p4.y;
  const double sx=-p2.x+p3.x+p4.x,sy=-p2.y+p3.y+p4.y;
  c1=::sqrt(rx*rx+ry*ry); c2=::sqrt(sx*sx+sy*sy);
  double s1=.25*(::fmax(c1,c2)/::fmin(c1,c2)-1);
  const double fac1=::fmin(.5,s1)+1;
  double fac2=4*o.area/(c1*c2);
  // These four unsuffixed native literals are default REAL32 even with
  // MYREAL8 variables. Preserve their promoted values, not binary64 decimal
  // literals; Q3b's retained first run isolated this characteristic-length
  // mismatch. Other active raw literals are exactly representable integers.
  fac2=static_cast<double>(3.413f)*::fmax(0.,fac2-static_cast<double>(.7071f));
  fac2=static_cast<double>(.78f)+static_cast<double>(.22f)*fac2*fac2*fac2;
  const double faci=2*fac1*fac2,ll=::fmax(g.l13,g.l24);
  g.lm=.5*(g.l13+g.l24);
  o.nodal_factors[0]=::sqrt(g.l24/ll); o.nodal_factors[1]=::sqrt(g.l13/ll);
  s1=::sqrt(faci*(1.25+hs)*ll); s1=::fmax(s1,1e-10);
  o.characteristic_length=o.area/s1;
  if (!Positive(o.reciprocal_area)||!Positive(o.characteristic_length)||!Positive(g.lm)||
      !Positive(o.nodal_factors[0])||!Positive(o.nodal_factors[1])) return Status::kNonfiniteResult;
  return Status::kSuccess;
}
} // namespace tl::fea::qeph::detail
