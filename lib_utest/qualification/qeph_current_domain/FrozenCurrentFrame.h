// SPDX-License-Identifier: AGPL-3.0-or-later
// Frozen pre-extension current-frame body from37f9e911. Qualification only.
#pragma once
#include "lib_src/elements/qeph/QephCurrentFrame.h"
namespace tl::qualification::qeph_current_domain {
using namespace tl::fea::qeph;
using namespace tl::fea::qeph::detail;
TL_QEPH_HD inline Status FrozenCurrentFrame(const Vec3 (&x)[4],Matrix3& f,double& area) {
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

}
