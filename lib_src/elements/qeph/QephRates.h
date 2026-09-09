// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete active CZDEF IVECTOR0 arithmetic, including warped curvature terms.
#pragma once
#include "QephGeometryWork.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline void ComputeRates(GeometryWork& g) {
  auto& o=g.values; auto& vdef=o.regular_rate; auto& vhg=o.hourglass_rate;
  const auto& r=o.projected_omega; const double aa=o.reciprocal_area;
  const double r13x=(r[0]-r[4])*aa,r24x=(r[2]-r[6])*aa;
  const double rsomx=(r[6]+r[4]+r[0]+r[2])*aa,rhix=(r[0]-r[2]+r[4]-r[6])*.25;
  const double r13y=(r[1]-r[5])*aa,r24y=(r[3]-r[7])*aa;
  const double rsomy=(r[7]+r[5]+r[1]+r[3])*aa,rhiy=(r[1]-r[3]+r[5]-r[7])*.25;
  vdef[0]=g.y24*g.v13.x-g.y13*g.v24.x;
  vdef[1]=-g.x24*g.v13.y+g.x13*g.v24.y;
  const double bxv2=g.y24*g.v13.y-g.y13*g.v24.y,byv1=-g.x24*g.v13.x+g.x13*g.v24.x;
  vdef[2]=bxv2+byv1;
  vdef[5]=g.y24*r13y-g.y13*r24y; vdef[6]=g.x24*r13x-g.x13*r24x;
  const double bxr1=g.y13*r24x-g.y24*r13x,byr2=-g.x24*r13y+g.x13*r24y;
  vdef[7]=bxr1+byr2;
  const double bcxy=o.area*.25;
  const double bcx=g.v13.z-g.my13*r13x+g.mx13*r13y;
  const double bcy=g.v24.z+g.my13*r24x-g.mx13*r24y;
  vdef[3]=g.y24*bcx-g.y13*bcy+bcxy*rsomy;
  vdef[4]=g.x13*bcy-g.x24*bcx-bcxy*rsomx;
  vhg[0]=g.vhi.x-g.mx13*vdef[0]-g.my13*byv1;
  vhg[1]=g.vhi.y-g.mx13*bxv2-g.my13*vdef[1];
  vhg[2]=rhiy-g.mx13*vdef[5]-g.my13*byr2;
  vhg[3]=-rhix-g.mx13*bxr1-g.my13*vdef[6];
  vhg[4]=(g.vhi.z*4.-(g.my13*rsomx-g.my23*(r13x+r24x)+g.mx23*(r13y+r24y)-g.mx13*rsomy)*o.area)*4;
  vhg[5]=(g.vhi.z*4.-(g.my13*rsomx-g.my34*(r13x-r24x)+g.mx34*(r13y-r24y)-g.mx13*rsomy)*o.area)*4;
  vhg[0]=vhg[0]+(g.y24*g.v13.z-g.y13*g.v24.z)*o.effective_warpage;
  vhg[1]=vhg[1]+(-g.x24*g.v13.z+g.x13*g.v24.z)*o.effective_warpage;
  const double deta=o.effective_warpage*4*aa;
  vdef[5]=vdef[5]+(g.x13*g.v13.x-g.x24*g.v24.x)*deta;
  vdef[6]=vdef[6]+(g.y13*g.v13.y-g.y24*g.v24.y)*deta;
  vdef[7]=vdef[7]+(g.x13*g.v13.y-g.x24*g.v24.y+g.y13*g.v13.x-g.y24*g.v24.x)*deta;
  // OFFG=1 is fixed by the validated active-cell geometry path. Native OFF=1;
  // the inactive/negative-OFF force branch is outside this operation's domain.
}
} // namespace tl::fea::qeph::detail
