// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected CBACOOR/CBADEF1: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "QbatChecks.h"

namespace tl::fea::qbat::detail {
TL_QBAT_HD inline Status PointGeometry(Geometry& g) {
  const auto& p=g.centered_projected_position_m;
  const double x13=(p[0].x-p[2].x)*.5;
  const double x24=(p[1].x-p[3].x)*.5;
  const double y13=(p[0].y-p[2].y)*.5;
  const double y24=(p[1].y-p[3].y)*.5;
  const double mx13=(p[0].x+p[2].x)*.5;
  const double my13=(p[0].y+p[2].y)*.5;
  const double mx23=(p[1].x+p[2].x)*.5;
  const double mx34=(p[2].x+p[3].x)*.5;
  const double my23=(p[1].y+p[2].y)*.5;
  const double my34=(p[2].y+p[3].y)*.5;
  const double inverse=g.reciprocal_area_per_m2;
  const double gamma1=-mx13*y24+my13*x24;
  const double gamma2=mx13*y13-my13*x13;
  auto& core=g.native_vcore;
  core[0]=y24*inverse;
  core[1]=-y13*inverse;
  core[2]=-x24*inverse;
  core[3]=x13*inverse;
  core[4]=gamma1*inverse;
  core[5]=gamma2*inverse;
  core[6]=mx23;
  core[7]=my23;
  core[8]=mx34;
  core[9]=my34;
  core[10]=mx13;
  core[11]=my13;
  // Native DATA PG uses an unsuffixed REAL32 literal. Do not substitute sqrt(1/3).
  constexpr double pg=static_cast<double>(.577350269189626f);
  double j1=(mx23*my13-mx13*my23)*pg;
  double j2=-(mx13*my34-mx34*my13)*pg;
  const double j0=g.area_m2*.25;
  const double signed_jacobian[4]{j0+j2-j1,j0+j2+j1,j0-j2+j1,j0-j2-j1};
  for (unsigned i=0;i<4;++i) {
    // Keep native ABS on admitted convex geometry, while excluding inverted or
    // numerically collapsed Gauss cells instead of silently accepting folding.
    if (!Positive(signed_jacobian[i])) return Status::kUnsupportedGeometry;
    g.point[i].jacobian_m2=::fabs(signed_jacobian[i]);
  }
  j1=(my23-my34)*pg;
  j2=-(my23+my34)*pg;
  g.point[0].hx_per_m=j1/g.point[0].jacobian_m2;
  g.point[1].hx_per_m=j2/g.point[1].jacobian_m2;
  g.point[2].hx_per_m=-j1/g.point[2].jacobian_m2;
  g.point[3].hx_per_m=-j2/g.point[3].jacobian_m2;
  j1=(mx34-mx23)*pg;
  j2=(mx34+mx23)*pg;
  g.point[0].hy_per_m=j1/g.point[0].jacobian_m2;
  g.point[1].hy_per_m=j2/g.point[1].jacobian_m2;
  g.point[2].hy_per_m=-j1/g.point[2].jacobian_m2;
  g.point[3].hy_per_m=-j2/g.point[3].jacobian_m2;
  for (auto& point:g.point) {
    auto& b=point.membrane_b_per_m;
    b[0]=core[0]+point.hx_per_m*core[4];
    b[1]=core[1]+point.hx_per_m*core[5];
    b[2]=point.hx_per_m*.25;
    b[3]=-b[2];
    b[4]=core[2]+point.hy_per_m*core[4];
    b[5]=core[3]+point.hy_per_m*core[5];
    b[6]=point.hy_per_m*.25;
    b[7]=-b[6];
  }
  // CBADEFSH evaluates this signed product sequence; these coefficients do not
  // claim to evaluate velocities, DT1 frame correction or generalized rates.
  g.assumed_shear_per_m[0]=core[0];
  g.assumed_shear_per_m[1]=core[1];
  g.assumed_shear_per_m[2]=core[2];
  g.assumed_shear_per_m[3]=core[3];
  return Status::kSuccess;
}
} // namespace tl::fea::qbat::detail
