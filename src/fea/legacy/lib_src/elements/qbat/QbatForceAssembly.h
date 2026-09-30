// SPDX-License-Identifier: AGPL-3.0-or-later
// CBAFORI1/CBAFORCT/flat CBAPROJ: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatForceTypes.h"

namespace tl::fea::qbat::detail {
TL_QBAT_HD inline void PointForces(const PointDerivatives& derivatives,
    const SurfaceHistory& point,double volume,Vec3 (&force)[4]) {
  const auto& b=derivatives.membrane_b_per_m;
  const auto& s=point.force_stress_pa;
  force[0].x=force[0].x+volume*(b[0]*s[0]);
  force[0].y=force[0].y+volume*(b[4]*s[1]);
  force[2].x=force[2].x+volume*(b[2]*s[0]);
  force[2].y=force[2].y+volume*(b[6]*s[1]);
  force[1].x=force[1].x+volume*(b[1]*s[0]);
  force[1].y=force[1].y+volume*(b[5]*s[1]);
  force[3].x=-force[2].x;
  force[3].y=-force[2].y;
}
TL_QBAT_HD inline void ConstantShearForce(const Geometry& g,const HistoryValues& h,
    double initial_volume,Vec3 (&force)[4]) {
  const auto& p=g.centered_projected_position_m;
  const double x13=((p[0].x-p[2].x)*.5)*g.reciprocal_area_per_m2;
  const double x24=((p[1].x-p[3].x)*.5)*g.reciprocal_area_per_m2;
  const double y13=((p[0].y-p[2].y)*.5)*g.reciprocal_area_per_m2;
  const double y24=((p[1].y-p[3].y)*.5)*g.reciprocal_area_per_m2;
  const double thoff=initial_volume*h.force_stress_pa[2]*(h.element_active?1.:0.);
  const double sx1=-thoff*x24;
  const double sy1=thoff*y24;
  const double sx2=thoff*x13;
  const double sy2=-thoff*y13;
  force[0].x=force[0].x+sx1;
  force[0].y=force[0].y+sy1;
  force[1].x=force[1].x+sx2;
  force[1].y=force[1].y+sy2;
}
TL_QBAT_HD inline bool ProjectForces(const Geometry& g,const Vec3 (&packed)[4],
    bool active,Vec3 (&world)[4],Vec3 (&couple)[4]) {
  const auto& a=packed;
  const Vec3 force[4]{{a[0].x+a[2].x,a[0].y+a[2].y,a[0].z+a[2].z},
      {a[1].x+a[3].x,a[1].y+a[3].y,a[1].z+a[3].z},
      {-a[0].x+a[2].x,-a[0].y+a[2].y,-a[0].z+a[2].z},
      {-a[1].x+a[3].x,-a[1].y+a[3].y,-a[1].z+a[3].z}};
  const auto& q=g.frame.v;
  for (unsigned i=0;i<4;++i) {
    const auto f=force[i];
    world[i]={q[0]*f.x+q[1]*f.y+q[2]*f.z,q[3]*f.x+q[4]*f.y+q[5]*f.z,
        q[6]*f.x+q[7]*f.y+q[8]*f.z};
    // Native VM=0 for NPTT1/IDRIL0, including its signed-zero arithmetic.
    couple[i]={q[0]*0.+q[1]*0.,q[3]*0.+q[4]*0.,q[6]*0.+q[7]*0.};
    // CBAPROJ applies the final parent OFF after world projection. Earlier
    // surface caches and work remain intact when the fourth point removes it.
    const double mask=active?1.:0.;
    world[i].x*=mask; world[i].y*=mask; world[i].z*=mask;
    couple[i].x*=mask; couple[i].y*=mask; couple[i].z*=mask;
    if (!Finite(world[i]) || !Finite(couple[i])) return false;
  }
  return true;
}
} // namespace tl::fea::qbat::detail
