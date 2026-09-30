// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Selected CZCORC1 world-rate gather and explicit midpoint correction.
#pragma once
#include "QephGeometryWork.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline void GatherRates(const PrescribedInterval& interval,GeometryWork& g) {
  const auto& v=interval.velocity_midpoint; auto& o=g.values;
  for (unsigned n=0;n<4;++n) {
    const auto r=Project(o.frame,interval.omega_midpoint[n]);
    o.projected_omega[2*n]=r.x; o.projected_omega[2*n+1]=r.y;
  }
  const auto vg13=Difference(v[0],v[2]),vg24=Difference(v[1],v[3]);
  const Vec3 vghi{v[0].x-v[1].x+v[2].x-v[3].x,
                  v[0].y-v[1].y+v[2].y-v[3].y,
                  v[0].z-v[1].z+v[2].z-v[3].z};
  g.v13=Project(o.frame,vg13); g.v24=Project(o.frame,vg24); g.vhi=Project(o.frame,vghi);
}
TL_QEPH_HD inline void CorrectMidpointVelocity(double dt,GeometryWork& g) {
  // Native correction acts on local x/y translational differences only.
  // Do not rotate omega to another temporal phase or impose finite-step zero
  // rigid strain: the retained Q1 analytic general-axis shear residual is O(h).
  const double dt05=.5*dt,dt025=.25*dt;
  const double exz=g.y24*g.v13.z-g.y13*g.v24.z;
  const double eyz=-g.x24*g.v13.z+g.x13*g.v24.z;
  const double ddry=dt05*exz*g.values.reciprocal_area;
  const double ddrx=dt05*eyz*g.values.reciprocal_area;
  const double v13x=g.v13.x,v24x=g.v24.x,vhix=g.vhi.x;
  const double ddrz1=::fabs(g.x13-g.x24)<1e-10?0:dt025*(g.v13.y-g.v24.y)/(g.x13-g.x24);
  g.v13.x=g.v13.x-ddry*g.v13.z-ddrz1*g.v13.y;
  g.v24.x=g.v24.x-ddry*g.v24.z-ddrz1*g.v24.y;
  g.vhi.x=g.vhi.x-ddry*g.vhi.z-ddrz1*g.vhi.y;
  const double ddrz2=::fabs(g.y13+g.y24)<1e-10?0:dt025*(v13x+v24x)/(g.y13+g.y24);
  g.v13.y=g.v13.y-ddrx*g.v13.z-ddrz2*v13x;
  g.v24.y=g.v24.y-ddrx*g.v24.z-ddrz2*v24x;
  g.vhi.y=g.vhi.y-ddrx*g.vhi.z-ddrz2*vhix;
}
TL_QEPH_HD inline void NormalizeRates(GeometryWork& g) {
  const double aa=g.values.reciprocal_area;
  g.v13.x=g.v13.x*aa; g.v24.x=g.v24.x*aa; g.vhi.x=g.vhi.x*.25;
  g.v13.y=g.v13.y*aa; g.v24.y=g.v24.y*aa; g.vhi.y=g.vhi.y*.25;
  g.v13.z=g.v13.z*aa; g.v24.z=g.v24.z*aa; g.vhi.z=g.vhi.z*.25;
}
} // namespace tl::fea::qeph::detail
