// SPDX-License-Identifier: AGPL-3.0-or-later
// C3SROTO3/C3FINT3/C3FCUM3/C3MCUM3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3HistoryData.h"
#include "T3Material.h"
namespace tl::fea::t3::detail {
struct LocalForceWork { Vec3 force[3]{},couple[3]{}; };
TL_T3_HD inline void InternalForces(const GeometryWork& g,const MaterialWork& m,
                                   const HistoryValues& h,LocalForceWork& out) {
  // IFRAM_OLD1 C3SROTO3 copies FOR/MOM unchanged. No history-frame rotation.
  const double px=g.kinematics.derivative[0],py1=g.kinematics.derivative[1],py2=g.kinematics.derivative[2];
  auto& f=out.force; auto& c=out.couple;
  const double f1=h.stress[0]*m.thickness,f3=h.stress[2]*m.thickness;
  f[0].x=f1*px+f3*py1; f[1].x=-f1*px+f3*py2; f[2].x=-f[0].x-f[1].x;
  const double f2=h.stress[1]*m.thickness;
  f[0].y=f2*py1+f3*px; f[1].y=f2*py2-f3*px; f[2].y=-f[0].y-f[1].y;
  const double f4=h.stress[3]*m.thickness,f5=h.stress[4]*m.thickness;
  f[0].z=f5*px+f4*py1; f[1].z=-f5*px+f4*py2; f[2].z=-f[0].z-f[1].z;
  const double th2=m.thickness*m.thickness;
  const double m2=h.bending_stress[1]*th2,m3=h.bending_stress[2]*th2;
  c[0].x=-m2*py1-m3*px; c[1].x=-m2*py2+m3*px; c[2].x=-c[0].x-c[1].x;
  const double m1=h.bending_stress[0]*th2;
  c[0].y=m1*px+m3*py1; c[1].y=-m1*px+m3*py2; c[2].y=-c[0].y-c[1].y;
  double m4=f4*force_constant::third,m5=f5*force_constant::third;
  m5=m5*px;
  c[0].y=c[0].y+m5*(2.*py1+3.*py2)+m4*py1*(py1+2.*py2);
  c[1].y=c[1].y+m5*(3.*py1+2.*py2)-m4*py2*(2.*py1+py2);
  c[2].y=c[2].y+m5*(py1+py2)+m4*(py2*py2-py1*py1);
  m5=m5*px; m4=m4*px;
  c[0].x=c[0].x-m5-m4*(2.*py1+py2);
  c[1].x=c[1].x+m5-m4*(py1+2.*py2);
  c[2].x=c[2].x-m4*3.*(py1+py2);
}
TL_T3_HD inline void ProjectForces(const GeometryWork& g,const LocalForceWork& local,
                                  Vec3 (&force)[3],Vec3 (&couple)[3]) {
  const auto& f=g.kinematics.frame;
  for(unsigned n=0;n<3;++n) {
    const auto v=local.force[n],w=local.couple[n];
    // Native C3FCUM3 has three terms; C3MCUM3 exactly two (IDRIL0).
    force[n]={f.v[0]*v.x+f.v[1]*v.y+f.v[2]*v.z,
      f.v[3]*v.x+f.v[4]*v.y+f.v[5]*v.z,f.v[6]*v.x+f.v[7]*v.y+f.v[8]*v.z};
    couple[n]={f.v[0]*w.x+f.v[1]*w.y,f.v[3]*w.x+f.v[4]*w.y,f.v[6]*w.x+f.v[7]*w.y};
  }
}
} // namespace tl::fea::t3::detail
