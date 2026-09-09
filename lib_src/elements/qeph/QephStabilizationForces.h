// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens, CZFINTN1 native linear damping/force contraction.
#pragma once
#include "QephElasticForces.h"
#include "QephStabilizationState.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline void StabilizationForces(const GeometryWork& g,const MaterialWork& m,
    HistoryValues& h,const StabilizationWork& w,LocalForceWork& local) {
  using namespace force_constant;
  const auto* vg=h.stabilization; const auto* vh=g.values.hourglass_rate;
  const auto* dhg=w.increment; auto* f=local.force; auto& cm=local.couple;
  const double c8=four_over_3*1.;
  double ss1=(g.my34*vg[0]+g.my23*vg[6])*c8;
  double ss2=(g.mx23*vg[7]+g.mx34*vg[1])*c8;
  double sf1=(g.my34*vg[2]+g.my23*vg[8])*c8;
  double sf2=-(g.mx23*vg[9]+g.mx34*vg[3])*c8;
  const double hsura=m.thickness*g.values.reciprocal_area;
  double c2=c8*m.thickness;
  double sc5=(g.my34*vg[4]+g.mx34*vg[5])*c2;
  double sc6=(g.my23*vg[10]+g.mx23*vg[11])*c2;
  double ss3=sc5+sc6;
  const double hvl=m.dn*::sqrt(m.rho*g.values.area*1.)*1.;
  const double ssv0=g.my23*g.my23,ssv1=g.my34*g.my34;
  const double ssv2=g.mx23*g.mx23,ssv3=g.mx34*g.mx34;
  const double hxxv=fivep333*(ssv1+ssv0);
  const double hxyv=-fivep333*(g.my34*g.mx34+g.my23*g.mx23);
  const double hyyv=fivep333*(ssv2+ssv3);
  c2=hvl*m.g_sqrt*m.shf_sqrt*::sqrt(one_over_12);
  const double cxzv=(ssv1+ssv3)*c2,cyzv=(ssv2+ssv0)*c2;
  const double aux=g.values.reciprocal_area*hvl;
  const double c1m=m.a11_sqrt*aux,c2m=m.a12_sqrt*aux;
  const double cxxv=c1m*hxxv,cyyv=c1m*hyyv,cxyv=c2m*hxyv;
  const double ss1v=cxxv*vh[0]+cxyv*vh[1];
  const double ss2v=cyyv*vh[1]+cxyv*vh[0];
  const double sf1v=(cxxv*vh[2]+cxyv*vh[3])*threep464;
  const double sf2v=(-cyyv*vh[3]-cxyv*vh[2])*threep464;
  const double sc5v=cxzv*vh[4]*hsura,sc6v=cyzv*vh[5]*hsura;
  const double ss3v=sc5v+sc6v;
  ss1=ss1+ss1v; ss2=ss2+ss2v; ss3=ss3+ss3v;
  sc5=sc5+sc5v; sc6=sc6+sc6v; sf1=sf1+sf1v; sf2=sf2+sf2v;
  const double y13s=g.my13*ss3,x13s=g.mx13*ss3;
  const double y34s6=g.my34*sc6,y23s5=g.my23*sc5;
  const double x23s5=g.mx23*sc5,x34s6=g.mx34*sc6;
  c2=.25*m.thickness;
  const double b13=(g.my13*g.x24-g.mx13*g.y24)*hsura;
  f[0].x=f[0].x+b13*ss1; f[2].x=c2*ss1;
  f[0].y=f[0].y+b13*ss2; f[2].y=c2*ss2; f[2].z=ss3;
  const double b24=(g.mx13*g.y13-g.my13*g.x13)*hsura;
  f[1].x=f[1].x+b24*ss1; f[3].x=-f[2].x;
  f[1].y=f[1].y+b24*ss2; f[3].y=-f[2].y; f[3].z=-f[2].z;
  double c3=w.bending_factor*b13; const double c4=w.bending_factor*c2;
  cm[0][0]=cm[0][0]+c3*sf2+y23s5+y34s6;
  cm[2][0]=cm[2][0]+c4*sf2-y13s;
  cm[0][1]=cm[0][1]+c3*sf1-x23s5-x34s6;
  cm[2][1]=cm[2][1]+c4*sf1+x13s;
  c3=w.bending_factor*b24;
  cm[1][0]=cm[1][0]+c3*sf2+y23s5-y34s6;
  cm[3][0]=cm[3][0]-c4*sf2-y13s;
  cm[1][1]=cm[1][1]+c3*sf1-x23s5+x34s6;
  cm[3][1]=cm[3][1]-c4*sf1+x13s;
  c2=g.values.effective_warpage*hsura;
  f[0].z=f[0].z+c2*(ss1*g.y24-ss2*g.x24);
  f[1].z=f[1].z+c2*(-ss1*g.y13+ss2*g.x13);
  const double esy=((ss1-ss1v)*dhg[0]+(ss2-ss2v)*dhg[1])*m.thickness+
      .25*((sc5-sc5v)*dhg[4]+(sc6-sc6v)*dhg[5]);
  const double emy=(sf1-sf1v)*dhg[2]-(sf2-sf2v)*dhg[3];
  const double work0=.5*esy,work1=.5*w.bending_factor*emy*m.thickness;
  const double tesy=(ss1v*dhg[0]+ss2v*dhg[1])*m.thickness+
      (sf1v*dhg[2]-sf2v*dhg[3])*m.thickness*w.bending_factor+
      .25*(sc5v*dhg[4]+sc6v*dhg[5]);
  h.internal_work[0]=h.internal_work[0]+work0;
  h.internal_work[1]=h.internal_work[1]+work1;
  h.hourglass_viscous_work=h.hourglass_viscous_work+tesy;
}
} // namespace tl::fea::qeph::detail
