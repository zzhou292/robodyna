// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected CNCOEF3/CBACOOR/CNDT3: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "QbatForceChecks.h"
#include "QbatVelocityCorrection.h"

namespace tl::fea::qbat::detail {
namespace force_constant {
constexpr double onep414=(1.+4./10.)+1./100.+4./1000.;
constexpr double em30=1./(1e20*1e10);
}
TL_QBAT_HD inline bool CurrentLength(const PrescribedInterval& in,Kinematics& k) {
  const auto& g=k.geometry;
  const auto& x=in.position_endpoint;
  Vec3 local[4]{};
  for (unsigned j=1;j<4;++j) {
    local[j]=LocalVector(g.frame,{x[j].x-x[0].x,x[j].y-x[0].y,x[j].z-x[0].z});
  }
  const auto& p=g.centered_projected_position_m;
  const double x13=(p[0].x-p[2].x)*.5;
  const double x24=(p[1].x-p[3].x)*.5;
  const double y13=(p[0].y-p[2].y)*.5;
  const double y24=(p[1].y-p[3].y)*.5;
  const double l13=x13*x13+y13*y13;
  const double l24=x24*x24+y24*y24;
  const double ll=::fmax(l13,l24);
  double c1=p[1].x*p[3].y-p[1].y*p[3].x;
  double c2=p[0].x*p[2].y-p[0].y*p[2].x;
  const double lm=::fmax(::fabs(c1),::fabs(c2));
  const double rx=local[1].x+local[2].x-local[3].x;
  const double ry=local[1].y+local[2].y-local[3].y;
  const double sx=-local[1].x+local[2].x+local[3].x;
  const double sy=-local[1].y+local[2].y+local[3].y;
  c1=::sqrt(rx*rx+ry*ry);
  c2=::sqrt(sx*sx+sy*sy);
  if (!Positive(c1) || !Positive(c2) || !Positive(ll)) return false;
  double s1=.25*(::fmax(c1,c2)/::fmin(c1,c2)-1);
  const double fac1=::fmin(.5,s1)+1;
  double fac2=4*g.area_m2/(c1*c2);
  fac2=static_cast<double>(3.413f)*::fmax(0.,fac2-static_cast<double>(.7071f));
  fac2=static_cast<double>(.78f)+static_cast<double>(.22f)*fac2*fac2*fac2;
  const double faci=2*fac1*fac2;
  k.nodal_factor[0]=::sqrt(l24/ll);
  k.nodal_factor[1]=::sqrt(l13/ll);
  s1=::sqrt(faci*((4./3.)+lm*g.reciprocal_area_per_m2)*ll);
  if (!tl::math::Finite(s1)) return false;
  s1=::fmax(s1,1e-20);
  k.characteristic_length_m=g.area_m2/s1;
  return Positive(k.characteristic_length_m) &&
      Positive(k.nodal_factor[0]) && Positive(k.nodal_factor[1]);
}
TL_QBAT_HD inline bool ForceCoefficients(const Reference& r,const Material& p,
    const HistoryValues& base,bool active,const Kinematics& k,ForceDiagnostics& d) {
  d.membrane_viscosity=r.input().options.membrane_viscosity;
  d.numerical_viscosity=r.input().options.numerical_viscosity;
  if (d.numerical_viscosity==0) d.numerical_viscosity=1./1000.;
  d.sound_speed_m_s=p.sound_speed;
  double viscosity=::fmax(d.membrane_viscosity,d.numerical_viscosity);
  viscosity=::sqrt(1+viscosity*viscosity)-viscosity;
  d.viscosity_timestep_factor=viscosity;
  const double length=k.characteristic_length_m*viscosity/::sqrt(1.);
  d.unscaled_element_dt_s=1.*length/p.sound_speed;
  const double volume=base.thickness_m*k.geometry.area_m2;
  // Selected NODADT0, IDTMINS0 and centered TYPE1; no nodal clock admission.
  d.translation_stiffness_n_m=.5*1.*volume*p.a11*(active?1.:0.)/
      ::fmax(length*length,1e-20);
  d.rotation_stiffness_nm=0;
  return Positive(d.sound_speed_m_s) && Positive(viscosity) && Positive(d.unscaled_element_dt_s) &&
      tl::math::Finite(volume) && tl::math::Finite(d.translation_stiffness_n_m);
}
} // namespace tl::fea::qbat::detail
