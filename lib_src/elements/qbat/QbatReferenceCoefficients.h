// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected CNDLENI: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "QbatChecks.h"

namespace tl::fea::qbat::detail {
TL_QBAT_HD inline Status ReferenceCoefficients(const ReferenceInput& input,
    const NativeQuadReference& q, StartupCoefficients& result) {
  const auto& material=input.quadrilateral;
  const auto& local=q.local_position;
  const double cx=.25*(local[1].x+local[2].x+local[3].x);
  const double cy=.25*(local[1].y+local[2].y+local[3].y);
  const Vec3 centered[4]{{-cx,-cy,0}, {local[1].x-cx,local[1].y-cy,0},
      {local[2].x-cx,local[2].y-cy,0}, {local[3].x-cx,local[3].y-cy,0}};
  const double x13=(centered[0].x-centered[2].x)*.5;
  const double x24=(centered[1].x-centered[3].x)*.5;
  const double y13=(centered[0].y-centered[2].y)*.5;
  const double y24=(centered[1].y-centered[3].y)*.5;
  const double l13=x13*x13+y13*y13;
  const double l24=x24*x24+y24*y24;
  const double al1=::fmax(l13,l24);
  double c1=centered[1].x*centered[3].y-centered[1].y*centered[3].x;
  double c2=centered[0].x*centered[2].y-centered[0].y*centered[2].x;
  const double al2=::fmax(::fabs(c1),::fabs(c2))/q.area;
  const double rx=x24-x13;
  const double ry=y24-y13;
  const double sx=-x24-x13;
  const double sy=-y24-y13;
  c1=::sqrt(rx*rx+ry*ry);
  c2=::sqrt(sx*sx+sy*sy);
  if (!Positive(c1) || !Positive(c2)) return Status::kNonfiniteResult;
  double s1=.25*(::fmax(c1,c2)/::fmin(c1,c2)-1);
  const double fac1=::fmin(.5,s1)+1;
  double fac2=q.area/(c1*c2);
  // Unsuffixed source literals are REAL32 values promoted to MYREAL8.
  fac2=static_cast<double>(3.413f)*::fmax(0.,fac2-static_cast<double>(.7071f));
  fac2=static_cast<double>(.78f)+static_cast<double>(.22f)*fac2*fac2*fac2;
  const double faci=2*fac1*fac2;
  s1=::sqrt(faci*((4./3.)+al2)*al1); // Native IHBE11, not QEPH's 5/4.
  if (!Positive(s1)) return Status::kNonfiniteResult;
  s1=::fmax(s1,1e-20);
  StartupCoefficients next;
  next.characteristic_length_m=q.area/s1;
  const double nu=material.poisson_ratio;
  next.sound_speed_m_s=::sqrt(material.young_modulus/(1-nu*nu)/material.density);
  double viscosity=::fmax(input.options.membrane_viscosity,input.options.numerical_viscosity);
  viscosity=::sqrt(1+viscosity*viscosity)-viscosity;
  next.viscosity_timestep_factor=viscosity;
  next.unscaled_element_dt_s=next.characteristic_length_m*viscosity/next.sound_speed_m_s;
  double vv=viscosity*next.characteristic_length_m;
  vv=vv*vv;
  const double thickness=material.thickness;
  next.nodal_translation_stiffness_n_m=.5*thickness*q.area*input.initial_a11_pa/vv;
  next.nodal_rotation_stiffness_nm=next.nodal_translation_stiffness_n_m*thickness*thickness*(1./12.);
  const double values[]{next.characteristic_length_m,next.sound_speed_m_s,viscosity,
      next.unscaled_element_dt_s,next.nodal_translation_stiffness_n_m,next.nodal_rotation_stiffness_nm};
  for (double value:values) {
    if (!Positive(value)) return Status::kNonfiniteResult;
  }
  result=next;
  return Status::kSuccess;
}
} // namespace tl::fea::qbat::detail
