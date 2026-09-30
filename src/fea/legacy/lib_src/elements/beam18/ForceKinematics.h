// SPDX-License-Identifier: AGPL-3.0-or-later
// PEVEC3/PDEFO3/PCURV3, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "ForceChecks.h"

namespace tl::fea::beam18::force_detail {
TL_BEAM18_HD inline Vec3 Local(const ForceGeometry& g, Vec3 value) noexcept {
  using tl::math::fixed3::Dot;
  return {Dot(g.axis[0],value),Dot(g.axis[1],value),Dot(g.axis[2],value)};
}
TL_BEAM18_HD inline bool Normalize(Vec3& v) noexcept {
  const double norm = ::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
  if (!detail::Positive(norm)) return false;
  v.x /= norm; v.y /= norm; v.z /= norm;
  return tl::math::fixed3::Finite(v);
}
TL_BEAM18_HD inline bool Geometry(const HistoryValues& accepted, const PrescribedInterval& in,
    ForceGeometry& g) noexcept {
  using namespace tl::math::fixed3;
  auto& e1 = g.axis[0]; auto& e2 = g.axis[1]; auto& e3 = g.axis[2];
  e1 = Subtract(in.position_endpoint_m[1],in.position_endpoint_m[0]);
  g.length_m = ::sqrt(e1.x*e1.x+e1.y*e1.y+e1.z*e1.z);
  if (!detail::Positive(g.length_m)) return false;
  e1.x /= g.length_m; e1.y /= g.length_m; e1.z /= g.length_m;
  e2 = accepted.section_seed;
  e3 = Cross(e1,e2);
  e2 = Cross(e3,e1);
  const double r1 = Dot(e1,in.angular_velocity_midpoint_rad_s[0]);
  const double r2 = Dot(e1,in.angular_velocity_midpoint_rad_s[1]);
  const double theta = (r1+r2)/2.*in.dt_s;
  const double norm2 = ::sqrt(Dot(e2,e2)), norm3 = ::sqrt(Dot(e3,e3));
  if (!detail::Positive(norm2) || !detail::Positive(norm3) || !tl::math::Finite(theta)) return false;
  const double cost = ::cos(theta)/norm2, sint = ::sin(theta)/norm3;
  e2 = {e2.x*cost+e3.x*sint,e2.y*cost+e3.y*sint,e2.z*cost+e3.z*sint};
  if (!Normalize(e2)) return false;
  e3 = Cross(e1,e2);
  return Normalize(e3);
}
TL_BEAM18_HD inline bool Rates(const ForceGeometry& g, const PrescribedInterval& in,
    GeneralizedRate& out) noexcept {
  const auto v1 = Local(g,in.velocity_midpoint_m_s[0]);
  const auto v2 = Local(g,in.velocity_midpoint_m_s[1]);
  auto r1 = Local(g,in.angular_velocity_midpoint_rad_s[0]);
  auto r2 = Local(g,in.angular_velocity_midpoint_rad_s[1]);
  double exx = (v2.x-v1.x)/g.length_m;
  double exy = (v2.y-v1.y)/g.length_m;
  double exz = (v2.z-v1.z)/g.length_m;
  const double dt05 = .5*in.dt_s;
  const double rxav = .5*dt05*(r1.x+r2.x);
  const double exx00 = exx, exy00 = exy, exz00 = exz;
  const double exy0 = dt05*exy, exz0 = dt05*exz;
  exx = exx-(exy0*exy00+exz0*exz00);
  exy = exy+exz0*exx00;
  exz = exz+exy0*exx00;
  const double rz10 = r1.z, ry10 = r1.y, rz20 = r2.z, ry20 = r2.y;
  r1.x = r1.x-exy0*ry10-exz0*rz10;
  r1.y = r1.y-rxav*(rz10-exy);
  r1.z = r1.z+rxav*(ry10+exz);
  r2.x = r2.x-exy0*ry20-exz0*rz20;
  r2.y = r2.y-rxav*(rz20-exy);
  r2.z = r2.z+rxav*(ry20+exz);
  // Selected source releases resolve all native IR factors to one.
  r1.x *= 1.; r1.y *= 1.; r1.z *= 1.;
  r2.x *= 1.; r2.y *= 1.; r2.z *= 1.;
  exz *= 1.; exy *= 1.;
  r1.y = 1.*r1.y-(1.-1.)*(1.5*exz+.5*r2.y);
  r2.y = 1.*r2.y-(1.-1.)*(1.5*exz+.5*r1.y);
  r1.z = 1.*r1.z+(1.-1.)*(1.5*exy-.5*r2.z);
  r2.z = 1.*r2.z+(1.-1.)*(1.5*exy-.5*r1.z);
  out.curvature_x = (r2.x-r1.x)/g.length_m;
  out.curvature_y = (r2.y-r1.y)/g.length_m;
  out.curvature_z = (r2.z-r1.z)/g.length_m;
  out.axial = exx;
  out.shear_z = exz+.5*(r1.y+r2.y);
  out.shear_y = exy-.5*(r1.z+r2.z);
  const double values[]{out.axial,out.shear_y,out.shear_z,out.curvature_x,out.curvature_y,out.curvature_z};
  for (double v : values) if (!tl::math::Finite(v)) return false;
  return true;
}
} // namespace tl::fea::beam18::force_detail
