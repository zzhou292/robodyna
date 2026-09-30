// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Types.h"

namespace tl::fea::type45::detail {
using namespace tl::math::fixed3;
TL_TYPE45_HD inline bool Nonnegative(double x) { return Finite(x) && x >= 0; }
TL_TYPE45_HD inline bool Positive(double x) { return Finite(x) && x > 0; }
// All callers validate finiteness first. Include the sign of represented zero
// without comparing C++ object padding or introducing a lossy identity hash.
TL_TYPE45_HD inline bool Same(double a, double b) {
  return a == b && (a != 0 || ::copysign(1., a) == ::copysign(1., b));
}
TL_TYPE45_HD inline bool Same(Vec3 a, Vec3 b) {
  return Same(a.x,b.x) && Same(a.y,b.y) && Same(a.z,b.z);
}
TL_TYPE45_HD inline double Get(Vec3 v, unsigned axis) {
  return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}
TL_TYPE45_HD inline void Set(Vec3& v, unsigned axis, double x) {
  if (axis == 0) v.x = x;
  else if (axis == 1) v.y = x;
  else v.z = x;
}
TL_TYPE45_HD inline bool Blocked(Kind kind, unsigned dof) {
  if (dof < 3) return kind != Kind::Cylindrical || dof != 0;
  return kind != Kind::Spherical && dof != 3;
}
TL_TYPE45_HD inline double Get(const DofValues& values, unsigned dof) {
  return dof < 3 ? Get(values.translation,dof) : Get(values.rotation,dof-3);
}
TL_TYPE45_HD inline void Set(DofValues& values, unsigned dof, double x) {
  if (dof < 3) Set(values.translation,dof,x);
  else Set(values.rotation,dof-3,x);
}
TL_TYPE45_HD inline bool Valid(const Property& p) {
  if ((p.kind != Kind::Spherical && p.kind != Kind::Revolute &&
       p.kind != Kind::Cylindrical) ||
      (p.working_units != WorkingUnits::SI &&
       p.working_units != WorkingUnits::MillimetreTonneSecond) ||
      !Positive(p.automatic_stiffness_scale) ||
      !Positive(p.critical_damping_ratio) || p.critical_damping_ratio > 1) return false;
  for (unsigned i=0; i<6; ++i) {
    const double k=Get(p.free_stiffness,i), c=Get(p.free_viscosity,i);
    if (!Nonnegative(k) || !Nonnegative(c) || (Blocked(p.kind,i) && (k!=0 || c!=0)))
      return false;
  }
  return true;
}
TL_TYPE45_HD inline bool Same(const Property& a, const Property& b) {
  if (a.kind!=b.kind || a.working_units!=b.working_units ||
      !Same(a.automatic_stiffness_scale,b.automatic_stiffness_scale) ||
      !Same(a.critical_damping_ratio,b.critical_damping_ratio)) return false;
  for (unsigned i=0; i<6; ++i)
    if (!Same(Get(a.free_stiffness,i),Get(b.free_stiffness,i)) ||
        !Same(Get(a.free_viscosity,i),Get(b.free_viscosity,i))) return false;
  return true;
}
TL_TYPE45_HD inline bool Valid(const HistoryValues& h) {
  return Orthonormal(h.frame,1e-10) && Finite(h.local_displacement_m) &&
    Finite(h.relative_rotation_rad) && Finite(h.local_force_n) &&
    Finite(h.local_couple_nm) && Finite(h.internal_work_j);
}
} // namespace tl::fea::type45::detail
