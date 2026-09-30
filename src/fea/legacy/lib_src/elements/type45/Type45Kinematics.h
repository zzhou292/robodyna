// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Reference.h"

namespace tl::fea::type45::detail {
// QROT33, with the native diagonal/off-diagonal construction. Axes are columns;
// there is no quaternion-to-matrix round trip or replacement relative rotation.
TL_TYPE45_HD inline Matrix3 IncrementFrame(Vec3 axis, double cosine, double sine) {
  const double ci=1-cosine;
  const double xs=axis.x*sine, ys=axis.y*sine, zs=axis.z*sine;
  const double xy=axis.x*axis.y*ci, xz=axis.x*axis.z*ci, yz=axis.y*axis.z*ci;
  return {{axis.x*axis.x*ci+cosine, xy-zs, xz+ys,
           xy+zs, axis.y*axis.y*ci+cosine, yz-xs,
           xz-ys, yz+xs, axis.z*axis.z*ci+cosine}};
}
TL_TYPE45_HD inline bool AdvanceKinematics(
    const Reference& reference, const HistoryValues& accepted,
    const Interval& interval, HistoryValues& next, Diagnostics& diagnostics) {
  const auto& a=interval.angular_velocity_rad_s[0];
  const auto& b=interval.angular_velocity_rad_s[1];
  const double dt=interval.dt_s;
  const Vec3 mean{.5*(b.x+a.x)*dt, .5*(b.y+a.y)*dt, .5*(b.z+a.z)*dt};
  Vec3 axis=ToLocal(accepted.frame,mean);
  const double angle=Norm(axis);
  if (!Finite(angle)) return false;
  if (angle>0) axis=Divide(axis,angle);
  const auto increment=IncrementFrame(axis,::cos(angle),::sin(angle));
  for (unsigned row=0; row<3; ++row) {
    for (unsigned col=0; col<3; ++col) {
      next.frame.v[3*row+col]=accepted.frame.v[3*row]*increment.v[col]+
        accepted.frame.v[3*row+1]*increment.v[3+col]+
        accepted.frame.v[3*row+2]*increment.v[6+col];
    }
  }
  const Vec3 relative{(b.x-a.x)*dt,(b.y-a.y)*dt,(b.z-a.z)*dt};
  for (unsigned i=0; i<3; ++i) {
    const auto column=Column(next.frame,i);
    const double rm=Get(accepted.relative_rotation_rad,i)+column.x*relative.x+
      column.y*relative.y+column.z*relative.z;
    // Native stores ROT2 and ROT1 separately before RUSER33 subtracts them.
    Set(next.relative_rotation_rad,i,.5*rm-(-.5*rm));
  }
  diagnostics.local_separation_m=ToLocal(next.frame,
    Subtract(interval.position_m[1],interval.position_m[0]));
  next.local_displacement_m=Subtract(diagnostics.local_separation_m,
                                   reference.local_separation_m());
  diagnostics.local_velocity_m_s=Divide(
    Subtract(next.local_displacement_m,accepted.local_displacement_m),dt);
  diagnostics.relative_rate_rad_s=Divide(
    Subtract(next.relative_rotation_rad,accepted.relative_rotation_rad),dt);
  return Orthonormal(next.frame,1e-10) && Finite(next.local_displacement_m) &&
    Finite(next.relative_rotation_rad) && Finite(diagnostics.local_separation_m) &&
    Finite(diagnostics.local_velocity_m_s) && Finite(diagnostics.relative_rate_rad_s);
}
} // namespace tl::fea::type45::detail
