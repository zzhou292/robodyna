// SPDX-License-Identifier: AGPL-3.0-or-later
// ROTBMR arithmetic adapted from OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "NodalRigidGroupMath.h"

#if defined(__CUDACC__)
#define TL_RIGID_FRAME_HD __host__ __device__
#else
#define TL_RIGID_FRAME_HD
#endif

namespace tl::fea::rigid {
// Previous axes and the SAVED previous-frame angular velocity are input values.
// This native update has a small-angle floor and separate axis normalization;
// it is not the existing exact quaternion update. No phase is inferred here.
// dt=0 still executes native arithmetic. Failure leaves output unchanged.
TL_RIGID_FRAME_HD inline MathStatus RotatePrincipalFrame(const Matrix3& previous,
    Vec3 saved_body_omega,double previous_drift_dt,Matrix3& output) {
  if(!detail::Orthonormal(previous)||!detail::Finite(saved_body_omega)||
      !tl::math::Finite(previous_drift_dt)||previous_drift_dt<0) return MathStatus::InvalidInput;
  const double rx=previous_drift_dt*saved_body_omega.x;
  const double ry=previous_drift_dt*saved_body_omega.y;
  const double rz=previous_drift_dt*saved_body_omega.z;
  const double rx2=rx*rx,ry2=ry*ry,rz2=rz*rz;
  const double square=rx2+ry2+rz2;
  if(!tl::math::Finite(square)) return MathStatus::NonfiniteResult;
  const double r2=::fmax(1e-10,square),r=::sqrt(r2);
  double c=::cos(r); const double cm1=1-c; c=c*r2;
  const double s=::sin(r)*r,sz=rz*s,cz=rz*cm1;
  const double e11=rx2*cm1+c,e22=ry2*cm1+c;
  double e12=rx*ry*cm1; const double e21=e12-sz; e12=e12+sz;
  const double e13=rx*cz-ry*s,e23=ry*cz+rx*s;
  const auto* a=previous.v;
  Vec3 first{a[0]*e11+a[1]*e12+a[2]*e13,
             a[3]*e11+a[4]*e12+a[5]*e13,
             a[6]*e11+a[7]*e12+a[8]*e13};
  Vec3 second{a[0]*e21+a[1]*e22+a[2]*e23,
              a[3]*e21+a[4]*e22+a[5]*e23,
              a[6]*e21+a[7]*e22+a[8]*e23};
  double norm=::sqrt(first.x*first.x+first.y*first.y+first.z*first.z);
  if(!tl::math::Finite(norm)||norm<=0) return MathStatus::NonfiniteResult;
  first.x=first.x/norm; first.y=first.y/norm; first.z=first.z/norm;
  norm=::sqrt(second.x*second.x+second.y*second.y+second.z*second.z);
  if(!tl::math::Finite(norm)||norm<=0) return MathStatus::NonfiniteResult;
  second.x=second.x/norm; second.y=second.y/norm; second.z=second.z/norm;
  const auto third=detail::Cross(first,second);
  const Matrix3 next{{first.x,second.x,third.x,first.y,second.y,third.y,first.z,second.z,third.z}};
  if(!detail::Orthonormal(next)) return MathStatus::NonfiniteResult;
  output=next; return MathStatus::Success;
}
} // namespace tl::fea::rigid
#undef TL_RIGID_FRAME_HD
