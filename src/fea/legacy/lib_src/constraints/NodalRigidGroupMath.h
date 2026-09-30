// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../math/Fixed3.h"
#include "../math/Quaternion.h"
#include <cstddef>

#if defined(__CUDACC__)
#define TL_RIGID_HD __host__ __device__
#else
#define TL_RIGID_HD
#endif

namespace tl::fea::rigid {
using tl::math::Vec3;
using tl::math::Matrix3;
enum class MathStatus { Success,InvalidInput,NonfiniteResult };
struct Wrench { Vec3 force{},couple{}; };
// Columns are right-handed unit principal axes expressed in WORLD coordinates.
// Inertias are kg*m^2. This value has no timestep, ownership or evolving history.
struct PrincipalFrame { Matrix3 axes{}; Vec3 inertia{}; };
struct PrincipalCorrection {
  Vec3 effective{},added{};
  bool threshold_reached=false,changed=false;
};
namespace detail {
TL_RIGID_HD inline bool Finite(Vec3 v) {
  return tl::math::Finite(v.x)&&tl::math::Finite(v.y)&&tl::math::Finite(v.z);
}
TL_RIGID_HD inline Vec3 Add(Vec3 a,Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
TL_RIGID_HD inline Vec3 Subtract(Vec3 a,Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
TL_RIGID_HD inline Vec3 Cross(Vec3 a,Vec3 b) {
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
TL_RIGID_HD inline double Dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
TL_RIGID_HD inline Vec3 Column(const Matrix3& m,unsigned i) { return {m.v[i],m.v[3+i],m.v[6+i]}; }
TL_RIGID_HD inline Vec3 ToLocal(const Matrix3& m,Vec3 v) {
  return {Dot(Column(m,0),v),Dot(Column(m,1),v),Dot(Column(m,2),v)};
}
TL_RIGID_HD inline Vec3 ToWorld(const Matrix3& m,Vec3 v) {
  return {m.v[0]*v.x+m.v[1]*v.y+m.v[2]*v.z,
          m.v[3]*v.x+m.v[4]*v.y+m.v[5]*v.z,
          m.v[6]*v.x+m.v[7]*v.y+m.v[8]*v.z};
}
TL_RIGID_HD inline bool Positive(Vec3 v) { return Finite(v)&&v.x>0&&v.y>0&&v.z>0; }
TL_RIGID_HD inline bool Orthonormal(const Matrix3& m) {
  const auto a=Column(m,0),b=Column(m,1),c=Column(m,2);
  return Finite(a)&&Finite(b)&&Finite(c)&&
    ::fabs(Dot(a,a)-1)<=1e-12&&::fabs(Dot(b,b)-1)<=1e-12&&::fabs(Dot(c,c)-1)<=1e-12&&
    ::fabs(Dot(a,b))<=1e-12&&::fabs(Dot(a,c))<=1e-12&&::fabs(Dot(b,c))<=1e-12&&
    ::fabs(Dot(Cross(a,b),c)-1)<=1e-12;
}
} // namespace detail

// Pinned OpenRadioss INIRBY Ispher=2 arithmetic, lines 755-760. The outer
// threshold uses <= with native single-literal promotion of 1.E-3, while
// individual corrections use < with binary64 EM03=ONE/1000. The largest ORIGINAL
// principal inertia is used for all corrections; added inertia is a diagnostic.
// Failure leaves output unchanged. No caller-selectable regularization factor.
TL_RIGID_HD inline MathStatus CorrectPrincipalInertia(Vec3 raw,PrincipalCorrection& output) {
  if(!detail::Finite(raw)) return MathStatus::InvalidInput;
  const double largest=::fmax(raw.x,::fmax(raw.y,raw.z));
  if(largest<=0) return MathStatus::InvalidInput;
  const double smallest=::fmin(raw.x,::fmin(raw.y,raw.z));
  PrincipalCorrection next; next.effective=raw;
  next.threshold_reached=smallest<=static_cast<double>(1e-3f)*largest;
  if(next.threshold_reached) {
    double* values[]{&next.effective.x,&next.effective.y,&next.effective.z};
    double* additions[]{&next.added.x,&next.added.y,&next.added.z};
    for(unsigned i=0;i<3;++i) if(*values[i]/largest<1e-3) {
      const double before=*values[i]; *values[i]=before+.1*largest;
      *additions[i]=*values[i]-before; next.changed=true;
    }
  }
  if(!detail::Positive(next.effective)||!detail::Finite(next.added)) return MathStatus::NonfiniteResult;
  output=next; return MathStatus::Success;
}

// The arrays are complete, readable local-member lists in declared source order.
// Positions/forces/couples use m, N and N*m in WORLD coordinates. All input is
// evaluated before publishing output, including the last member. No allocations.
TL_RIGID_HD inline MathStatus AggregateWrench(Vec3 center,const Vec3* positions,
    const Vec3* forces,const Vec3* couples,std::size_t count,Wrench& output) {
  if(!detail::Finite(center)||!positions||!forces||!couples||!count) return MathStatus::InvalidInput;
  Wrench next;
  for(std::size_t i=0;i<count;++i) {
    if(!detail::Finite(positions[i])||!detail::Finite(forces[i])||!detail::Finite(couples[i]))
      return MathStatus::InvalidInput;
    const auto arm=detail::Subtract(positions[i],center);
    // Retain RGBODFP's left-associated C + r_y*F_z - r_z*F_y.
    // Forming C + Cross(r,F) changes cancellation before small-J division.
    const auto f=forces[i],c=couples[i];
    const Vec3 moment{(c.x+arm.y*f.z)-arm.z*f.y,
                      (c.y+arm.z*f.x)-arm.x*f.z,
                      (c.z+arm.x*f.y)-arm.y*f.x};
    next.force=detail::Add(next.force,forces[i]); next.couple=detail::Add(next.couple,moment);
    if(!detail::Finite(arm)||!detail::Finite(moment)||!detail::Finite(next.force)||!detail::Finite(next.couple))
      return MathStatus::NonfiniteResult;
  }
  output=next; return MathStatus::Success;
}

// Equivalent to the free explicit anisotropic RGBODFP branch, including Euler
// gyroscopic terms. Input is the CURRENT principal frame; this routine neither
// updates that frame nor selects its temporal phase. It returns acceleration
// directly, omitting the native multiply/divide by the primary's scalar proxy J.
TL_RIGID_HD inline MathStatus AngularAcceleration(const PrincipalFrame& frame,
    Vec3 omega_world,Vec3 couple_world,Vec3& output) {
  if(!detail::Orthonormal(frame.axes)||!detail::Positive(frame.inertia)||
     !detail::Finite(omega_world)||!detail::Finite(couple_world)) return MathStatus::InvalidInput;
  const auto w=detail::ToLocal(frame.axes,omega_world),t=detail::ToLocal(frame.axes,couple_world);
  const auto j=frame.inertia;
  const Vec3 local{(t.x+(j.y-j.z)*w.y*w.z)/j.x,
                   (t.y+(j.z-j.x)*w.z*w.x)/j.y,
                   (t.z+(j.x-j.y)*w.x*w.y)/j.z};
  const auto next=detail::ToWorld(frame.axes,local);
  if(!detail::Finite(w)||!detail::Finite(t)||!detail::Finite(local)||!detail::Finite(next))
    return MathStatus::NonfiniteResult;
  output=next; return MathStatus::Success;
}
} // namespace tl::fea::rigid
#undef TL_RIGID_HD
