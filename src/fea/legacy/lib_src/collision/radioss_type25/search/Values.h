// SPDX-License-Identifier: AGPL-3.0-or-later
// Native I25BUCE_CRIT extrema and INTCRIT NTY25 scalar ordering, a62b27e6.
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::search {
namespace detail {
TL_MATH_HOST_DEVICE inline bool Same(SourceStamp a, SourceStamp b) noexcept {
  return a.source == b.source && a.topology == b.topology && a.activity == b.activity;
}
TL_MATH_HOST_DEVICE inline double Maximum(double a, double b) noexcept { return a < b ? b : a; }
TL_MATH_HOST_DEVICE inline double Minimum(double a, double b) noexcept { return b < a ? b : a; }
TL_MATH_HOST_DEVICE inline void Observe(MotionExtrema& e, Vector v) noexcept {
  e.maximum = {Maximum(e.maximum.x,v.x), Maximum(e.maximum.y,v.y), Maximum(e.maximum.z,v.z)};
  e.minimum = {Minimum(e.minimum.x,v.x), Minimum(e.minimum.y,v.y), Minimum(e.minimum.z,v.z)};
}
TL_MATH_HOST_DEVICE inline void Merge(MotionExtrema& a, const MotionExtrema& b) noexcept {
  a.maximum = {Maximum(a.maximum.x,b.maximum.x), Maximum(a.maximum.y,b.maximum.y), Maximum(a.maximum.z,b.maximum.z)};
  a.minimum = {Minimum(a.minimum.x,b.minimum.x), Minimum(a.minimum.y,b.minimum.y), Minimum(a.minimum.z,b.minimum.z)};
}
TL_MATH_HOST_DEVICE inline void Merge(Extrema& a, const Extrema& b) noexcept {
  Merge(a.secondary_displacement,b.secondary_displacement); Merge(a.main_displacement,b.main_displacement);
  Merge(a.secondary_velocity,b.secondary_velocity); Merge(a.main_velocity,b.main_velocity);
  a.maximum_gap_change = Maximum(a.maximum_gap_change,b.maximum_gap_change);
  a.secondary_uses += b.secondary_uses; a.main_uses += b.main_uses;
}
TL_MATH_HOST_DEVICE inline Vector Relative(const MotionExtrema& a, const MotionExtrema& b) noexcept {
  return {Maximum(Maximum(a.maximum.x-b.minimum.x,b.maximum.x-a.minimum.x),0.),
          Maximum(Maximum(a.maximum.y-b.minimum.y,b.maximum.y-a.minimum.y),0.),
          Maximum(Maximum(a.maximum.z-b.minimum.z,b.maximum.z-a.minimum.z),0.)};
}
TL_MATH_HOST_DEVICE inline bool Finite(const MotionExtrema& e) noexcept {
  return tl::math::fixed3::Finite(e.maximum) && tl::math::fixed3::Finite(e.minimum) &&
      e.maximum.x >= e.minimum.x && e.maximum.y >= e.minimum.y && e.maximum.z >= e.minimum.z;
}
}
namespace detail {
TL_MATH_HOST_DEVICE inline bool Empty(const MotionExtrema& e) noexcept {
  return e.maximum.x==-ExtentIdentity && e.maximum.y==-ExtentIdentity && e.maximum.z==-ExtentIdentity &&
      e.minimum.x==ExtentIdentity && e.minimum.y==ExtentIdentity && e.minimum.z==ExtentIdentity;
}
TL_MATH_HOST_DEVICE inline bool Valid(const MotionExtrema& e,std::uint64_t uses) noexcept {
  return uses ? Finite(e) : Empty(e);
}
}
// force_sort represents the native KFORSMS/frontier override at the value
// boundary. This helper alone grants no authority over a candidate inventory.
TL_MATH_HOST_DEVICE inline Status EvaluateBudget(const Extrema& extrema, double margin,
    double previous_dt, bool force_sort, ActivityPolicy activity, Budget& output) noexcept {
  using namespace detail;
  if (!normal_detail::Nonnegative(margin) || !normal_detail::Nonnegative(previous_dt) ||
      !tl::math::Finite(extrema.maximum_gap_change)) return Status::InvalidInput;
  if (activity!=ActivityPolicy::Immutable && activity!=ActivityPolicy::MonotoneRetirement)
    return Status::InvalidInput;
  if (activity==ActivityPolicy::Immutable && (!extrema.secondary_uses || !extrema.main_uses))
    return Status::UnsupportedLifecycle;
  if (!Valid(extrema.secondary_displacement,extrema.secondary_uses) ||
      !Valid(extrema.main_displacement,extrema.main_uses) ||
      !Valid(extrema.secondary_velocity,extrema.secondary_uses) ||
      !Valid(extrema.main_velocity,extrema.main_uses)) return Status::NonfiniteResult;
  const auto delta = Relative(extrema.secondary_displacement,extrema.main_displacement);
  const auto velocity = Relative(extrema.secondary_velocity,extrema.main_velocity);
  Budget next;
  next.displacement = ::sqrt(delta.x*delta.x+delta.y*delta.y+delta.z*delta.z);
  next.relative_speed = ::sqrt(velocity.x*velocity.x+velocity.y*velocity.y+velocity.z*velocity.z);
  next.raw_motion = MotionFactor * next.relative_speed * previous_dt;
  next.stored_motion = next.raw_motion;
  next.raw_distance = margin - MotionFactor * (next.displacement+next.raw_motion+extrema.maximum_gap_change);
  next.distance = next.raw_distance;
  if (next.raw_motion > 2*margin) {
    next.velocity = VelocityStatus::Warning;
    if (next.raw_motion > 5*margin) {
      next.velocity = VelocityStatus::Error;
      next.stored_motion = margin;
    }
  }
  next.requires_sort = next.raw_distance <= 0 || force_sort;
  if (next.requires_sort) next.distance = -1.;
  const double values[]{next.displacement,next.relative_speed,next.raw_motion,
      next.stored_motion,next.raw_distance,next.distance};
  for (double value:values) if (!tl::math::Finite(value)) return Status::NonfiniteResult;
  output = next;
  return Status::Ok;
}
TL_MATH_HOST_DEVICE inline Status EvaluateBudget(const Extrema& extrema,double margin,
    double previous_dt,bool force_sort,Budget& output) noexcept {
  return EvaluateBudget(extrema,margin,previous_dt,force_sort,ActivityPolicy::Immutable,output);
}
} // namespace tlfea::contact::radioss_type25::search
