// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tlfea::contact::self_contact_transaction {
namespace {

struct Interval {
  double lower = 0;
  double upper = 0;
};

double Down(double value) noexcept {
  return std::nextafter(
      value, -std::numeric_limits<double>::infinity());
}

double Up(double value) noexcept {
  return std::nextafter(
      value, std::numeric_limits<double>::infinity());
}

double Component(Vec3 value, unsigned component) noexcept {
  return component == 0 ? value.x :
      (component == 1 ? value.y : value.z);
}

double Component(tl::math::Vec3 value,
                 unsigned component) noexcept {
  return component == 0 ? value.x :
      (component == 1 ? value.y : value.z);
}

void SetComponent(Vec3* value, unsigned component,
                  double next) noexcept {
  if (component == 0) value->x = next;
  else if (component == 1) value->y = next;
  else value->z = next;
}

bool Difference(double first, double second,
                Interval* output) noexcept {
  if (!output || !std::isfinite(first) ||
      !std::isfinite(second))
    return false;
  if (first == second) {
    *output = {};
    return true;
  }
  const double value = first - second;
  if (!std::isfinite(value)) return false;
  *output = {Down(value), Up(value)};
  return std::isfinite(output->lower) &&
      std::isfinite(output->upper);
}

bool Product(const Interval& first, const Interval& second,
             Interval* output) noexcept {
  if (!output ||
      !std::isfinite(first.lower) ||
      !std::isfinite(first.upper) ||
      !std::isfinite(second.lower) ||
      !std::isfinite(second.upper) ||
      first.lower > first.upper ||
      second.lower > second.upper)
    return false;
  if ((first.lower == 0 && first.upper == 0) ||
      (second.lower == 0 && second.upper == 0)) {
    *output = {};
    return true;
  }
  const double products[4]{
      first.lower * second.lower,
      first.lower * second.upper,
      first.upper * second.lower,
      first.upper * second.upper};
  double lower = products[0], upper = products[0];
  for (const double value : products) {
    if (!std::isfinite(value)) return false;
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  *output = {Down(lower), Up(upper)};
  return std::isfinite(output->lower) &&
      std::isfinite(output->upper);
}

bool Subtract(const Interval& first, const Interval& second,
              Interval* output) noexcept {
  if (!output) return false;
  const double lower = first.lower - second.upper;
  const double upper = first.upper - second.lower;
  if (!std::isfinite(lower) || !std::isfinite(upper))
    return false;
  *output = {Down(lower), Up(upper)};
  return std::isfinite(output->lower) &&
      std::isfinite(output->upper);
}

bool Cross(const Interval first[3], const Interval second[3],
           Interval output[3]) noexcept {
  for (unsigned component = 0; component < 3; ++component) {
    const unsigned a = (component + 1) % 3;
    const unsigned b = (component + 2) % 3;
    Interval positive, negative;
    if (!Product(first[a], second[b], &positive) ||
        !Product(first[b], second[a], &negative) ||
        !Subtract(positive, negative, output + component))
      return false;
  }
  return true;
}

bool SameIdentity(
    const tl::fea::NodalRigidGroupSnapshot& first,
    const tl::fea::NodalRigidGroupSnapshot& second) noexcept {
  return first.source_kind == second.source_kind &&
      first.source_group_id == second.source_group_id &&
      first.source_node_set_id == second.source_node_set_id;
}

}  // namespace

RigidMemberSweepStatus BuildRigidMemberSweepBounds(
    Vec3 accepted_member, Vec3 prepared_member,
    const tl::fea::NodalRigidGroupSnapshot& accepted_group,
    const tl::fea::NodalRigidGroupSnapshot& prepared_group,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, SelfContactSweptParentBounds* output) noexcept {
  using R = RigidMemberSweepStatus;
  if (!output || !SameIdentity(accepted_group, prepared_group) ||
      trajectory != tl::fea::NodalRigidMemberTrajectory::
          EndpointCorrectedSecondOrderDriftV1 ||
      !(duration > 0) || !std::isfinite(duration) ||
      !IsFinite(accepted_member) || !IsFinite(prepared_member) ||
      !tl::fea::rigid::detail::Finite(accepted_group.state.center) ||
      !tl::fea::rigid::detail::Finite(prepared_group.state.center) ||
      !tl::fea::rigid::detail::Finite(prepared_group.state.omega))
    return trajectory ==
            tl::fea::NodalRigidMemberTrajectory::
                EndpointCorrectedSecondOrderDriftV1
        ? R::InvalidInput : R::UnsupportedTrajectory;

  const auto omega = prepared_group.state.omega;
  constexpr long double Pi =
      3.141592653589793238462643383279502884L;
  const long double wx = static_cast<long double>(omega.x);
  const long double wy = static_cast<long double>(omega.y);
  const long double wz = static_cast<long double>(omega.z);
  const long double angle =
      static_cast<long double>(duration) *
      std::sqrt(wx * wx + wy * wy + wz * wz);
  if (!std::isfinite(angle)) return R::NonfiniteResult;
  if (angle >= Pi) return R::RotationLimit;

  Interval arm[3], w[3];
  for (unsigned component = 0; component < 3; ++component) {
    if (!Difference(
            Component(accepted_member, component),
            Component(accepted_group.state.center, component),
            arm + component))
      return R::NonfiniteResult;
    const double value = Component(omega, component);
    w[component] = {value, value};
  }
  Interval first_cross[3], second_cross[3];
  if (!Cross(w, arm, first_cross) ||
      !Cross(w, first_cross, second_cross))
    return R::NonfiniteResult;

  double duration_squared = duration * duration;
  if (!std::isfinite(duration_squared))
    return R::NonfiniteResult;
  if (duration_squared != 0)
    duration_squared = Up(duration_squared);
  else
    duration_squared = std::numeric_limits<double>::denorm_min();

  SelfContactSweptParentBounds next;
  for (unsigned component = 0; component < 3; ++component) {
    const double curvature = std::max(
        std::fabs(second_cross[component].lower),
        std::fabs(second_cross[component].upper));
    double deviation = 0;
    if (curvature != 0) {
      deviation = duration_squared * curvature;
      if (!std::isfinite(deviation))
        return R::NonfiniteResult;
      deviation = Up(deviation);
      deviation = Up(deviation * 0.125);
      if (!std::isfinite(deviation))
        return R::NonfiniteResult;
    }
    const double first = Component(accepted_member, component);
    const double second = Component(prepared_member, component);
    const double lower = std::min(first, second) - deviation;
    const double upper = std::max(first, second) + deviation;
    if (!std::isfinite(lower) || !std::isfinite(upper))
      return R::NonfiniteResult;
    SetComponent(&next.lower, component, Down(lower));
    SetComponent(&next.upper, component, Up(upper));
  }
  if (!IsFinite(next.lower) || !IsFinite(next.upper))
    return R::NonfiniteResult;

  // Proof.  The authenticated owner certificate defines, for u in [0,1],
  //
  // x(u) = (1-u)x0 + u*x1
  //        - .5*u*(1-u)*h^2*q,
  // q = w x (w x (x0-c0)).
  //
  // This is the center/relative form stated on NodalPreparedView after its
  // linear center terms cancel.  It contains x0 and x1 exactly, is nonlinear
  // whenever the owner's constant prepared spin has nonzero centripetal
  // curvature, and reduces to the explicit ordinary rigid-member recurrence
  // in exact arithmetic.  Since 0 <= u(1-u) <= 1/4, each coordinate differs
  // from its endpoint chord by at most h^2*|q_i|/8.  The interval cross
  // products enclose exact-real q for the binary64 inputs: one nextafter
  // surrounds every rounded product, sum and difference.  Directed h^2,
  // deviation and final-face rounding therefore make next an enclosure, never
  // a crossing certificate.  Overflow, nonfinite input and rotations outside
  // the owner's admitted minor-arc domain fail closed before publication.
  *output = next;
  return R::Ok;
}

}  // namespace tlfea::contact::self_contact_transaction
