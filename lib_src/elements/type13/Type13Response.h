// SPDX-License-Identifier: AGPL-3.0-or-later
// REDEF3 H1, four owned curves, zero damping, A=LSCALE=1; R4DEF3 failure.
#pragma once
#include "Type13Curve.h"
#include "Type13RecurrenceChecks.h"

namespace tl::fea::type13::detail {

TL_TYPE13_HD inline Status ChannelResponse(
    const Property& property, unsigned channel_index, double length,
    const NativeChannelHistory& accepted, bool active, double dt,
    NativeChannelHistory& next) {
  const double deformation = next.deformation / length;
  const double old_deformation = accepted.deformation / length;
  double plastic = accepted.accumulated_plastic_deformation / length;
  double work = accepted.signed_work / length;
  const double increment = deformation - old_deformation;
  const auto& channel = property.channel(channel_index);
  const double stiffness = channel.native_stiffness;
  double force = accepted.elastic_plastic_force + stiffness * increment;
  const double query = force >= 0 ? plastic + force / stiffness
                                  : -plastic + force / stiffness;
  if (!tl::math::fixed3::Finite(deformation) ||
      !tl::math::fixed3::Finite(old_deformation) || !Nonnegative(plastic) ||
      !tl::math::fixed3::Finite(work) || !tl::math::fixed3::Finite(force) ||
      !tl::math::fixed3::Finite(query)) {
    return Status::NonfiniteResult;
  }

  double bound = 0;
  const auto status = Interpolate(property.curve(channel.declaration.curve_index),
                                  query, next.curve_position, bound);
  if (status != Status::Success) {
    return status;
  }
  if (force >= 0 && force > bound) {
    plastic = plastic + (force - bound) / stiffness;
    force = bound;
  } else if (force < 0 && force < bound) {
    plastic = plastic + (bound - force) / stiffness;
    force = bound;
  }
  next.elastic_plastic_force = force;

  // REDEF3 explicit dynamic branch, including the resolved zero coefficients.
  const double rate = increment / (dt == 0 ? 1e30 : dt);
  const double rate_magnitude = ::fmax(1.0, ::fabs(rate / 1));
  const double factor = 1 + 0 * ::log(rate_magnitude) + 0 * 0;
  next.force = (factor * force + 0 * rate + 0 * 0) * (active ? 1 : 0);
  if (!tl::math::fixed3::Finite(rate) || !tl::math::fixed3::Finite(factor)) {
    return Status::NonfiniteResult;
  }
  work = work + (deformation - old_deformation) *
                    (next.force + accepted.force) * .5;
  next.deformation = deformation * length;
  next.accumulated_plastic_deformation = plastic * length;
  next.signed_work = work * length;
  if (!tl::math::fixed3::Finite(next.deformation) ||
      !Nonnegative(next.accumulated_plastic_deformation) ||
      !tl::math::fixed3::Finite(next.elastic_plastic_force) ||
      !tl::math::fixed3::Finite(next.force) ||
      !tl::math::fixed3::Finite(next.signed_work)) {
    return Status::NonfiniteResult;
  }
  return Status::Success;
}

TL_TYPE13_HD inline Status Response(
    const Property& property, const Reference& reference,
    const NativeHistory& accepted, double dt, NativeHistory& next) {
  double criterion = 0;
  for (unsigned k = 0; k < ChannelCount; ++k) {
    const auto status = ChannelResponse(property, k, reference.length_native,
        accepted.channels[k], accepted.active, dt, next.channels[k]);
    if (status != Status::Success) {
      return status;
    }
    if (accepted.active) {
      const auto& channel = property.channel(k).declaration;
      const double limit = next.channels[k].deformation > 0
                               ? channel.failure_positive
                               : channel.failure_negative;
      const double denominator = reference.length_native * limit;
      if (!tl::math::fixed3::Finite(denominator) || denominator == 0) {
        return Status::NonfiniteResult;
      }
      const double ratio = next.channels[k].deformation / denominator;
      // Ifail2=0 overwrites declared alpha/beta with ONE/TWO.
      criterion = criterion + 1 * (ratio * ratio);
      if (!Nonnegative(criterion)) {
        return Status::NonfiniteResult;
      }
    }
  }
  next.failure_criterion = accepted.failure_criterion < 1
                               ? ::fmin(criterion, 1.0) : 1;
  if (accepted.active && criterion >= 1) {
    next.active = false;
  }
  return Status::Success;
}
} // namespace tl::fea::type13::detail
