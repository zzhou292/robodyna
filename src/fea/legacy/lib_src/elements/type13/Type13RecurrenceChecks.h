// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type13Units.h"
#include "Type13RecurrenceTypes.h"

namespace tl::fea::type13::detail {
TL_TYPE13_HD inline bool ValidReference(const Property& property,
                                       const Reference& reference) {
  if (!property.initialized() || !Positive(reference.length_native) ||
      !Positive(reference.length_m) ||
      reference.length_m != reference.length_native * property.units().length_to_m ||
      !tl::math::fixed3::Orthonormal(reference.axes) ||
      !tl::math::fixed3::Finite(reference.position_m[0]) ||
      !tl::math::fixed3::Finite(reference.position_m[1])) {
    return false;
  }
  const auto chord = tl::math::fixed3::Subtract(reference.position_m[1],
                                               reference.position_m[0]);
  const auto expected = tl::math::fixed3::Scale(
      tl::math::fixed3::Column(reference.axes, 0), reference.length_m);
  const double scale = tl::math::fixed3::Norm(reference.position_m[0]) +
                       tl::math::fixed3::Norm(reference.position_m[1]) +
                       reference.length_m;
  const double error = tl::math::fixed3::Norm(
      tl::math::fixed3::Subtract(chord, expected));
  return Positive(scale) && tl::math::fixed3::Finite(error) &&
         error <= 64 * 2.2204460492503131e-16 * scale;
}

TL_TYPE13_HD inline bool ValidNodes(const NativeEndpointKinematics (&nodes)[2]) {
  for (const auto& node : nodes) {
    if (!tl::math::fixed3::Finite(node.position) ||
        !tl::math::fixed3::Finite(node.velocity) ||
        !tl::math::fixed3::Finite(node.angular_velocity)) {
      return false;
    }
  }
  return true;
}

TL_TYPE13_HD inline bool ValidHistory(const NativeHistory& history) {
  if (!tl::math::fixed3::Unit(history.transverse_axis) ||
      !Nonnegative(history.failure_criterion) || history.failure_criterion > 1 ||
      (history.active ? history.failure_criterion >= 1
                      : history.failure_criterion != 1)) {
    return false;
  }
  for (const auto& channel : history.channels) {
    if (!tl::math::fixed3::Finite(channel.deformation) ||
        !Nonnegative(channel.accumulated_plastic_deformation) ||
        !tl::math::fixed3::Finite(channel.elastic_plastic_force) ||
        !tl::math::fixed3::Finite(channel.force) ||
        !tl::math::fixed3::Finite(channel.signed_work) ||
        channel.curve_position >= CurvePoints - 1) {
      return false;
    }
  }
  return true;
}
} // namespace tl::fea::type13::detail
