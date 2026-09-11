// SPDX-License-Identifier: AGPL-3.0-or-later
// R4DEF3 large-displacement, INISPRI=0, double precision branch.
#pragma once
#include "Type13RecurrenceChecks.h"

namespace tl::fea::type13::detail {
TL_TYPE13_HD inline Status Deformation(
    const Reference& reference, const NativeHistory& accepted,
    const NativeFrame& frame, const NativeEndpointKinematics (&nodes)[2],
    double dt, NativeHistory& next) {
  const auto midpoint_y = tl::math::fixed3::Column(frame.midpoint_axes, 1);
  const auto midpoint_z = tl::math::fixed3::Column(frame.midpoint_axes, 2);
  const auto relative_velocity = tl::math::fixed3::Subtract(
      nodes[1].velocity, nodes[0].velocity);
  const auto summed_spin = tl::math::fixed3::Add(
      nodes[1].angular_velocity, nodes[0].angular_velocity);
  const double half_dt = .5 * dt;
  const double half_shear_y =
      tl::math::fixed3::Dot(relative_velocity, midpoint_y) * half_dt;
  const double half_shear_z =
      tl::math::fixed3::Dot(relative_velocity, midpoint_z) * half_dt;
  const double denominator = ::fmax(frame.midpoint_length, 1e-30);
  const double rotation_y =
      half_dt * tl::math::fixed3::Dot(summed_spin, midpoint_y) +
      2 * ::atan(half_shear_z / denominator);
  const double rotation_z =
      half_dt * tl::math::fixed3::Dot(summed_spin, midpoint_z) -
      2 * ::atan(half_shear_y / denominator);

  next.channels[0].deformation = frame.length - reference.length_native;
  next.channels[1].deformation =
      accepted.channels[1].deformation - rotation_z * frame.midpoint_length;
  next.channels[2].deformation =
      accepted.channels[2].deformation + rotation_y * frame.midpoint_length;
  const auto relative_spin_increment = tl::math::fixed3::Scale(
      tl::math::fixed3::Subtract(nodes[1].angular_velocity,
                                nodes[0].angular_velocity), dt);
  for (unsigned k = 0; k < 3; ++k) {
    const auto axis = tl::math::fixed3::Column(frame.midpoint_axes, k);
    // Preserve accepted + three products, not accepted + Dot(increment,axis).
    next.channels[k + 3].deformation = accepted.channels[k + 3].deformation +
        relative_spin_increment.x * axis.x +
        relative_spin_increment.y * axis.y +
        relative_spin_increment.z * axis.z;
  }
  next.transverse_axis = tl::math::fixed3::Column(frame.axes, 1);
  for (const auto& channel : next.channels) {
    if (!tl::math::fixed3::Finite(channel.deformation)) {
      return Status::NonfiniteResult;
    }
  }
  return Status::Success;
}
} // namespace tl::fea::type13::detail
