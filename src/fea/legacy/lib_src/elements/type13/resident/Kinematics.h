// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Type13RecurrenceChecks.h"

namespace tl::fea::type13::batch_detail {
// Subsequent owner states are represented in SI. The independent oracle takes
// this exact adapted packet too. Initial native TT0 instead uses retained raw
// source positions; an SI round trip is deliberately not substituted there.
TL_TYPE13_HD inline bool FromSI(WorkingUnits units, Vec3 position, Vec3 velocity,
                                Vec3 spin, NativeEndpointKinematics& output) {
  NativeEndpointKinematics next;
  next.position = tl::math::fixed3::Divide(position, units.length_to_m);
  next.velocity = tl::math::fixed3::Scale(velocity, units.time_to_s / units.length_to_m);
  next.angular_velocity = tl::math::fixed3::Scale(spin, units.time_to_s);
  if (!tl::math::fixed3::Finite(next.position) ||
      !tl::math::fixed3::Finite(next.velocity) ||
      !tl::math::fixed3::Finite(next.angular_velocity)) {
    return false;
  }
  output = next;
  return true;
}

// Selected R2LEN3 profile: zero XCM/XCR, no rigid endpoint association.
// Native evaluates OFF after the current force response. Its removal interval
// therefore retains current force but already has zero endpoint STI/STIR.
TL_TYPE13_HD inline bool EndpointStiffness(const Evaluation& value,
                                          double& translation, double& rotation) {
  if (!detail::Positive(value.stability.translation_stiffness_N_per_m) ||
      !detail::Positive(value.stability.rotation_stiffness_Nm_per_rad)) {
    return false;
  }
  translation = value.native_history.active
      ? value.stability.translation_stiffness_N_per_m : 0;
  rotation = value.native_history.active
      ? value.stability.rotation_stiffness_Nm_per_rad : 0;
  return true;
}
} // namespace tl::fea::type13::batch_detail
