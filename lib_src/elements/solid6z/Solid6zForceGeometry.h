// SPDX-License-Identifier: AGPL-3.0-or-later
// S6ZRCOOR3 / S6ZDERI3 / SGCOOR3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid6zForceGradients.h"
#include "Solid6zJacobian.h"
#include "lib_src/elements/solid_common/FrameTensor.h"
#include "lib_src/elements/solid_common/SolidCharacteristicLength.h"

namespace tl::fea::solid6z::force_detail {
TL_BRICK_HD inline Status Current(const Reference& reference,
    const PrescribedInterval& interval, CurrentGeometry& output) noexcept {
  namespace common = tl::fea::solid_common;
  CurrentGeometry next;
  Vec3 world[6], velocity[6];
  for (unsigned n = 0; n < 6; ++n) {
    world[n] = interval.position_endpoint_m[reference.source_slot(n)];
    velocity[n] = interval.velocity_midpoint_m_s[reference.source_slot(n)];
  }
  const Vec3 embedded[8]{world[0],world[1],world[2],world[2],
                         world[3],world[4],world[5],world[5]};
  if (!common::CyclicFrame(embedded,next.frame)) return Status::InvalidGeometry;
  for (unsigned n = 0; n < 6; ++n) {
    next.local_position_m[n] = common::Local(next.frame,world[n]);
    next.local_velocity_m_s[n] = common::Local(next.frame,velocity[n]);
    if (!common::Finite(next.local_position_m[n]) ||
        !common::Finite(next.local_velocity_m_s[n])) return Status::NonfiniteResult;
  }
  const detail::Jacobian jacobian = detail::EvaluateJacobian(next.local_position_m);
  if (!detail::Finite(jacobian)) return Status::NonfiniteResult;
  if (!common::Positive(jacobian.volume)) return Status::InvalidGeometry;
  StartupGeometry inverse;
  if (!detail::ReferenceInverse(jacobian,inverse)) return Status::NonfiniteResult;
  PointGradients(inverse.inverse_reference_jacobian,next.point_gradient_per_m);
  next.current_volume_m3 = jacobian.volume;
  const auto& x = next.local_position_m;
  const Vec3 local_embedded[8]{x[0],x[1],x[2],x[2],x[3],x[4],x[5],x[5]};
  if (!common::Law42CharacteristicLength(local_embedded,next.current_volume_m3,
                                        next.characteristic_length_m)) return Status::InvalidGeometry;

  double initial_gradient[3][6];
  PointGradients(reference.geometry().inverse_reference_jacobian,initial_gradient);
  Vec3 displacement[6]{};
  const auto& initial = reference.input().position_m;
  const Vec3 initial_last = initial[reference.source_slot(5)];
  for (unsigned n = 0; n < 5; ++n) {
    const Vec3 initial_node = initial[reference.source_slot(n)];
    for (unsigned axis = 0; axis < 3; ++axis) {
      const double saved = common::Component(initial_node,axis)-common::Component(initial_last,axis);
      const double value = common::Component(world[n],axis)-common::Component(world[5],axis)-saved;
      common::SetComponent(displacement[n],axis,value);
    }
  }
  VectorGradient(initial_gradient,displacement,next.world_displacement_gradient);
  if (!common::MaterialGradient(next.frame,next.world_displacement_gradient,
                                next.material_displacement_gradient)) return Status::NonfiniteResult;
  VectorGradient(next.point_gradient_per_m,next.local_velocity_m_s,next.velocity_gradient_per_s);
  EngineeringRate(next.velocity_gradient_per_s,interval.dt_s,next.engineering_rate_per_s);
  for (double value : next.engineering_rate_per_s) {
    if (!tl::math::Finite(value)) return Status::NonfiniteResult;
  }
  output = next;
  return Status::Success;
}
} // namespace tl::fea::solid6z::force_detail
