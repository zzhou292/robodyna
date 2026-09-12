// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ShellBatchFields.h"
#include "../../math/Quaternion.h"

namespace tl::fea::beam_endpoint {
struct Motion {
  tl::math::Vec3 position[2]{}, velocity[2]{}, angular_velocity[2]{};
};
// Exactly two mechanical endpoints. An orientation-only node is never gathered.
TL_SURFACE_HD inline bool Gather(const std::size_t (&nodes)[2],
    DeviceNodalKinematicsView view, Motion& output) noexcept {
  if (!view.position_xyz || !view.velocity_xyz || !view.angular_velocity_xyz ||
      !view.orientation_wxyz || nodes[0] == nodes[1]) return false;
  Motion next;
  for (unsigned n = 0; n < 2; ++n) {
    const auto node = nodes[n];
    if (node >= view.node_count) return false;
    next.position[n] = shell_batch_fields::ReadVector(view.position_xyz, node);
    next.velocity[n] = shell_batch_fields::ReadVector(view.velocity_xyz, node);
    next.angular_velocity[n] = shell_batch_fields::ReadVector(view.angular_velocity_xyz, node);
    const auto* q = view.orientation_wxyz + 4 * node;
    if (!shell_batch_fields::FiniteVector(next.position[n]) ||
        !shell_batch_fields::FiniteVector(next.velocity[n]) ||
        !shell_batch_fields::FiniteVector(next.angular_velocity[n]) ||
        !tl::math::UnitQuaternion({q[0], q[1], q[2], q[3]})) return false;
  }
  output = next;
  return true;
}
// RHS-signed cache work, in endpoint/source order. This is a contributor
// observation, not material energy. h is the supplied owner interval.
TL_SURFACE_HD inline bool AccumulateRhsWork(const std::size_t (&nodes)[2],
    const tl::math::Vec3 (&force)[2], const tl::math::Vec3 (&couple)[2],
    const NodalPreparedView& view, double h, double& kick, double& drift) noexcept {
  Motion base, current;
  if (!Gather(nodes, view.base_kinematics, base) || !Gather(nodes, view.kinematics, current) ||
      !tl::math::Finite(h) || h <= 0 || !tl::math::Finite(view.kick_dt) || view.kick_dt <= 0)
    return false;
  double next_kick = kick, next_drift = drift;
  for (unsigned n = 0; n < 2; ++n) {
    namespace fields = shell_batch_fields;
    const auto w = current.angular_velocity[n];
    const tl::math::Vec3 rotation{h * w.x, h * w.y, h * w.z};
    const auto dx = fields::Difference(current.position[n], base.position[n]);
    next_kick += view.kick_dt * (fields::Dot(force[n], fields::Mean(base.velocity[n], current.velocity[n])) +
        fields::Dot(couple[n], fields::Mean(base.angular_velocity[n], current.angular_velocity[n])));
    next_drift += fields::Dot(force[n], dx) + fields::Dot(couple[n], rotation);
    if (!tl::math::Finite(next_kick) || !tl::math::Finite(next_drift)) return false;
  }
  kick = next_kick;
  drift = next_drift;
  return true;
}
} // namespace tl::fea::beam_endpoint
