// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "DeviceFamilies.h"
#include "../../ShellBatchFields.h"

namespace tl::fea::solids::batch_detail {
// Literal original slot expressions. The serial caller adds directly into its
// running diagnostics; the parallel caller records each += operand separately.
template<class Traits, class Cache, class Sum>
TL_BRICK_HD inline void AccumulateMeasurementWork(const typename Traits::Parent& parent,
    const Cache& accepted, const NodalPreparedView& view, Sum& kick, Sum& drift) noexcept {
  for (unsigned n = 0; n < Traits::nodes; ++n) {
    namespace fields = shell_batch_fields;
    const auto node = parent.domain_nodes[n];
    const auto v0 = fields::ReadVector(view.base_kinematics.velocity_xyz, node);
    const auto v1 = fields::ReadVector(view.kinematics.velocity_xyz, node);
    const auto dx = fields::Difference(fields::ReadVector(view.kinematics.position_xyz, node),
        fields::ReadVector(view.base_kinematics.position_xyz, node));
    const auto rhs = accepted.rhs_force_n[n];
    kick += view.kick_dt * fields::Dot(rhs, fields::Mean(v0, v1));
    drift += fields::Dot(rhs, dx);
  }
}
} // namespace tl::fea::solids::batch_detail
