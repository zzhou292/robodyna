// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
#include "../../ShellBatchFields.h"

namespace tl::fea::type13::batch_detail {
TL_TYPE13_HD inline bool Measure(const DeviceModel& model,
                                 const Evaluation* accepted,
                                 const Evaluation* trial,
                                 const NodalPreparedView& view,
                                 BatchDiagnostics& diagnostics) {
  namespace fields = shell_batch_fields;
  diagnostics.element_count = model.element_count;
  diagnostics.minimum_native_dt_s = trial[0].stability.critical_dt_s;
  for (std::size_t e = 0; e < model.element_count; ++e) {
    const auto& old = accepted[e];
    const auto& now = trial[e];
    diagnostics.active_count += now.native_history.active;
    diagnostics.newly_failed_count += old.native_history.active && !now.native_history.active;
    for (unsigned k = 0; k < ChannelCount; ++k) {
      diagnostics.internal_work_J[k] += now.signed_work_J[k];
      diagnostics.internal_work_increment_J[k] += now.signed_work_J[k] - old.signed_work_J[k];
    }
    diagnostics.minimum_native_dt_s = ::fmin(diagnostics.minimum_native_dt_s,
                                             now.stability.critical_dt_s);
    for (unsigned local = 0; local < 2; ++local) {
      const auto node = model.elements[e].nodes[local];
      const auto& rhs = old.endpoints[local];
      const auto v0 = fields::ReadVector(view.base_kinematics.velocity_xyz, node);
      const auto v1 = fields::ReadVector(view.kinematics.velocity_xyz, node);
      const auto w0 = fields::ReadVector(view.base_kinematics.angular_velocity_xyz, node);
      const auto w1 = fields::ReadVector(view.kinematics.angular_velocity_xyz, node);
      const auto dx = fields::Difference(fields::ReadVector(view.kinematics.position_xyz, node),
                                         fields::ReadVector(view.base_kinematics.position_xyz, node));
      const double h = model.config.owner.fixed_dt;
      const Vec3 rotation{h * w1.x, h * w1.y, h * w1.z};
      // Native endpoint cache already uses nodal RHS signs. This measures this
      // contributor only; it is neither total kinetic nor constitutive work.
      diagnostics.internal_kick_work_J += view.kick_dt *
          (fields::Dot(rhs.force_N, fields::Mean(v0, v1)) +
           fields::Dot(rhs.couple_Nm, fields::Mean(w0, w1)));
      diagnostics.internal_drift_work_J += fields::Dot(rhs.force_N, dx) +
                                            fields::Dot(rhs.couple_Nm, rotation);
    }
  }
  for (unsigned k = 0; k < ChannelCount; ++k) {
    if (!tl::math::Finite(diagnostics.internal_work_J[k]) ||
        !tl::math::Finite(diagnostics.internal_work_increment_J[k])) {
      return false;
    }
  }
  return tl::math::Finite(diagnostics.internal_kick_work_J) &&
         tl::math::Finite(diagnostics.internal_drift_work_J) &&
         tl::math::Finite(diagnostics.minimum_native_dt_s) &&
         diagnostics.minimum_native_dt_s > 0;
}
} // namespace tl::fea::type13::batch_detail
