// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Measure.h"
namespace tl::fea::type13::batch_detail {
TL_TYPE13_HD inline Measurement PrepareMeasurement(const Storage& state,
    const Evaluation* accepted, const Evaluation* trial, const NodalPreparedView& view,
    std::size_t e) noexcept {
  Measurement out;
  // Failed Evaluate may leave trial untouched. Never inspect that slot.
  if (state.candidate_status[e] != Status::Success) return out;
  const auto& old = accepted[e];
  const auto& now = trial[e];
  out.active = now.native_history.active;
  out.newly_failed = old.native_history.active && !now.native_history.active;
  out.native_dt = now.stability.critical_dt_s;
  for (unsigned k = 0; k < ChannelCount; ++k) {
    out.work[k] = now.signed_work_J[k];
    out.increment[k] = now.signed_work_J[k] - old.signed_work_J[k];
  }
  connector_measurement::Addends kick{out.kick}, drift{out.drift};
  connector_measurement::AccumulateEndpointWork(state.model.elements[e].nodes,
      old.endpoints, view, state.model.config.owner.fixed_dt, kick, drift);
  return out;
}
TL_TYPE13_HD inline bool MeasurePrepared(const DeviceModel& model,
    const Measurement* values, BatchDiagnostics& diagnostics) noexcept {
  diagnostics.element_count = model.element_count;
  diagnostics.minimum_native_dt_s = values[0].native_dt;
  for (std::size_t e = 0; e < model.element_count; ++e) {
    const auto& now = values[e];
    diagnostics.active_count += now.active;
    diagnostics.newly_failed_count += now.newly_failed;
    for (unsigned k = 0; k < ChannelCount; ++k) {
      diagnostics.internal_work_J[k] += now.work[k];
      diagnostics.internal_work_increment_J[k] += now.increment[k];
    }
    diagnostics.minimum_native_dt_s = ::fmin(diagnostics.minimum_native_dt_s, now.native_dt);
    for (unsigned local = 0; local < 2; ++local) {
      diagnostics.internal_kick_work_J += now.kick[local];
      diagnostics.internal_drift_work_J += now.drift[local];
    }
  }
  for (unsigned k = 0; k < ChannelCount; ++k) {
    if (!tl::math::Finite(diagnostics.internal_work_J[k]) ||
        !tl::math::Finite(diagnostics.internal_work_increment_J[k])) return false;
  }
  return tl::math::Finite(diagnostics.internal_kick_work_J) &&
      tl::math::Finite(diagnostics.internal_drift_work_J) &&
      tl::math::Finite(diagnostics.minimum_native_dt_s) && diagnostics.minimum_native_dt_s > 0;
}
} // namespace tl::fea::type13::batch_detail
