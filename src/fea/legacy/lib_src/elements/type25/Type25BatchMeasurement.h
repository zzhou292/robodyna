// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25BatchMeasure.h"
namespace tl::fea::type25::batch_detail {
TL_SURFACE_HD inline Measurement PrepareMeasurement(const Storage& state,
    const Slab& accepted, const Slab& trial, const NodalPreparedView& view, std::size_t e) noexcept {
  Measurement out;
  if (state.candidate_status[e] != Status::Success) return out;
  const auto& old = accepted.element[e];
  const auto& now = trial.element[e];
  out.active = now.history.active;
  out.newly_failed = old.history.active && !now.history.active;
  out.native_dt = now.critical_dt_s;
  for (unsigned c = 0; c < 4; ++c) {
    out.work[c] = now.history.internal_work_J[c];
    out.increment[c] = now.history.internal_work_J[c] - old.history.internal_work_J[c];
  }
  connector_measurement::Addends kick{out.kick}, drift{out.drift};
  connector_measurement::AccumulateEndpointWork(state.model.elements[e].nodes,
      old.endpoints, view, state.model.config.owner.fixed_dt, kick, drift);
  return out;
}
TL_SURFACE_HD inline bool MeasurePrepared(const DeviceModel& model,
    const Measurement* values, BatchDiagnostics& d) noexcept {
  d.element_count = model.config.element_count;
  d.minimum_native_dt = values[0].native_dt;
  for (std::size_t e = 0; e < model.config.element_count; ++e) {
    const auto& now = values[e];
    if (now.active) ++d.active_count;
    if (now.newly_failed) ++d.newly_failed_count;
    for (unsigned c = 0; c < 4; ++c) {
      d.internal_work_J[c] += now.work[c];
      d.internal_work_increment_J[c] += now.increment[c];
    }
    if (now.native_dt < d.minimum_native_dt) d.minimum_native_dt = now.native_dt;
    for (unsigned i = 0; i < 2; ++i) {
      d.internal_kick_work += now.kick[i];
      d.internal_drift_work += now.drift[i];
    }
  }
  for (unsigned c = 0; c < 4; ++c)
    if (!tl::math::Finite(d.internal_work_J[c]) || !tl::math::Finite(d.internal_work_increment_J[c])) return false;
  return tl::math::Finite(d.internal_kick_work) && tl::math::Finite(d.internal_drift_work) &&
      tl::math::Finite(d.minimum_native_dt) && d.minimum_native_dt > 0;
}
} // namespace tl::fea::type25::batch_detail
