// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "IntervalValues.h"

#if defined(__CUDACC__)
#define TL_WALL_INTERVAL_HD __host__ __device__
#else
#define TL_WALL_INTERVAL_HD
#endif
namespace tlfea::contact::nodal_wall_mapped {
TL_WALL_INTERVAL_HD inline bool ApplyInterval(const IntervalSummary& summary,
    const nodal_wall_device_detail::Storage& storage, const tl::fea::NodalPreparedView& view,
    NodalWallDiagnostics& diagnostics) noexcept {
  using namespace nodal_wall_device_detail;
  if (!IntervalSerialDomain(summary, storage.model.node_count, storage.base.diagnostics,
      diagnostics, view.kick_dt)) return false;
  auto& d = diagnostics;
  const auto& b = storage.base.diagnostics;
  const auto& v = view;
  d.base_potential = b.potential.value;
  d.base_potential_error = b.potential.error;
  d.potential_increment=d.potential.value-b.potential.value;
  d.kick_work += summary.kick_work;
  d.drift_work += summary.drift_work;
  Q4IntegralInterval kick = summary.kick.Get(), drift = summary.drift.Get();
  Q4IntegralInterval moment_y = summary.moment_y.Get(), moment_z = summary.moment_z.Get();
  Q4IntegralInterval potential_delta, defect;
  const double addition_kick = summary.addition_kick, addition_drift = summary.addition_drift;
  const double force_uncertainty = summary.force_uncertainty, quadratic = summary.quadratic;
  if (!Radius(d.kick_work,kick,&d.kick_work_roundoff) ||
      !AddUpper(d.kick_work_roundoff,addition_kick,&d.kick_work_roundoff) ||
      !Radius(d.drift_work,drift,&d.drift_work_roundoff) ||
      !AddUpper(d.drift_work_roundoff,addition_drift,&d.drift_work_roundoff) ||
      !AddUpper(b.potential.error,d.potential.error,&d.work_uncertainty) ||
      !AddUpper(d.work_uncertainty,force_uncertainty,&d.work_uncertainty) ||
      !AddUpper(d.work_uncertainty,d.drift_work_roundoff,&d.work_uncertainty) ||
      !q4_bounds::Difference(d.potential.value,b.potential.value,&potential_delta) ||
      !q4_bounds::Add(potential_delta,{d.drift_work,d.drift_work},&defect))
    return false;
  d.conservative_defect = d.potential_increment+d.drift_work;
  d.quadratic_work_upper = quadratic;
  double arithmetic=0;
  Q4IntegralInterval impulse;
  if (!Radius(d.conservative_defect,defect,&arithmetic) ||
      !AddUpper(d.work_uncertainty,arithmetic,&d.work_uncertainty) ||
      !q4_bounds::Scale({b.resultant.lower,b.resultant.upper},v.kick_dt,&impulse))
    return false;
  d.wall_kick_impulse=v.kick_dt*b.wall_reaction.x;
  d.wall_kick_moment={0,v.kick_dt*b.wall_moment.y,v.kick_dt*b.wall_moment.z};
  if (!Radius(d.wall_kick_impulse,impulse,&d.wall_kick_impulse_error) ||
      !q4_bounds::Scale(moment_y,v.kick_dt,&moment_y) || !q4_bounds::Scale(moment_z,v.kick_dt,&moment_z) ||
      !Radius(d.wall_kick_moment.y,moment_y,&d.wall_kick_moment_error.y) ||
      !Radius(d.wall_kick_moment.z,moment_z,&d.wall_kick_moment_error.z) ||
      !IsFinite(d.kick_work) || !IsFinite(d.drift_work) || !IsFinite(d.potential_increment) ||
      !IsFinite(d.conservative_defect)) return false;
  return true;
}
TL_WALL_INTERVAL_HD inline bool FinalizeInterval(nodal_wall_device_detail::Storage& storage,
    const tl::fea::NodalPreparedView& view, const IntervalSummary& summary,
    bool* tree_used = nullptr) noexcept {
  if (tree_used) *tree_used = false;
  auto diagnostics = storage.result.diagnostics;
  if (!ApplyInterval(summary, storage, view, diagnostics)) {
    return nodal_wall_device_detail::MeasureInterval(storage, view);
  }
  storage.result.diagnostics = diagnostics;
  if (tree_used) *tree_used = true;
  return true;
}
} // namespace tlfea::contact::nodal_wall_mapped
#undef TL_WALL_INTERVAL_HD
