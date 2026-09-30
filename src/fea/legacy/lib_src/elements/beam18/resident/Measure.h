// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
#include "ResultChecks.h"
#include "../../beam_common/EndpointFields.h"

namespace tl::fea::beam18::batch_detail {
struct MeasurementOperands {
  double internal_work_increment_j[2]{},plastic_work_increment_j=0,minimum_unscaled_dt_s=0;
};
TL_BEAM18_HD inline MeasurementOperands Operands(const ForceDiagnostics& value) noexcept {
  return {{value.internal_work_increment_j[0],value.internal_work_increment_j[1]},
      value.plastic_work_increment_j,value.minimum_unscaled_dt_s};
}
TL_BEAM18_HD inline bool AccumulateMeasurementParent(Control& control,const Storage& state,
    unsigned accepted,std::size_t p,const MeasurementOperands& value,
    const NodalPreparedView* view) noexcept {
  auto& d=control.diagnostics;
  const auto& parent=state.parents[p];
    for (unsigned c = 0; c < 2; ++c)
      d.native_internal_work_increment_j[c] += value.internal_work_increment_j[c];
    d.plastic_work_increment_j += value.plastic_work_increment_j;
    d.minimum_native_dt_s = ::fmin(d.minimum_native_dt_s, value.minimum_unscaled_dt_s);
    if (view && !beam_endpoint::AccumulateRhsWork(parent.domain_nodes,
        state.slab[accepted][p].rhs_force_n, state.slab[accepted][p].rhs_couple_nm,
        *view, state.config.owner.fixed_dt, d.internal_kick_work_j, d.internal_drift_work_j)) {
      control.status = BatchStatus::NonfiniteResult;
      control.parent = p;
      return false;
    }
    const double values[]{d.native_internal_work_increment_j[0], d.native_internal_work_increment_j[1],
        d.plastic_work_increment_j, d.internal_kick_work_j, d.internal_drift_work_j};
    for (double value : values) if (!tl::math::Finite(value)) {
      control.status = BatchStatus::NonfiniteResult;
      control.parent = p;
      return false;
    }
  return true;
}
TL_BEAM18_HD inline bool Measure(Storage& state, unsigned accepted, unsigned trial,
    const NodalPreparedView* view) noexcept {
  auto& control = state.control;
  auto& d = control.diagnostics;
  d.parent_count = state.count;
  for (std::size_t p = 0; p < state.count; ++p) {
    const auto& parent = state.parents[p];
    const auto& now = state.slab[trial][p];
    if (state.status[p] || !ValidResult(parent, state.materials[parent.material_index], now, d.time, d.epoch)) {
      control.status = state.status[p] ? BatchStatus::ElementFailure : BatchStatus::NonfiniteResult;
      control.parent = p;
      control.element_status = state.status[p];
      return false;
    }
    if(!AccumulateMeasurementParent(control,state,accepted,p,Operands(now.diagnostics),view))return false;
  }
  return true;
}
} // namespace tl::fea::beam18::batch_detail
