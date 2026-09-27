// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "DeviceFamilies.h"
#include "ResultValidation.h"
#include "MeasurementWork.h"

namespace tl::fea::solids::batch_detail {
TL_BRICK_HD inline double NativeDt(const Cache18& c) noexcept {
  return c.diagnostics.minimum_unscaled_dt_s;
}
TL_BRICK_HD inline double NativeDt(const Cache24& c) noexcept {
  return c.diagnostics.material.unscaled_element_dt_s;
}
TL_BRICK_HD inline double NativeDt(const Cache6z& c) noexcept {
  return c.material.unscaled_element_dt_s;
}
TL_BRICK_HD inline double Work(const Cache18& c) noexcept { return c.diagnostics.internal_work_increment_j; }
TL_BRICK_HD inline double Work(const Cache24& c) noexcept {
  return c.diagnostics.material.internal_work_j + c.diagnostics.stabilization_work_j;
}
TL_BRICK_HD inline double Work(const Cache6z& c) noexcept { return c.total_internal_work_increment_j; }
TL_BRICK_HD inline double HourglassWork(const Cache18&) noexcept { return 0; }
TL_BRICK_HD inline double HourglassWork(const Cache24& c) noexcept { return c.diagnostics.stabilization_work_j; }
TL_BRICK_HD inline double HourglassWork(const Cache6z& c) noexcept {
  return c.stabilization.first_work_j + c.stabilization.second_work_j;
}
TL_BRICK_HD inline double PlasticWork(const Cache18& c) noexcept { return c.diagnostics.plastic_work_increment_j; }
TL_BRICK_HD inline double PlasticWork(const Cache24&) noexcept { return 0; }
TL_BRICK_HD inline double PlasticWork(const Cache6z&) noexcept { return 0; }
TL_BRICK_HD inline double NativeDt(const Cache18Law44& c) noexcept { return c.diagnostics.minimum_unscaled_dt_s; }
TL_BRICK_HD inline double NativeDt(const Cache18Law90& c) noexcept { return c.diagnostics.minimum_unscaled_dt_s; }
TL_BRICK_HD inline double Work(const Cache18Law44& c) noexcept { return c.diagnostics.internal_work_increment_j; }
TL_BRICK_HD inline double Work(const Cache18Law90& c) noexcept { return c.diagnostics.internal_work_increment_j; }
TL_BRICK_HD inline double HourglassWork(const Cache18Law44&) noexcept { return 0; }
TL_BRICK_HD inline double HourglassWork(const Cache18Law90&) noexcept { return 0; }
TL_BRICK_HD inline double PlasticWork(const Cache18Law44& c) noexcept { return c.diagnostics.plastic_work_increment_j; }
TL_BRICK_HD inline double PlasticWork(const Cache18Law90&) noexcept { return 0; }
template<class Traits, class Check>
TL_BRICK_HD inline bool MeasureFamilyWithCheck(Storage& state, Control& control,
    unsigned accepted, unsigned trial, unsigned family_index,
    const NodalPreparedView* view, Check check) noexcept {
  auto& family = FamilyStorage<Traits>(state);
  auto& diagnostics = control.diagnostics;
  diagnostics.parent_count[family_index] = family.count;
  for (std::size_t p = 0; p < family.count; ++p) {
    const auto& parent = family.parents[p];
    const auto& now = family.slab[trial][p];
    if (family.status[p] != 0 || !check(p, diagnostics.time, diagnostics.epoch)) {
      control.status = family.status[p] ? BatchStatus::ElementFailure : BatchStatus::NonfiniteResult;
      control.family = Traits::family;
      control.parent = p;
      control.element_status = family.status[p];
      return false;
    }
    const auto& cache = now.cache;
    diagnostics.native_internal_work_increment_j[family_index] += Work(cache);
    diagnostics.physical_hourglass_work_increment_j[family_index] += HourglassWork(cache);
    diagnostics.plastic_work_increment_j += PlasticWork(cache);
    if (NativeDt(cache) < diagnostics.minimum_native_dt_s)
      diagnostics.minimum_native_dt_s = NativeDt(cache);
    if (view) {
      AccumulateMeasurementWork<Traits>(parent, family.slab[accepted][p].cache, *view,
          diagnostics.internal_kick_work_j, diagnostics.internal_drift_work_j);
    }
    const double finite[]{diagnostics.native_internal_work_increment_j[family_index],
        diagnostics.physical_hourglass_work_increment_j[family_index], diagnostics.plastic_work_increment_j,
        diagnostics.internal_kick_work_j, diagnostics.internal_drift_work_j};
    if (!FiniteValues(finite)) {
      control.status = BatchStatus::NonfiniteResult;
      control.family = Traits::family;
      control.parent = p;
      return false;
    }
  }
  return true;
}
// Retained callers keep their original destination and ordered fold.
template<class Traits, class Check>
TL_BRICK_HD inline bool MeasureFamilyWithCheck(Storage& state, unsigned accepted, unsigned trial,
    unsigned family_index, const NodalPreparedView* view, Check check) noexcept {
  return MeasureFamilyWithCheck<Traits>(state, state.control, accepted, trial,
      family_index, view, check);
}
template<class Traits>
TL_BRICK_HD inline bool MeasureFamily(Storage& state, unsigned accepted, unsigned trial,
    unsigned family_index, const NodalPreparedView* view) noexcept {
  return MeasureFamilyWithCheck<Traits>(state, accepted, trial, family_index, view,
      DirectResultCheck<Traits>{state, trial});
}
template<class Traits>
TL_BRICK_HD inline bool MeasureValidatedFamily(Storage& state, Control& control,
    unsigned accepted, unsigned trial, unsigned family_index,
    const NodalPreparedView* view) noexcept {
  return MeasureFamilyWithCheck<Traits>(state, control, accepted, trial, family_index, view,
      StagedResultCheck{FamilyStorage<Traits>(state).result_valid});
}
template<class Traits>
TL_BRICK_HD inline bool MeasureValidatedFamily(Storage& state, unsigned accepted, unsigned trial,
    unsigned family_index, const NodalPreparedView* view) noexcept {
  return MeasureValidatedFamily<Traits>(state, state.control, accepted, trial,
      family_index, view);
}
} // namespace tl::fea::solids::batch_detail
