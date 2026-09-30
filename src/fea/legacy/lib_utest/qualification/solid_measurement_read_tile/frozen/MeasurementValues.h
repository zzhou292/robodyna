// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Measure.h"

namespace tl::fea::solids::batch_detail {
struct MeasurementAddends {
  double* values;
  unsigned next = 0;
  TL_BRICK_HD void operator+=(double value) noexcept { values[next++] = value; }
};

template<class Traits>
TL_BRICK_HD inline void PrepareMeasurementOperands(Storage& state, unsigned accepted,
    unsigned trial, std::size_t parent_index, double time, std::uint64_t epoch,
    const NodalPreparedView* view) noexcept {
  auto& family = FamilyStorage<Traits>(state);
  MeasurementOperands<Traits::nodes> next;
  const auto valid = CheckParentResult<Traits>(state, trial, parent_index, time, epoch);
  family.result_valid[parent_index] = valid;
  // A failed update or invalid result must not dereference unavailable trial,
  // accepted, material, parent, or view inputs beyond the existing predicate.
  if (valid == 1) {
    const auto& cache = family.slab[trial][parent_index].cache;
    next.work = Work(cache);
    next.hourglass_work = HourglassWork(cache);
    next.plastic_work = PlasticWork(cache);
    next.native_dt = NativeDt(cache);
    if (view) {
      MeasurementAddends kick{next.kick}, drift{next.drift};
      AccumulateMeasurementWork<Traits>(family.parents[parent_index],
          family.slab[accepted][parent_index].cache, *view, kick, drift);
    }
  }
  // Do not reject nonfinite derived operands here. The original parent fold
  // performs its finite check only after all that parent's diagnostic writes.
  family.measurement[parent_index] = next;
}

template<class Traits>
TL_BRICK_HD inline bool MeasureOperandFamily(Storage& state, Control& control,
    unsigned family_index, const NodalPreparedView* view) noexcept {
  auto& family = FamilyStorage<Traits>(state);
  auto& diagnostics = control.diagnostics;
  diagnostics.parent_count[family_index] = family.count;
  for (std::size_t p = 0; p < family.count; ++p) {
    if (family.status[p] != 0 || family.result_valid[p] != 1) {
      control.status = family.status[p] ? BatchStatus::ElementFailure : BatchStatus::NonfiniteResult;
      control.family = Traits::family;
      control.parent = p;
      control.element_status = family.status[p];
      return false;
    }
    const auto& value = family.measurement[p];
    diagnostics.native_internal_work_increment_j[family_index] += value.work;
    diagnostics.physical_hourglass_work_increment_j[family_index] += value.hourglass_work;
    diagnostics.plastic_work_increment_j += value.plastic_work;
    if (value.native_dt < diagnostics.minimum_native_dt_s)
      diagnostics.minimum_native_dt_s = value.native_dt;
    if (view) {
      for (unsigned n = 0; n < Traits::nodes; ++n) {
        diagnostics.internal_kick_work_j += value.kick[n];
        diagnostics.internal_drift_work_j += value.drift[n];
      }
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

// Retained callers keep the existing state.control result location.
template<class Traits>
TL_BRICK_HD inline bool MeasureOperandFamily(Storage& state, unsigned family_index,
    const NodalPreparedView* view) noexcept {
  return MeasureOperandFamily<Traits>(state, state.control, family_index, view);
}
template<class Traits>
TL_BRICK_HD inline bool MeasureFinalFamily(Storage& state, Control& control,
    unsigned accepted, unsigned trial, unsigned family_index,
    const NodalPreparedView* view, bool operands_prepared) noexcept {
  return operands_prepared ? MeasureOperandFamily<Traits>(state, control, family_index, view)
      : MeasureValidatedFamily<Traits>(state, control, accepted, trial, family_index, view);
}
template<class Traits>
TL_BRICK_HD inline bool MeasureFinalFamily(Storage& state, unsigned accepted, unsigned trial,
    unsigned family_index, const NodalPreparedView* view, bool operands_prepared) noexcept {
  return MeasureFinalFamily<Traits>(state, state.control, accepted, trial,
      family_index, view, operands_prepared);
}
} // namespace tl::fea::solids::batch_detail
