// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RawGeometry.h"
namespace tlfea::contact::radioss_type25 {
// Exactly ONE native DST3_3/MAINF logical cohort. Resource caps do not
// authorize regrouping repeated history targets across original cohort boundaries.
struct GeometryHistoryBatch {
  const NativeRawGeometryResult* geometry = nullptr;
  std::size_t count = 0;
  // Trial rows AFTER BeginHistory and native selection; no second rollover.
  const NativeGeometryHistory* selected_rows = nullptr;
  std::size_t row_count = 0;
  NativeGeometryHistory* staged_rows = nullptr;
  NativeGeometryFinalResult* results = nullptr;
  NativeGeometryHistory* scratch_rows = nullptr;
  std::size_t scratch_row_capacity = 0;
  NativeGeometryFinalResult* scratch_results = nullptr;
  std::size_t scratch_result_capacity = 0;
};
struct GeometryBatchReport {
  GeometryStatus status = GeometryStatus::InvalidInput;
  std::size_t geometry_index = SIZE_MAX, history_index = SIZE_MAX;
};
namespace geometry_detail {
TL_MATH_HOST_DEVICE inline GeometryBatchForecast Forecast(
    std::size_t count, std::size_t rows, GeometryBatchLimits limits) {
  GeometryBatchForecast out;
  if (count > limits.rows || rows > limits.history_rows ||
      count > SIZE_MAX / sizeof(NativeGeometryFinalResult) ||
      rows > SIZE_MAX / sizeof(NativeGeometryHistory)) {
    out.status = GeometryStatus::CapacityExceeded; return out;
  }
  out.history_scratch_bytes = rows * sizeof(NativeGeometryHistory);
  out.result_scratch_bytes = count * sizeof(NativeGeometryFinalResult);
  if (out.history_scratch_bytes > limits.scratch_bytes ||
      out.result_scratch_bytes > limits.scratch_bytes - out.history_scratch_bytes) {
    out.status = GeometryStatus::CapacityExceeded; return out;
  }
  out.total_scratch_bytes = out.history_scratch_bytes + out.result_scratch_bytes;
  out.status = GeometryStatus::Ok; return out;
}
struct Range { const void* data; std::size_t bytes, alignment; };
TL_MATH_HOST_DEVICE inline bool RangeValid(Range r) {
  const auto address = reinterpret_cast<std::uintptr_t>(r.data);
  return !r.bytes || (r.data && address % r.alignment == 0 && r.bytes <= UINTPTR_MAX - address);
}
TL_MATH_HOST_DEVICE inline bool Disjoint(Range a, Range b) {
  if (!a.bytes || !b.bytes) return true;
  const auto x = reinterpret_cast<std::uintptr_t>(a.data), y = reinterpret_cast<std::uintptr_t>(b.data);
  return x + a.bytes <= y || y + b.bytes <= x;
}
TL_MATH_HOST_DEVICE inline bool Ranges(const GeometryHistoryBatch& b) {
  // Checked half-open ranges as in existing nodal/solid owners; no writes yet.
  const Range ranges[]{
    {b.geometry, b.count * sizeof(NativeRawGeometryResult), alignof(NativeRawGeometryResult)},
    {b.selected_rows, b.row_count * sizeof(NativeGeometryHistory), alignof(NativeGeometryHistory)},
    {b.staged_rows, b.row_count * sizeof(NativeGeometryHistory), alignof(NativeGeometryHistory)},
    {b.results, b.count * sizeof(NativeGeometryFinalResult), alignof(NativeGeometryFinalResult)},
    {b.scratch_rows, b.row_count * sizeof(NativeGeometryHistory), alignof(NativeGeometryHistory)},
    {b.scratch_results, b.count * sizeof(NativeGeometryFinalResult), alignof(NativeGeometryFinalResult)}};
  for (const auto& r : ranges) if (!RangeValid(r)) return false;
  for (unsigned i = 2; i < 6; ++i)
    for (unsigned j = 0; j < i; ++j) if (!Disjoint(ranges[i], ranges[j])) return false;
  return true;
}
TL_MATH_HOST_DEVICE inline bool ValidRow(const NativeContactRow& row) {
  return normal_detail::Valid(row.history.normal) && v::Finite(row.history.previous_force) &&
      v::Finite(row.history.staged_force) && tl::math::Finite(row.penetration_auxiliary) &&
      tl::math::Finite(row.penetration_offset) && tl::math::Finite(row.selection_metric[0]) &&
      tl::math::Finite(row.selection_metric[1]);
}
} // namespace geometry_detail
} // namespace tlfea::contact::radioss_type25
