// SPDX-License-Identifier: AGPL-3.0-or-later
// I25DST3_3 two-pass offsets + I25MAINF zero-PENE stiffness staging.
#pragma once
#include "geometry/HistoryChecks.h"
#include "geometry/HistoryRow.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline GeometryBatchForecast PreflightNativeGeometryHistory(
    std::size_t count, std::size_t rows, GeometryBatchLimits limits = {}) {
  return geometry_detail::Forecast(count, rows, limits);
}
namespace geometry_detail {
TL_MATH_HOST_DEVICE inline GeometryBatchReport FinalizeHistory(
    const GeometryProfile& profile, double time, GeometryHistoryBatch batch,
    GeometryBatchLimits limits) {
  if (!normal_detail::Nonnegative(time)) return {GeometryStatus::InvalidInput};
  if (!Supported(profile)) return {GeometryStatus::UnsupportedProfile};
  const auto forecast = Forecast(batch.count, batch.row_count, limits);
  if (forecast.status != GeometryStatus::Ok) return {forecast.status};
  if (batch.scratch_row_capacity < batch.row_count ||
      batch.scratch_result_capacity < batch.count) return {GeometryStatus::CapacityExceeded};
  if (!Ranges(batch)) return {GeometryStatus::InvalidInput};
  for (std::size_t i = 0; i < batch.row_count; ++i)
    if (!batch.selected_rows[i].secondary_source_id || !ValidRow(batch.selected_rows[i].row))
      return {GeometryStatus::InvalidInput, SIZE_MAX, i};
  for (std::size_t i = 0; i < batch.count; ++i) {
    const auto& g = batch.geometry[i];
    if (g.key.history_index >= batch.row_count || !Finite(g))
      return {GeometryStatus::InvalidInput, i, g.key.history_index};
    const auto& r = batch.selected_rows[g.key.history_index];
    const auto main = std::int64_t(r.row.irtlm[0]);
    const auto magnitude = main < 0 ? -main : main;
    int sector = g.selection_code % 5; if (sector < 0) sector = -sector;
    if (g.key.secondary_source_id != r.secondary_source_id || g.key.generation != r.generation ||
        g.key.main_segment <= 0 || magnitude != g.key.main_segment ||
        g.selection_code != r.row.irtlm[1] || sector < 1 || sector > 4)
      return {GeometryStatus::InvalidInput, i, g.key.history_index};
  }
  for (std::size_t i = 0; i < batch.row_count; ++i) batch.scratch_rows[i] = batch.selected_rows[i];
  // Every TIME0 retained-contact maximum precedes ANY offset subtraction.
  if (time == 0)
    for (std::size_t i = 0; i < batch.count; ++i) {
      const auto& g = batch.geometry[i];
      auto& row = batch.scratch_rows[g.key.history_index].row;
      InitialOffset(time, g, row);
    }
  // Second pass keeps original native row order, including repeated targets.
  for (std::size_t i = 0; i < batch.count; ++i) {
    const auto& g = batch.geometry[i];
    auto& row = batch.scratch_rows[g.key.history_index].row;
    NativeGeometryFinalResult result; result.geometry = g;
    result.penetration = ShiftOffset(g, row);
    if (!normal_detail::Nonnegative(result.penetration) || !ValidRow(row))
      return {GeometryStatus::NonfiniteResult, i, g.key.history_index};
    batch.scratch_results[i] = result;
  }
  // Separate MAINF pass follows completion of every geometry offset update.
  for (std::size_t i = 0; i < batch.count; ++i) {
    const auto& result = batch.scratch_results[i];
    const auto& g = result.geometry;
    auto& row = batch.scratch_rows[g.key.history_index].row;
    StageStiffness(result, row);
  }
  for (std::size_t i = 0; i < batch.row_count; ++i) batch.staged_rows[i] = batch.scratch_rows[i];
  for (std::size_t i = 0; i < batch.count; ++i) batch.results[i] = batch.scratch_results[i];
  return {GeometryStatus::Ok};
}
} // namespace geometry_detail
// Bounded serial reference/qualification adapter, NOT a million-row GPU
// production kernel. A retained CUDA owner uses ordered secondary-row incidence
// and one writer per history row with geometry_detail::HistoryRow phase leaves.
TL_MATH_HOST_DEVICE inline GeometryBatchReport FinalizeNativeGeometryHistory(
    const GeometryProfile& p, double native_time, GeometryHistoryBatch b,
    GeometryBatchLimits limits = {}) {
  return geometry_detail::FinalizeHistory(p, native_time, b, limits);
}
} // namespace tlfea::contact::radioss_type25
