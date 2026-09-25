// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/GeometryTypes.h"
#include <vector>
namespace type25_geometry_test {
namespace n = tlfea::contact::radioss_type25;
// Qualification only: complete original current-geometry expressions, with
// incoming stiffness and selected barycentric operands supplied by the caller.
n::NativeRawGeometryResult OracleRaw(const n::GeometryProfile&,
                                    const n::NativeGeometryInput&);
struct HistoryOracleResult {
  std::vector<n::NativeGeometryFinalResult> results;
  std::vector<n::NativeGeometryHistory> rows;
};
// One original logical cohort. The bounded wrapper admits at most 256 geometry
// entries and 256 history slots; repeated target slots retain native row order.
HistoryOracleResult OracleHistory(const n::GeometryProfile&, double time,
    const std::vector<n::NativeRawGeometryResult>&,
    const std::vector<n::NativeGeometryHistory>&);
} // namespace type25_geometry_test
