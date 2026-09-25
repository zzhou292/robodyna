// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Prepare.h"
namespace tlfea::contact::radioss_type25::geometry_detail {
// Shared row-local arithmetic. Callers first authenticate keys/ranges; these
// leaves neither publish physical state nor authorize candidate selection.
// GPU owner integration assigns one writer to each secondary history row and
// iterates its complete, ordinal-sorted incidence in these SAME three phases.
TL_MATH_HOST_DEVICE inline void InitialOffset(double time,
    const NativeRawGeometryResult& geometry, NativeContactRow& row) {
  if (time == 0 && geometry.geometric_penetration != 0 && row.irtlm[0] > 0)
    row.penetration_offset = Max(geometry.geometric_penetration, row.penetration_offset);
}
TL_MATH_HOST_DEVICE inline double ShiftOffset(
    const NativeRawGeometryResult& geometry, NativeContactRow& row) {
  const double penetration = geometry.geometric_penetration;
  if (penetration == 0) return penetration; // Native CYCLE preserves signed zero.
  if (row.irtlm[0] < 0) row.penetration_offset = penetration;
  return Max(0., penetration - row.penetration_offset);
}
TL_MATH_HOST_DEVICE inline void StageStiffness(
    const NativeGeometryFinalResult& result, NativeContactRow& row) {
  const double stiffness = result.geometry.incoming_stiffness;
  if (stiffness > 0 && result.penetration == 0)
    row.history.normal.staged_stiffness = Max(row.history.normal.staged_stiffness, stiffness);
}
} // namespace tlfea::contact::radioss_type25::geometry_detail
