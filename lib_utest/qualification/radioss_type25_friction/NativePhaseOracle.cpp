// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <stdexcept>
namespace type25_friction_test {
extern "C" void rd_type25_phase(const double*, const int*, const double*,
    const int*, const int*, double*, int*, int*);
namespace {
n::HistoryPhaseResult Phase(const n::NativeContactRow& row, n::HistoryPhaseInput in, int mode) {
  // Qualification fixture has32 actual local main entries, all assigned the
  // specified stiffness; production itself does not gain a32-row restriction.
  if (mode == 0 && row.irtlm[0] > 0 && in.secondary_stiffness != 0 &&
      row.irtlm[3] == in.local_processor && (row.irtlm[2] < 1 || row.irtlm[2] > 32))
    throw std::invalid_argument("Native row fixture main index is outside its32-entry array");
  const auto& h = row.history;
  const double values[]{h.normal.previous_penetration, h.normal.previous_stiffness,
      h.normal.staged_penetration, h.normal.staged_stiffness, h.normal.damping_half_force,
      h.previous_force.x, h.previous_force.y, h.previous_force.z,
      h.staged_force.x, h.staged_force.y, h.staged_force.z,
      row.penetration_auxiliary, row.penetration_offset, row.selection_metric[0], row.selection_metric[1]};
  const double stiffness[]{in.secondary_stiffness, in.main_stiffness};
  double output[15]{}; int marker[4]{}, kept = 0;
  rd_type25_phase(values, row.irtlm, stiffness, &in.local_processor, &mode, output, marker, &kept);
  n::HistoryPhaseResult result;
  auto& next = result.row;
  next.history.normal = {output[0], output[1], output[2], output[3], output[4]};
  next.history.previous_force = {output[5], output[6], output[7]};
  next.history.staged_force = {output[8], output[9], output[10]};
  next.penetration_auxiliary = output[11]; next.penetration_offset = output[12];
  next.selection_metric[0] = output[13]; next.selection_metric[1] = output[14];
  for (unsigned i = 0; i < 4; ++i) next.irtlm[i] = marker[i];
  result.retained_candidate = kept != 0;
  return result;
}
}
n::HistoryPhaseResult BeginOracle(const n::NativeContactRow& row, n::HistoryPhaseInput input) {
  return Phase(row, input, 0);
}
n::NativeContactRow EndOracle(const n::NativeContactRow& row) {
  return Phase(row, {1., 1., 1}, 1).row;
}
} // namespace type25_friction_test
