// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/solid_candidate_validation/TestSupport.h"
#include "lib_src/elements/solids/resident/MeasurementValues.h"

namespace solid_operands_test {
using namespace solid_validation_test;

template<class Traits> void Stage(d::Storage& state, unsigned accepted, unsigned trial,
    const fe::NodalPreparedView* view, double time = 0, std::uint64_t epoch = 0) {
  const auto count = d::FamilyStorage<Traits>(state).count;
  for (std::size_t p = 0; p < count; ++p)
    d::PrepareMeasurementOperands<Traits>(state, accepted, trial, p, time, epoch, view);
}
inline void StageAll(d::Storage& state, const fe::NodalPreparedView* view = nullptr) {
  Stage<d::Traits18>(state, 0, 1, view);
  Stage<d::Traits24>(state, 0, 1, view);
  Stage<d::Traits6z>(state, 0, 1, view);
  Stage<d::Traits18Law44>(state, 0, 1, view);
  Stage<d::Traits18Law90>(state, 0, 1, view);
}
inline bool FoldAll(d::Storage& state, const fe::NodalPreparedView* view = nullptr) {
  return d::MeasureOperandFamily<d::Traits18>(state, 0, view) &&
      d::MeasureOperandFamily<d::Traits24>(state, 1, view) &&
      d::MeasureOperandFamily<d::Traits6z>(state, 2, view) &&
      d::MeasureOperandFamily<d::Traits18Law44>(state, 3, view) &&
      d::MeasureOperandFamily<d::Traits18Law90>(state, 4, view);
}
inline void CompareAll(HostRig& rig, const fe::NodalPreparedView* view = nullptr,
    double seed = 0) {
  StageAll(rig.state, view);
  Reset(rig.state);
  rig.state.control.diagnostics.internal_kick_work_j = seed;
  rig.state.control.diagnostics.internal_drift_work_j = -seed;
  rig.state.control.diagnostics.plastic_work_increment_j = seed;
  const auto original = rig.state.control;
  const bool expected = MeasureAll(rig.state, true, 0, 1, view);
  const auto control = rig.state.control;
  rig.state.control = original;
  // Final fold needs only the non-null marker; no accepted or view readback.
  const auto* marker = view ? reinterpret_cast<const fe::NodalPreparedView*>(1) : nullptr;
  EXPECT_EQ(FoldAll(rig.state, marker), expected);
  SameControl(rig.state.control, control);
}
template<class Traits> struct MeasuredRows : Rows<Traits> {
  std::vector<d::MeasurementOperands<Traits::nodes>> operands;
  MeasuredRows(d::Storage& state, std::size_t count)
      : Rows<Traits>(state, count), operands(count) {
    d::FamilyStorage<Traits>(state).measurement = operands.data();
  }
};
} // namespace solid_operands_test
