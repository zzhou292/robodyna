// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_parallel_recovery/Fixture.h"
#include "lib_src/solvers/cin_advance/DriftValues.h"
#include "FrozenDrift.h"
#include <cfloat>

namespace tl::fea::cin_drift_test {
namespace packet = cin_parallel_test;
namespace drift = cin_advance::drift;
namespace recovery = cin_advance::recovery;
inline void SameWitness(const cin_limiter::Witness& a, const cin_limiter::Witness& b) {
  EXPECT_EQ(a.epoch, b.epoch);
  EXPECT_EQ(a.attempt, b.attempt);
  const auto& x = a.values;
  const auto& y = b.values;
  EXPECT_EQ(x.kind, y.kind);
  EXPECT_EQ(x.node, y.node);
  EXPECT_EQ(x.group, y.group);
  EXPECT_EQ(x.translation_fixed_bits, y.translation_fixed_bits);
  EXPECT_EQ(x.rotation_fixed, y.rotation_fixed);
  EXPECT_EQ(x.rotation_present, y.rotation_present);
  packet::SameDoubles({x.minimum_dt_s, x.factor, x.mass_kg, x.inertia_kg_m2,
      x.translation_stiffness_n_per_m, x.rotation_stiffness_nm,
      x.principal_inertia_kg_m2.x, x.principal_inertia_kg_m2.y, x.principal_inertia_kg_m2.z,
      x.trace_upper_per_s2},
      {y.minimum_dt_s, y.factor, y.mass_kg, y.inertia_kg_m2,
      y.translation_stiffness_n_per_m, y.rotation_stiffness_nm,
      y.principal_inertia_kg_m2.x, y.principal_inertia_kg_m2.y, y.principal_inertia_kg_m2.z,
      y.trace_upper_per_s2});
}
inline void Same(const packet::Packet& a, const packet::Packet& b) {
  cin_recovery_test::Same(a, b);
  SameWitness(a.control.structural_limiter, b.control.structural_limiter);
}
inline packet::Packet DriftPacket(unsigned count, bool deformed = false) {
  auto result = cin_recovery_test::MotionPacket(count, deformed);
  if (!cin_recovery_test::Serial(result)) throw std::runtime_error("Drift packet recovery failed");
  for (unsigned node = 0; node < packet::Nodes; ++node) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      const auto index = 3*node+axis;
      result.trial[13*packet::Nodes+index] = -81-axis;
      result.trial[16*packet::Nodes+index] = 82+axis;
    }
  }
  return result;
}
inline void Prepared(packet::Packet& state, std::vector<drift::Row>& rows,
    recovery::FailureRow& failure) {
  auto input = state.Input();
  input.prepared_drift = rows.data();
  input.recovery_failure = &failure;
  if (input.control->status != NodalStatus::Ok) return;
  failure = recovery::NoFailure;
  for (unsigned i = 0; i < rows.size(); ++i) {
    const auto row = unsigned(rows.size()-1-i);
    rows[row] = drift::Prepare(input, row);
    if (rows[row].status != NodalStatus::Ok) failure = std::min(failure, row);
  }
  for (unsigned i = 0; i < rows.size(); ++i)
    drift::Publish(input, unsigned(rows.size()-1-i), failure);
  drift::Complete(input);
}
inline void FailPosition(packet::Packet& state, unsigned row, unsigned axis) {
  const auto index = 3*state.rows[row].secondary+axis;
  state.durations.drift_dt = 1;
  state.accepted[index] = DBL_MAX;
  state.trial[3*packet::Nodes+index] = DBL_MAX;
  // Keep all preceding orientation increments within their admitted bound.
  std::fill(state.trial.begin()+6*packet::Nodes, state.trial.begin()+9*packet::Nodes, .001);
}
cudaError_t LaunchFrozen(const cin_advance::Input&, cudaStream_t);
} // namespace tl::fea::cin_drift_test
