// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/solvers/NodalCinStorage.h"
#include <cfloat>

namespace tl::fea::cin_transfer_test {
TEST(CinPreparedForceRows, ReverseLeafOrderMatchesCompleteSerialAcrossSharedMastersAndRepeatedSlots) {
  for (unsigned count : {1,2,127,128,129}) for (bool zero_entry_inertia : {false,true}) {
    SCOPED_TRACE(count);
    auto serial = Population(count);
    if (zero_entry_inertia) serial.accepted[serial.TailOffset()+packet::Nodes+serial.rows[0].masters[0]] = 0;
    auto prepared = serial;
    for (unsigned step = 0; step < 3; ++step) {
      serial.Begin(step+1); prepared.Begin(step+1);
      cin_input_test::Seed(serial); cin_input_test::Seed(prepared);
      const auto a = FrozenForce(serial), b = PreparedForce(prepared);
      ASSERT_TRUE(a) << a.row << ' ' << a.node;
      cin_input_test::SameReport(a, b);
      SameForce(serial, prepared);
      serial.accepted = serial.trial;
      prepared.accepted = prepared.trial;
    }
  }
}
TEST(CinPreparedForceRows, EarlierApplyFailureWinsOverLaterPreparedLeafFailureWithExactPartialFields) {
  for (unsigned fault = 0; fault < 3; ++fault) {
    auto serial = Population(2);
    CenterFirstSecondary(serial);
    LatePatch(serial);
    const auto secondary = serial.rows[0].secondary;
    if (fault == 0) {
      serial.loads[serial.rows[0].masters[0]] = DBL_MAX;
      serial.loads[secondary] = DBL_MAX/8;
      for (unsigned axis = 1; axis < 6; ++axis) serial.loads[axis*packet::Nodes+secondary] = 0;
    } else if (fault == 1) {
      serial.trial.back() = DBL_MAX;
      serial.trial[serial.TailOffset()+secondary] = DBL_MAX/2;
    }
    auto prepared = serial;
    const auto a = FrozenForce(serial), b = PreparedForce(prepared);
    ASSERT_FALSE(a);
    EXPECT_EQ(a.row, fault == 2 ? 1u : 0u);
    if (fault == 0) EXPECT_EQ(a.node, serial.rows[0].masters[0]);
    if (fault == 1) EXPECT_EQ(a.node, secondary);
    cin_input_test::SameReport(a, b);
    SameForce(serial, prepared);
    auto retry = Population(2), reference = retry;
    ASSERT_TRUE(FrozenForce(reference));
    ASSERT_TRUE(PreparedForce(retry));
    SameForce(reference, retry);
  }
}
TEST(CinPreparedForceRows, InputPriorityStillPrecedesAnyPreparationOrEntryInertiaWrite) {
  for (auto fault : {cin_input_test::Fault::NodeBeforeWitness, cin_input_test::Fault::NumericalBeforeWitness,
                    cin_input_test::Fault::MissingActivity, cin_input_test::Fault::LateRow}) {
    auto serial = Population(2);
    auto prepared = serial;
    cin_input_test::Inject(serial, fault);
    cin_input_test::Inject(prepared, fault);
    const auto before = prepared.work;
    const auto a = FrozenForce(serial), b = PreparedForce(prepared);
    ASSERT_FALSE(a);
    cin_input_test::SameReport(a, b);
    SameForce(serial, prepared);
    packet::SameDoubles(prepared.work, before);
  }
}
TEST(CinPreparedForceRows, LeavesDoNotMutateAnyTransferDestinationOrPriorPatch) {
  auto p = Population(129);
  auto input = p.Input();
  const auto force = cin_advance::force_inputs::ForceView(input);
  for (unsigned node = 0; node < packet::Nodes; ++node) force.entry_inertia[node] = force.inertia[node];
  const auto before = p;
  std::vector<transfer::Row> rows(p.rows.size());
  for (unsigned row = 0; row < rows.size(); ++row) {
    rows[row].report = cin::detail::PrepareForceRow(input.model, force, row, rows[row]);
    ASSERT_TRUE(rows[row].report);
  }
  SameForce(p, before);
  // Earlier application changes common master loads/M/J/STI, never a later
  // leaf's secondary inputs or its entry-IN snapshot.
  ASSERT_TRUE(cin::detail::ApplyForceRow(input.model, force, 0, rows[0]));
  transfer::Row recomputed;
  recomputed.report = cin::detail::PrepareForceRow(input.model, force, rows.size()-1, recomputed);
  const auto& old = rows.back();
  cin_input_test::SameReport(recomputed.report, old.report);
  packet::SameDoubles({recomputed.secondary_mass, recomputed.transferred_coefficients.master[0].mass,
      recomputed.transferred_coefficients.master[0].inertia, recomputed.transferred_coefficients.master[0].translational_stiffness},
      {old.secondary_mass, old.transferred_coefficients.master[0].mass,
       old.transferred_coefficients.master[0].inertia, old.transferred_coefficients.master[0].translational_stiffness});
  for (unsigned slot = 0; slot < 4; ++slot) {
    const auto a = recomputed.transferred_load.force[slot], b = old.transferred_load.force[slot];
    packet::SameDoubles({a.x,a.y,a.z}, {b.x,b.y,b.z});
  }
}
TEST(CinPreparedForceRows, ActualV5TailAndExactCapsAreCountedBeforeAllocation) {
  nodal_detail::CinLayout layout;
  NodalCinLimits limits;
  ASSERT_TRUE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
  EXPECT_EQ(sizeof(transfer::Row), 512u);
  EXPECT_EQ(layout.prepared_transfers.count, 11165u);
  EXPECT_EQ(layout.prepared_transfers.bytes, 5716480u);
  EXPECT_EQ(layout.prepared_transfers.offset, layout.group_reports.offset+layout.group_reports.bytes);
  EXPECT_EQ(layout.prepared_recovery.offset, layout.prepared_transfers.offset+layout.prepared_transfers.bytes);
  EXPECT_EQ(layout.prepared_recovery.bytes, 1161160u);
  EXPECT_EQ(layout.recovery_failure.offset, layout.prepared_recovery.offset+layout.prepared_recovery.bytes);
  EXPECT_EQ(layout.recovery_failure.bytes, 4u);
  const auto prior_end = layout.recovery_failure.offset+layout.recovery_failure.bytes;
  EXPECT_EQ(layout.prepared_drift.offset, (prior_end+7u)/8u*8u);
  EXPECT_EQ(layout.prepared_drift.count, layout.attachments);
  EXPECT_EQ(layout.prepared_drift.bytes, 64u*layout.attachments);
  EXPECT_EQ(layout.device_bytes, layout.prepared_drift.offset+layout.prepared_drift.bytes);
  EXPECT_EQ(layout.optional_device_bytes, layout.device_bytes+2*layout.state_values*sizeof(double));
  const auto before = layout;
  limits.max_device_bytes = layout.optional_device_bytes;
  limits.max_host_bytes = layout.host_bytes;
  ASSERT_TRUE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
  --limits.max_device_bytes;
  EXPECT_FALSE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
  EXPECT_EQ(layout.device_bytes, before.device_bytes);
  ++limits.max_device_bytes;
  --limits.max_host_bytes;
  EXPECT_FALSE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
  EXPECT_EQ(layout.host_bytes, before.host_bytes);
}
} // namespace tl::fea::cin_transfer_test
