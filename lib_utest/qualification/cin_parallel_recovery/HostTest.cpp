// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/solvers/NodalCinStorage.h"
#include <cfloat>
#include <limits>

namespace tl::fea::cin_recovery_test {
TEST(CinRecoveryHost, NewIntervalRebindsEpochWithoutLosingDeformedStateOrCoefficientHistory) {
  auto state = Population(3, true, true);
  const auto assembly_loads = state.loads;
  ASSERT_TRUE(cin_transfer_test::PreparedForce(state));
  state.trial[0] = state.accepted[0]+.000125;
  state.Accept();
  const auto accepted = state.accepted;
  auto stale = state;
  EXPECT_FALSE(cin_advance::force_inputs::CheckPrefix(stale.Input()));
  EXPECT_EQ(stale.control.status, NodalStatus::StaleTrial);
  EXPECT_EQ(stale.control.node, UINT32_MAX);
  packet::SameDoubles(stale.accepted, accepted);

  BeginInterval(state, 7, assembly_loads);
  EXPECT_EQ(state.epoch, 1u);
  EXPECT_EQ(state.attempt, 7u);
  EXPECT_EQ(state.control.rows.base_epoch, 1u);
  EXPECT_EQ(state.control.rows.attempt, 7u);
  EXPECT_EQ(state.control.limit.base_epoch, 1u);
  EXPECT_EQ(state.control.limit.attempt, 7u);
  EXPECT_EQ(state.durations.previous_drift_dt, packet::H);
  EXPECT_EQ(state.durations.kick_dt, packet::H);
  EXPECT_EQ(state.durations.drift_dt, packet::H);
  EXPECT_TRUE(cin_advance::force_inputs::CheckPrefix(state.Input()));
  const auto input = state.Input();
  EXPECT_TRUE(cin::detail::CheckForceInputs(input.model,
      cin_advance::force_inputs::ForceView(input)));
  packet::SameDoubles(state.accepted, accepted);
  packet::SameDoubles(state.trial, accepted);
  packet::SameDoubles(state.loads, assembly_loads);
  for (std::size_t i = 0; i < state.stiffness.size(); ++i)
    EXPECT_EQ(state.work[i], state.stiffness[i]);
  for (const auto value : state.capture) EXPECT_EQ(value, -91);
  EXPECT_EQ(state.input_failure, 19u);
}

TEST(CinRecoveryHost, ReverseIndependentRowsMatchFrozenMotionAcrossBlockBoundaries) {
  for (unsigned count : {1, 2, 127, 128, 129}) {
    SCOPED_TRACE(count);
    auto source = MotionPacket(count);
    std::vector<recovery::Row> rows(count);
    for (unsigned sample = 0; sample < 3; ++sample) {
      auto serial = source, parallel = source;
      if (sample == 1) {
        std::fill(serial.work.begin()+3*packet::Nodes, serial.work.end(), -0.0);
        parallel.work = serial.work;
      }
      if (sample == 2) {
        std::fill(serial.work.begin()+3*packet::Nodes, serial.work.end(), DBL_TRUE_MIN);
        parallel.work = serial.work;
      }
      const auto expected = Serial(serial), actual = Prepared(parallel, rows);
      ASSERT_TRUE(expected);
      cin_input_test::SameReport(expected, actual);
      Same(serial, parallel);
    }
  }
}
TEST(CinRecoveryHost, FirstSourceRowWinsAndFailedOrLaterRowsKeepAllFourSeededFields) {
  for (unsigned failed : {0, 1, 127, 128}) {
    auto source = MotionPacket(129);
    // Reverse source-NID/domain-node ordering without changing source-row order.
    std::swap(source.rows.front().secondary, source.rows.back().secondary);
    source.patches[failed] = {};
    source.patches.back() = {};
    auto serial = source, parallel = source;
    std::vector<recovery::Row> rows(129);
    const auto expected = Serial(serial), actual = Prepared(parallel, rows);
    ASSERT_FALSE(expected);
    EXPECT_EQ(expected.row, failed);
    cin_input_test::SameReport(expected, actual);
    Same(serial, parallel);
    auto retry = MotionPacket(129), reference = retry;
    ASSERT_TRUE(Serial(reference));
    ASSERT_TRUE(Prepared(retry, rows));
    Same(reference, retry);
  }
}
TEST(CinRecoveryHost, SkewedMixedGeometryAndPermutedSourceRowsNeedNoPopulationSpecialCase) {
  for (unsigned count : {3, 17, 71, 130, 200}) {
    auto source = MotionPacket(count, true);
    const auto original_rows = source.rows;
    const auto original_patches = source.patches;
    for (unsigned row = 0; row < count; ++row) {
      const auto donor = row*37%count;
      source.rows[row] = original_rows[donor];
      source.patches[row] = original_patches[donor];
    }
    std::vector<recovery::Row> rows(count);
    for (bool failed : {false, true}) {
      auto serial = source, parallel = source;
      if (failed) serial.patches[count/2] = parallel.patches[count/2] = {};
      const auto expected = Serial(serial), actual = Prepared(parallel, rows);
      EXPECT_EQ(bool(expected), !failed);
      if (failed) EXPECT_EQ(expected.row, count/2);
      cin_input_test::SameReport(expected, actual);
      Same(serial, parallel);
    }
  }
}
TEST(CinRecoveryHost, PureLeavesReadOnlyMastersAndHeaderFailurePreservesOutputs) {
  auto source = MotionPacket(129);
  const auto before = source;
  auto input = source.Input();
  std::vector<recovery::Row> rows(129);
  for (unsigned row = 0; row < rows.size(); ++row) {
    rows[row] = recovery::Prepare(input, row);
    ASSERT_TRUE(rows[row].valid);
  }
  Same(before, source);
  input.prepared_recovery = rows.data();
  recovery::Publish(input, 0, recovery::NoFailure);
  const auto last = recovery::Prepare(input, 128);
  EXPECT_EQ(std::memcmp(&last.motion, &rows.back().motion, sizeof(last.motion)), 0);
  auto trial = recovery::MotionView(input);
  trial.angular_acceleration_xyz = nullptr;
  const auto prior = source;
  EXPECT_FALSE(cin::detail::MotionPointersValid(input.model, trial));
  const auto rejected = cin::RecoverMotionTrial(input.model, trial);
  EXPECT_EQ(rejected.status, cin::StageStatus::InvalidInput);
  EXPECT_EQ(rejected.row, UINT32_MAX);
  Same(prior, source);
}
TEST(CinRecoveryHost, MasterOverflowMatchesOldMappedFailureWithoutPublishingItsRow) {
  auto source = MotionPacket(2);
  for (auto node : source.rows[0].masters) {
    for (unsigned axis = 0; axis < 3; ++axis)
      source.trial[3*packet::Nodes+3*node+axis] = DBL_MAX;
  }
  auto serial = source, parallel = source;
  std::vector<recovery::Row> rows(2);
  const auto expected = Serial(serial), actual = Prepared(parallel, rows);
  ASSERT_FALSE(expected);
  cin_input_test::SameReport(expected, actual);
  Same(serial, parallel);
}
TEST(CinRecoveryHost, SeparateTypedTailAndActualV5ExactCapsAreInclusive) {
  nodal_detail::CinLayout layout;
  NodalCinLimits limits;
  ASSERT_TRUE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
  EXPECT_EQ(layout.prepared_recovery.count, 11165u);
  EXPECT_EQ(layout.prepared_recovery.bytes, 1161160u);
  EXPECT_EQ(layout.prepared_recovery.offset, layout.prepared_transfers.offset+layout.prepared_transfers.bytes);
  EXPECT_EQ(layout.recovery_failure.offset, layout.prepared_recovery.offset+layout.prepared_recovery.bytes);
  EXPECT_EQ(layout.recovery_failure.bytes, 4u);
  EXPECT_EQ(layout.device_bytes, layout.recovery_failure.offset+layout.recovery_failure.bytes);
  EXPECT_EQ(layout.optional_device_bytes, layout.device_bytes+2*layout.state_values*sizeof(double));
  RecordProperty("device_bytes", layout.device_bytes);
  RecordProperty("host_bytes", layout.host_bytes);
  RecordProperty("cin_storage_bytes", sizeof(nodal_detail::CinStorage));
  const auto original = layout;
  limits.max_device_bytes = layout.optional_device_bytes;
  limits.max_host_bytes = layout.host_bytes;
  ASSERT_TRUE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
  --limits.max_device_bytes;
  EXPECT_FALSE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
  EXPECT_EQ(layout.device_bytes, original.device_bytes);
  ++limits.max_device_bytes;
  --limits.max_host_bytes;
  EXPECT_FALSE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
  EXPECT_EQ(layout.host_bytes, original.host_bytes);
  ++limits.max_host_bytes;
  ASSERT_TRUE(layout.Initialize(376930,11165,13173,limits,sizeof(nodal_detail::CinStorage),779));
}
} // namespace tl::fea::cin_recovery_test
