// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/solvers/NodalCinStorage.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace tl::fea::cin_drift_test {
TEST(CinDriftHost, ReverseWorkersMatchFrozenRowsWithGeneralGeometryAndOrdering) {
  for (unsigned count : {1, 2, 3, 17, 71, 127, 128, 129, 130, 200}) {
    for (bool deformed : {false, true}) {
      SCOPED_TRACE(count);
      SCOPED_TRACE(deformed);
      auto source = DriftPacket(count, deformed);
      std::reverse(source.rows.begin(), source.rows.end());
      for (double dt : {packet::H, 0.0, -0.0, -packet::H}) {
        auto old = source, current = source;
        old.durations.drift_dt = current.durations.drift_dt = dt;
        std::vector<drift::Row> rows(count);
        recovery::FailureRow failure = 0;
        frozen::Drift(old.Input());
        Prepared(current, rows, failure);
        ASSERT_EQ(old.control.status, NodalStatus::Ok);
        EXPECT_EQ(failure, recovery::NoFailure);
        Same(old, current);
      }
    }
  }
}
TEST(CinDriftHost, EachFailingAxisIsStoredAndClearedButLaterAxesAndQuaternionAreUntouched) {
  for (unsigned axis = 0; axis < 3; ++axis) {
    auto source = DriftPacket(129);
    std::swap(source.rows.front().secondary, source.rows.back().secondary);
    FailPosition(source, 1, axis);
    FailPosition(source, 128, 0);
    const auto node = source.rows[1].secondary;
    source.accepted[9*packet::Nodes+4*node] = std::numeric_limits<double>::quiet_NaN();
    source.trial[6*packet::Nodes+3*node] = DBL_MAX;
    auto old = source, current = source;
    std::vector<drift::Row> rows(129);
    recovery::FailureRow failure = 71;
    frozen::Drift(old.Input());
    Prepared(current, rows, failure);
    ASSERT_EQ(old.control.status, NodalStatus::InvalidOutput);
    EXPECT_EQ(old.control.node, node);
    EXPECT_EQ(failure, 1u);
    EXPECT_EQ(rows[1].stored_axes, axis+1);
    Same(old, current);
    for (unsigned component = 0; component < 3; ++component) {
      const auto index = 3*node+component;
      if (component <= axis) {
        EXPECT_EQ(current.trial[13*packet::Nodes+index], 0);
        EXPECT_FALSE(std::signbit(current.trial[13*packet::Nodes+index]));
        EXPECT_FALSE(std::signbit(current.trial[16*packet::Nodes+index]));
      } else {
        EXPECT_EQ(current.trial[index], source.trial[index]);
        EXPECT_EQ(current.trial[13*packet::Nodes+index], source.trial[13*packet::Nodes+index]);
        EXPECT_EQ(current.trial[16*packet::Nodes+index], source.trial[16*packet::Nodes+index]);
      }
    }
    EXPECT_EQ(std::memcmp(current.trial.data()+9*packet::Nodes+4*node,
        source.trial.data()+9*packet::Nodes+4*node, 4*sizeof(double)), 0);
  }
}
TEST(CinDriftHost, OrientationFailureFollowsAllPositionStoresAndSourceRowWins) {
  for (bool angle : {false, true}) {
    auto source = DriftPacket(129);
    std::swap(source.rows[1].secondary, source.rows.back().secondary);
    const auto node = source.rows[1].secondary;
    source.accepted[9*packet::Nodes+4*node] = std::numeric_limits<double>::quiet_NaN();
    if (angle) source.trial[6*packet::Nodes+3*node] = DBL_MAX;
    FailPosition(source, 128, 0);
    // FailPosition seeds omega; restore this earlier angle failure if selected.
    if (angle) source.trial[6*packet::Nodes+3*node] = DBL_MAX;
    auto old = source, current = source;
    std::vector<drift::Row> rows(129);
    recovery::FailureRow failure = 0;
    frozen::Drift(old.Input());
    Prepared(current, rows, failure);
    EXPECT_EQ(old.control.status, angle ? NodalStatus::StepTooLarge : NodalStatus::InvalidOutput);
    EXPECT_EQ(failure, 1u);
    EXPECT_EQ(current.control.node, node);
    EXPECT_EQ(rows[1].stored_axes, 3u);
    Same(old, current);
    if (angle) EXPECT_EQ(current.control.limit.dt, 0);
  }
}
TEST(CinDriftHost, PureLeavesAndFailureRetryPreserveAllUnpublishedFields) {
  auto source = DriftPacket(129, true);
  auto input = source.Input();
  const auto before = source;
  std::vector<drift::Row> rows(129);
  for (unsigned row = 0; row < rows.size(); ++row) rows[row] = drift::Prepare(input, row);
  Same(before, source);
  const auto expected = rows.back();
  input.prepared_drift = rows.data();
  drift::Publish(input, 0, recovery::NoFailure);
  const auto after = drift::Prepare(input, 128);
  EXPECT_EQ(std::memcmp(&expected, &after, sizeof(after)), 0);
  recovery::FailureRow failure = 0;
  auto failed = before;
  FailPosition(failed, 128, 2);
  Prepared(failed, rows, failure);
  EXPECT_EQ(failure, 128u);
  auto retry = before, old = before;
  frozen::Drift(old.Input());
  Prepared(retry, rows, failure);
  EXPECT_EQ(failure, recovery::NoFailure);
  Same(old, retry);
  retry.control.status = NodalStatus::StaleTrial;
  const auto rejected = retry;
  const auto prior_rows = rows;
  failure = 37;
  Prepared(retry, rows, failure);
  Same(rejected, retry);
  EXPECT_EQ(failure, 37u);
  EXPECT_EQ(std::memcmp(rows.data(), prior_rows.data(), rows.size()*sizeof(drift::Row)), 0);
}
TEST(CinDriftHost, SharedOrientationValueRetainsLegacyReadsThresholdsAndFixedRotation) {
  for (double omega : {0.0, -0.0, DBL_TRUE_MIN, 1e-16, .125,
       std::nextafter(.125, 0.0), std::nextafter(.125, 1.0), DBL_MAX}) {
    for (bool fixed : {false, true}) for (bool bad_quaternion : {false, true}) {
      auto old = DriftPacket(1), current = old;
      const auto node = old.rows.front().secondary;
      std::fill(old.trial.begin()+6*packet::Nodes, old.trial.begin()+9*packet::Nodes, 0);
      old.trial[6*packet::Nodes+3*node] = omega;
      if (bad_quaternion) old.accepted[9*packet::Nodes+4*node] = 2;
      current = old;
      const auto expected = frozen_orientation::PrepareNodeOrientation(old.accepted.data(), old.trial.data(),
          node, packet::Nodes, 1, .125, fixed);
      const auto actual = nodal_detail::PrepareNodeOrientation(current.accepted.data(), current.trial.data(),
          node, packet::Nodes, 1, .125, fixed);
      EXPECT_EQ(actual, expected);
      Same(old, current);
      tl::math::Quaternion value{11, 12, 13, 14};
      const auto saved = value;
      const auto status = nodal_detail::PrepareNodeOrientationValue(current.accepted.data(), current.trial.data(),
          node, packet::Nodes, 1, .125, fixed, value);
      EXPECT_EQ(status, expected);
      if (status != NodalStatus::Ok) EXPECT_EQ(std::memcmp(&value, &saved, sizeof(value)), 0);
    }
  }
  // Angle rejection must precede even the first accepted-quaternion read.
  auto packet = DriftPacket(1);
  const auto node = packet.rows.front().secondary;
  packet.trial[6*cin_parallel_test::Nodes+3*node] = DBL_MAX;
  tl::math::Quaternion value;
  EXPECT_EQ(nodal_detail::PrepareNodeOrientationValue(nullptr, packet.trial.data(), node,
      cin_parallel_test::Nodes, 1, .125, false, value), NodalStatus::StepTooLarge);
}
TEST(CinDriftHost, TypedTailCountsAlignmentMetadataAndExactCapsWithoutNewAllocations) {
  nodal_detail::CinLayout layout;
  NodalCinLimits limits;
  ASSERT_TRUE(layout.Initialize(376930, 11165, 13173, limits, sizeof(nodal_detail::CinStorage), 779));
  EXPECT_EQ(sizeof(drift::Row), 64u);
  EXPECT_EQ(sizeof(nodal_detail::CinStorage), 664u);
  EXPECT_EQ(layout.host_bytes, 1375564u);
  EXPECT_EQ(layout.device_bytes, 37831712u);
  EXPECT_EQ(layout.optional_device_bytes, 62312528u);
  EXPECT_EQ(layout.prepared_drift.bytes, 714560u);
  EXPECT_EQ(layout.prepared_drift.offset, layout.recovery_failure.offset+8u);
  EXPECT_EQ(layout.device_bytes, layout.prepared_drift.offset+layout.prepared_drift.bytes);
  RecordProperty("host_delta", 32);
  RecordProperty("device_delta", 714564);
  const auto original = layout;
  limits.max_device_bytes = layout.optional_device_bytes;
  limits.max_host_bytes = layout.host_bytes;
  ASSERT_TRUE(layout.Initialize(376930, 11165, 13173, limits, sizeof(nodal_detail::CinStorage), 779));
  --limits.max_device_bytes;
  EXPECT_FALSE(layout.Initialize(376930, 11165, 13173, limits, sizeof(nodal_detail::CinStorage), 779));
  EXPECT_EQ(layout.device_bytes, original.device_bytes);
  ++limits.max_device_bytes;
  --limits.max_host_bytes;
  EXPECT_FALSE(layout.Initialize(376930, 11165, 13173, limits, sizeof(nodal_detail::CinStorage), 779));
  EXPECT_EQ(layout.host_bytes, original.host_bytes);
  ++limits.max_host_bytes;
  EXPECT_TRUE(layout.Initialize(376930, 11165, 13173, limits, sizeof(nodal_detail::CinStorage), 779));
}
} // namespace tl::fea::cin_drift_test
