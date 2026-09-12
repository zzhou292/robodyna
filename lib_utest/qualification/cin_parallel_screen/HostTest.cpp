// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/solvers/NodalCinStorage.h"
namespace tl::fea::cin_screen_test {
TEST(CinParallelScreenHost, DomainIndexTiesAndUnboundedSentinelAcrossTreeGrains) {
  for (unsigned count : {8u, 127u, 128u, 129u, 263u, 33001u}) {
    SCOPED_TRACE(count);
    Fixture f(count);
    const auto accepted = f.accepted;
    Compare(f.View(), .8, true);
    std::fill(f.translation.begin(), f.translation.end(), 0);
    std::fill(f.rotation.begin(), f.rotation.end(), 0);
    Compare(f.View(), .8, true);
    dt::Result result;
    std::uint32_t bad = 0;
    ASSERT_TRUE(Staged(f.View(), .8, result, bad));
    Exact(result.minimum_dt, std::numeric_limits<double>::max());
    EXPECT_EQ(result.limiting_node, UINT32_MAX);
    // Equal translation/rotation limits on multiple ordinary rows retain row0.
    f.translation[0] = 8;
    f.rotation[0] = 12;
    f.translation[count-1] = 8;
    f.mass[count-1] = 2;
    f.fixed[count-1] = 0;
    f.fixed_rotation[count-1] = 1;
    Compare(f.View(), .8, true);
    ASSERT_TRUE(Staged(f.View(), .8, result, bad));
    EXPECT_EQ(result.limiting_node, 0u);
    EXPECT_EQ(result.limiting_group, UINT32_MAX);
    EXPECT_EQ(f.accepted, accepted);
  }
  auto unbounded = tree::Empty();
  tree::Merge(unbounded, tree::Empty());
  EXPECT_EQ(unbounded.limiting_node, UINT32_MAX);
  // A fabricated equal-DBL_MAX bounded node is never produced by Include.
  dt::Result initial;
  dt::Include({std::numeric_limits<double>::max(), true}, 3, UINT32_MAX, initial);
  EXPECT_EQ(initial.limiting_node, UINT32_MAX);
  // Identical ordered group traces keep the first group. An equal ordinary
  // summary keeps its earlier domain winner when the rigid suffix is appended.
  Fixture tied;
  std::fill(tied.translation.begin(), tied.translation.end(), 0);
  std::fill(tied.rotation.begin(), tied.rotation.end(), 0);
  tied.translation[2] = 11;
  tied.translation[3] = 13;
  tied.groups[1] = tied.groups[0];
  std::copy_n(tied.accepted.data()+19*tied.nodes, rigid::GroupStateValues,
      tied.accepted.data()+19*tied.nodes+rigid::GroupStateValues);
  Compare(tied.View(), .8, true);
  dt::Result group_result;
  std::uint32_t invalid = UINT32_MAX;
  ASSERT_TRUE(Frozen(tied.View(), .8, group_result, invalid));
  ASSERT_EQ(group_result.limiting_group, 0u);
  const tree::Summary equal_ordinary{group_result.minimum_dt, 0, UINT32_MAX};
  ASSERT_TRUE(tree::Complete(tied.View(), .8, equal_ordinary, group_result, invalid));
  EXPECT_EQ(group_result.limiting_node, 0u);
  EXPECT_EQ(group_result.limiting_group, UINT32_MAX);
}
TEST(CinParallelScreenHost, FourCoefficientChecksPrecedeAllRolesAndPreserveMaskBranches) {
  Fixture f;
  f.mass[1] = NAN; // CIN secondary still consumes every coefficient.
  f.rotation[6] = 1;
  Compare(f.View(), .8, false, 1);
  f.mass[1] = 0;
  Compare(f.View(), .8, false, 6);
  f.rotation[6] = 0;
  f.mass[2] = -1; // Rigid member still consumes every coefficient.
  Compare(f.View(), .8, false, 2);
  f.mass[2] = 0;
  f.mass[130] = 0;
  f.fixed[130] = 7;
  f.inertia[130] = 0;
  f.fixed_rotation[130] = 1;
  Compare(f.View(), .8, true);
  f.fixed[130] = 5;
  Compare(f.View(), .8, false, 130);
  f.mass[130] = 2;
  f.fixed_rotation[130] = 0;
  Compare(f.View(), .8, false, 130);
  f.present[130] = 0;
  f.rotation[130] = 0;
  Compare(f.View(), .8, true);
}
TEST(CinParallelScreenHost, HeaderFailureLeavesErrorAndOutputSeedUnchanged) {
  Fixture f;
  for (double factor : {0., -0., -1., 1.01, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) Compare(f.View(), factor, false, 87);
  for (unsigned fault = 0; fault < 8; ++fault) {
    auto view = f.View();
    if (fault == 0) view.accepted = nullptr;
    if (fault == 1) view.mass = nullptr;
    if (fault == 2) view.cin_secondary = nullptr;
    if (fault == 3) view.rigid.members = nullptr;
    if (fault == 4) view.previous_drift_dt = -1;
    if (fault == 5) view.previous_drift_dt = NAN;
    if (fault == 6) view.nodes = 0;
    if (fault == 7) view.fixed_rotation = nullptr;
    Compare(view, .8, false, 87);
  }
  f.previous = -0.;
  Compare(f.View(), 1, true);
}
TEST(CinParallelScreenHost, MalformedGroupSuffixRetainsLastVisitedOrdinaryOrMemberIndex) {
  Fixture f;
  f.groups[0].count = 1;
  Compare(f.View(), .8, false, f.nodes-1);
  f.inertia[128] = -1;
  Compare(f.View(), .8, false, 128);
  f.inertia[128] = 3;
  f.groups[0].count = 2;
  f.groups[1].offset = 999;
  Compare(f.View(), .8, false, 3);
  f.groups[1].offset = 2;
  f.members[3].node = f.nodes;
  Compare(f.View(), .8, false, f.nodes);
  f.members[3].node = 5;
  f.groups[1].principal_inertia.z = 0;
  Compare(f.View(), .8, false, 4);
  f.groups[1].principal_inertia.z = 1;
  Compare(f.View(), .8, true);
}
TEST(CinParallelScreenHost, ExactScalarExtremesAndConsumedOnlyOrdinaryReadSet) {
  Fixture f;
  f.accepted[0] = NAN; // Ordinary x/v/omega/quaternion are not screen inputs.
  f.accepted[3*f.nodes] = INFINITY;
  Compare(f.View(), .8, true);
  f.mass[128] = std::numeric_limits<double>::max();
  Compare(f.View(), .8, false, 128);
  f.mass[128] = 2;
  f.translation[128] = std::numeric_limits<double>::denorm_min();
  Compare(f.View(), .8, false, 128);
  f.translation[128] = 4;
  f.translation[0] = 64;
  Compare(f.View(), std::numeric_limits<double>::denorm_min(), false, 0);
  f.translation[0] = 4;
  Compare(f.View(), .8, true);
}
TEST(CinParallelScreenHost, DedicatedCountedTailAndLateBudgetRetryPreserveKeys) {
  NodalCinLimits limits;
  nodal_detail::CinLayout layout;
  ASSERT_TRUE(layout.Initialize(372435, 11165, 13173, limits, sizeof(nodal_detail::CinStorage)));
  EXPECT_EQ(layout.screen.bytes, 4096u);
  EXPECT_EQ(layout.screen.count, 256u);
  EXPECT_EQ(layout.screen.offset, layout.input_failure.offset+layout.input_failure.bytes);
  EXPECT_EQ(layout.group_reports.count, 0u);
  EXPECT_EQ(layout.prepared_transfers.offset, layout.screen.offset+layout.screen.bytes);
  EXPECT_EQ(layout.prepared_transfers.count, 11165u);
  EXPECT_EQ(layout.prepared_transfers.bytes, 5716480u);
  EXPECT_EQ(layout.device_bytes, layout.prepared_transfers.offset+layout.prepared_transfers.bytes);
  EXPECT_EQ(layout.failure.bytes, 8u);
  EXPECT_EQ(layout.input_failure.bytes, 8u);
  auto exact = limits;
  exact.max_device_bytes = layout.optional_device_bytes;
  exact.max_host_bytes = layout.host_bytes;
  nodal_detail::CinLayout retry;
  ASSERT_TRUE(retry.Initialize(372435, 11165, 13173, exact, sizeof(nodal_detail::CinStorage)));
  const auto good = retry;
  --exact.max_device_bytes;
  EXPECT_FALSE(retry.Initialize(372435, 11165, 13173, exact, sizeof(nodal_detail::CinStorage)));
  EXPECT_EQ(retry.device_bytes, good.device_bytes);
  ++exact.max_device_bytes;
  ASSERT_TRUE(retry.Initialize(372435, 11165, 13173, exact, sizeof(nodal_detail::CinStorage)));
  --exact.max_host_bytes;
  EXPECT_FALSE(retry.Initialize(372435, 11165, 13173, exact, sizeof(nodal_detail::CinStorage)));
  EXPECT_EQ(retry.host_bytes, good.host_bytes);
  ++exact.max_host_bytes;
  ASSERT_TRUE(retry.Initialize(372435, 11165, 13173, exact, sizeof(nodal_detail::CinStorage)));
  nodal_detail::CinLayout old_metadata;
  const auto added_metadata = sizeof(util::ArenaRegion)+sizeof(tree::Summary*);
  ASSERT_TRUE(old_metadata.Initialize(372435, 11165, 13173, limits,
      sizeof(nodal_detail::CinStorage)-added_metadata));
  EXPECT_EQ(layout.host_bytes-old_metadata.host_bytes, added_metadata);
  RecordProperty("screen_host_metadata_bytes", std::to_string(added_metadata));
  RecordProperty("original_screen_bytes", std::to_string(layout.screen.bytes));
  RecordProperty("complete_optional_device_bytes", std::to_string(layout.optional_device_bytes));
}
} // namespace tl::fea::cin_screen_test
