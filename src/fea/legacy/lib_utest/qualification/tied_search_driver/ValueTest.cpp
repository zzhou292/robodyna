#include "Fixture.h"
#include <limits>
namespace tied_driver_test {
TEST(TiedSearchDriverValues, SourceOrderRestoredWithoutDroppingUnmatchedOrOwnNode) {
  Fixture f;
  const auto in = f.Input();
  ASSERT_TRUE(detail::CheckCounts(in, {}));
  ASSERT_TRUE(detail::CheckValues(in));
  std::vector<ts::NativeSearchBounds> bounds;
  std::vector<double> radii;
  ASSERT_TRUE(detail::PrepareBounds(in, bounds, radii));
  std::vector<CollisionPair> pairs{{5,1},{2,1},{0,4},{0,2},{1,4},{0,5}};
  ts::SearchDriverResult result;
  ASSERT_TRUE(detail::Reduce(in, pairs, bounds, result));
  ASSERT_EQ(result.rows.size(), 4u);
  EXPECT_EQ(result.rows[0].choice.ordered_master, 1u);
  EXPECT_EQ(result.rows[0].admissible_candidates, 2u);
  EXPECT_FALSE(result.rows[1].choice.matched);
  EXPECT_EQ(result.rows[2].excluded_own_node, 2u);
  EXPECT_FALSE(result.rows[2].choice.matched);
  EXPECT_TRUE(result.rows[3].choice.matched); // Distinct physical NID at same coordinates.
  EXPECT_EQ(result.matched_count, 2u);
  EXPECT_EQ(result.pair_count, 6u);
  EXPECT_EQ(result.rows[0].force_patch_status, ts::Status::Success);
}
TEST(TiedSearchDriverValues, InclusiveNativeBoxFilterAndRepeatedTriangleSlots) {
  Fixture f;
  f.masters.resize(1);
  f.masters[0].nodes = {0,1,2,2};
  f.masters[0].topology = ts::MasterTopology::TriangleRepeatedThird;
  auto in = f.Input();
  ASSERT_TRUE(detail::CheckValues(in));
  std::vector<ts::NativeSearchBounds> bounds;
  std::vector<double> radii;
  ASSERT_TRUE(detail::PrepareBounds(in, bounds, radii));
  f.positions[4] = {0,0,bounds[0].maximum.z};
  std::vector<CollisionPair> pairs{{0,1}};
  ts::SearchDriverResult result;
  ASSERT_TRUE(detail::Reduce(in, pairs, bounds, result));
  EXPECT_EQ(result.rows[0].within_native_bounds, 1u);
  f.positions[4].z = std::nextafter(f.positions[4].z, std::numeric_limits<double>::infinity());
  pairs = {{0,1}};
  result = {};
  ASSERT_TRUE(detail::Reduce(in, pairs, bounds, result));
  EXPECT_EQ(result.rows[0].within_native_bounds, 0u);
  f.masters[0].nodes[3] = 3;
  EXPECT_FALSE(detail::CheckValues(in));
}
TEST(TiedSearchDriverValues, ByteCapsPrecedeBorrowedReadAndForecastRetryIsExact) {
  Fixture f;
  auto in = f.Input();
  ts::SearchDriverLimits limits;
  limits.max_pairs = 7;
  ts::SearchDriverBudget expected;
  ASSERT_TRUE(detail::Budget(in, limits, {1234,567}, expected));
  EXPECT_EQ(expected.pair_capacity, 7u);
  limits.max_host_bytes = expected.startup_host_bytes;
  limits.max_device_bytes = expected.device_bytes;
  ts::SearchDriverBudget exact;
  ASSERT_TRUE(detail::Budget(in, limits, {1234,567}, exact));
  EXPECT_EQ(exact.startup_host_bytes, expected.startup_host_bytes);
  EXPECT_EQ(exact.device_bytes, expected.device_bytes);
  limits.max_pairs = 1;
  ASSERT_TRUE(detail::Budget(in, limits, {1234,567}, exact));
  limits.max_host_bytes = exact.startup_host_bytes - 1;
  in.working_positions = reinterpret_cast<const ts::Vec3*>(alignof(ts::Vec3));
  in.masters = reinterpret_cast<const ts::SearchMasterInput*>(alignof(ts::SearchMasterInput));
  const auto saved = exact;
  EXPECT_EQ(detail::Budget(in, limits, {1234,567}, exact).status, ts::SearchDriverStatus::ResourceLimit);
  EXPECT_EQ(exact.startup_host_bytes, saved.startup_host_bytes);
  limits.max_host_bytes++;
  ASSERT_TRUE(detail::Budget(in, limits, {1234,567}, exact));
  EXPECT_EQ(exact.pair_capacity, 1u);
}
TEST(TiedSearchDriverValues, LateMalformedNodesAndDuplicatePairsFailThenCleanRetry) {
  Fixture f;
  f.masters.back().nodes[3] = 999;
  auto report = detail::CheckValues(f.Input());
  EXPECT_FALSE(report);
  EXPECT_EQ(report.master, 1u);
  f.masters.back().nodes[3] = 3;
  ASSERT_TRUE(detail::CheckValues(f.Input()));
  std::vector<ts::NativeSearchBounds> bounds;
  std::vector<double> radii;
  ASSERT_TRUE(detail::PrepareBounds(f.Input(), bounds, radii));
  std::vector<CollisionPair> pairs{{0,2},{1,2},{2,0}};
  ts::SearchDriverResult staged;
  report = detail::Reduce(f.Input(), pairs, bounds, staged);
  EXPECT_FALSE(report);
  EXPECT_EQ(report.secondary, 0u);
  pairs = {{1,2},{0,2}};
  staged = {};
  ASSERT_TRUE(detail::Reduce(f.Input(), pairs, bounds, staged));
  EXPECT_EQ(staged.rows[0].choice.ordered_master, 1u);
}
TEST(TiedSearchDriverValues, SelectedSingularPatchRemainsSelectedAndExplicitlyBlocked) {
  Fixture f;
  f.masters.resize(1);
  for (std::size_t n = 0; n < 4; ++n) f.positions[n] = {double(n),0,0};
  f.positions[4] = {1.5,0,0};
  std::vector<ts::NativeSearchBounds> bounds;
  std::vector<double> radii;
  ASSERT_TRUE(detail::PrepareBounds(f.Input(), bounds, radii));
  std::vector<CollisionPair> pairs{{0,1}};
  ts::SearchDriverResult result;
  ASSERT_TRUE(detail::Reduce(f.Input(), pairs, bounds, result));
  ASSERT_TRUE(result.rows[0].choice.matched);
  EXPECT_EQ(result.rows[0].choice.ordered_master, 1u);
  EXPECT_EQ(result.rows[0].force_patch_status, ts::Status::SingularPatch);
  EXPECT_FALSE(result.rows[0].force_patch.prepared());
  EXPECT_EQ(result.singular_patch_count, 1u);
}
TEST(TiedSearchDriverValues, OriginalCountForecastIsBoundedWithoutAllocatingOriginalArrays) {
  ts::SearchDriverInput in;
  in.node_count = 194622;
  in.master_count = 171813;
  in.secondary_count = 11165;
  in.working_length_to_m = .001;
  in.working_positions = reinterpret_cast<const ts::Vec3*>(alignof(ts::Vec3));
  in.masters = reinterpret_cast<const ts::SearchMasterInput*>(alignof(ts::SearchMasterInput));
  in.secondary_nodes = reinterpret_cast<const std::uint32_t*>(alignof(std::uint32_t));
  ts::SearchDriverBudget result;
  ASSERT_TRUE(detail::Budget(in, {}, {0,0}, result)); // Scratch is qualified on the actual GPU.
  EXPECT_LE(result.startup_host_bytes, ts::SearchDriverLimits{}.max_host_bytes);
  EXPECT_LE(result.device_bytes, ts::SearchDriverLimits{}.max_device_bytes);
  EXPECT_EQ(result.pair_capacity, ts::SearchDriverLimits{}.max_pairs);
}
}
