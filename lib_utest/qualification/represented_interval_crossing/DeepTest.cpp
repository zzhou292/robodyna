// SPDX-License-Identifier: MIT
#include "Oracle.h"

namespace represented_interval_test {

TEST(RepresentedIntervalCrossing,
     DeepDyadicAffineContactUsesOwnedIterativeStack) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 52;
  limits.max_work_per_pair = 64;
  auto owner = Owner(limits);
  const auto a = Static(10, BaseTriangle());
  const double slope = std::ldexp(1.0, 40);
  const auto b = Path(20, BaseTriangle(1),
                      BaseTriangle(1 - slope));
  const auto result = One(owner, {a, b});
  ASSERT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.witness_time_numerator, 1u);
  EXPECT_EQ(result.witness_time_depth, 40u);
  EXPECT_LE(result.work, 41u);
  const auto oracle = ExactOracleAt(
      a, b, result.witness_time_numerator,
      std::uint64_t{1} << result.witness_time_depth);
  EXPECT_TRUE(oracle.valid);
  EXPECT_TRUE(oracle.intersects);
}

TEST(RepresentedIntervalCrossing,
     MaxDepth52ExhaustionIsBoundedAndNeverFalseSeparated) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 52;
  limits.max_work_per_pair = 53;
  limits.max_total_work = 53;
  auto owner = Owner(limits);
  EXPECT_EQ(owner.forecast().dfs_frame_capacity, 53u);
  EXPECT_GT(owner.forecast().dfs_frame_bytes, 0u);
  EXPECT_GT(owner.forecast().exact_scratch_bytes, 0u);
  const std::array<ct::Vec3, 3> diagonal{
      ct::Vec3{1.5, 1.5, 0}, {3.5, 1.5, 0}, {1.5, 3.5, 0}};
  const auto a = Static(10, BaseTriangle());
  const auto b = Static(20, diagonal);
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::Unresolved);
  EXPECT_EQ(result.reason, R::WorkExhausted);
  EXPECT_EQ(result.work, 53u);
  for (std::uint64_t numerator = 0; numerator <= 8; ++numerator)
    EXPECT_FALSE(ExactOracleAt(a, b, numerator, 8).intersects);
}

TEST(RepresentedIntervalCrossing,
     NondyadicIsolatedContactRemainsUnresolvedNeverSeparated) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 12;
  limits.max_work_per_pair = 256;
  auto owner = Owner(limits);
  const auto a = Static(10, BaseTriangle());
  const auto b = Path(20, BaseTriangle(1), BaseTriangle(-2));
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::Unresolved);
  EXPECT_EQ(result.reason, R::WorkExhausted);
  const auto exact_third = ExactOracleAt(a, b, 1, 3);
  ASSERT_TRUE(exact_third.valid);
  EXPECT_TRUE(exact_third.intersects);
  for (const auto time : {std::pair<std::uint64_t, std::uint64_t>{0, 1},
                          {1, 4}, {1, 2}, {1, 1}})
    EXPECT_FALSE(ExactOracleAt(a, b, time.first, time.second).intersects);
}

TEST(RepresentedIntervalCrossing,
     ExtremeBinary64ExponentsInterpolateWithoutFalseRangeResult) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 52;
  auto owner = Owner(limits);
  const double huge = std::ldexp(1.0, 900);
  const double tiny = std::numeric_limits<double>::denorm_min();
  const std::array<ct::Vec3, 3> base{
      ct::Vec3{0, 0, 0}, {huge, 0, 0}, {0, huge, 0}};
  const std::array<ct::Vec3, 3> first{
      ct::Vec3{huge, 0, tiny}, {2 * huge, 0, tiny},
      {huge, huge, tiny}};
  const std::array<ct::Vec3, 3> second{
      ct::Vec3{-huge, 0, tiny}, {0, 0, tiny},
      {-huge, huge, tiny}};
  const auto a = Static(10, base);
  const auto b = Path(20, first, second);
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.reason, R::None);
  for (std::uint64_t numerator = 0; numerator <= 4; ++numerator) {
    const auto oracle = ExactOracleAt(a, b, numerator, 4);
    EXPECT_TRUE(oracle.valid);
    EXPECT_FALSE(oracle.intersects);
  }
}

}  // namespace represented_interval_test
