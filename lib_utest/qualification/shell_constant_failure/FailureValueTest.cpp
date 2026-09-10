#include "lib_src/materials/failure/ShellConstantPlasticFailure.h"
#include <gtest/gtest.h>
#include <limits>

namespace {
namespace f = tl::material::failure;
void Same(const f::ConstantPlasticFailureResult& a, const f::ConstantPlasticFailureResult& b) {
  EXPECT_EQ(a.history.damage, b.history.damage);
  EXPECT_EQ(a.history.failure_time_s, b.history.failure_time_s);
  EXPECT_EQ(a.history.point_active, b.history.point_active);
  EXPECT_EQ(a.failed_now, b.failed_now);
}
TEST(ConstantPlasticFailure, AccumulationThresholdAndUnloadingPreserveFirstFailureTime) {
  const f::ConstantPlasticFailureParameters p{0.25};
  f::ConstantPlasticFailureResult result;
  ASSERT_TRUE(f::UpdateConstantPlasticFailure(p, {}, {0.0625, 0.01, true}, result));
  EXPECT_EQ(result.history.damage, 0.25);
  ASSERT_TRUE(f::UpdateConstantPlasticFailure(p, result.history, {0, 0.02, true}, result));
  EXPECT_EQ(result.history.damage, 0.25);
  EXPECT_TRUE(result.history.point_active);
  ASSERT_TRUE(f::UpdateConstantPlasticFailure(p, result.history, {0.1875, 0.03, true}, result));
  EXPECT_EQ(result.history.damage, 1);
  EXPECT_FALSE(result.history.point_active);
  EXPECT_EQ(result.history.failure_time_s, 0.03);
  EXPECT_TRUE(result.failed_now);
  ASSERT_TRUE(f::UpdateConstantPlasticFailure(p, result.history, {0.1, 0.04, true}, result));
  EXPECT_EQ(result.history.failure_time_s, 0.03);
  EXPECT_FALSE(result.failed_now);
}
TEST(ConstantPlasticFailure, InactiveElementDoesNotAccumulateAndPositiveOverflowSaturates) {
  f::ConstantPlasticFailureResult result;
  ASSERT_TRUE(f::UpdateConstantPlasticFailure({0.25}, {}, {0.3, 1, false}, result));
  EXPECT_EQ(result.history.damage, 0);
  EXPECT_TRUE(result.history.point_active);
  ASSERT_TRUE(f::UpdateConstantPlasticFailure({std::numeric_limits<double>::min()}, {},
      {std::numeric_limits<double>::max(), 1, true}, result));
  EXPECT_EQ(result.history.damage, 1);
  EXPECT_TRUE(result.failed_now);
}
TEST(ConstantPlasticFailure, InvalidLateInputPreservesOutputAndRetryUsesOriginalHistory) {
  const f::ConstantPlasticFailureResult held{{0.5, 0.125, false}, true};
  auto result = held;
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  for (double bad : {-1., nan, std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(f::UpdateConstantPlasticFailure({0.25}, {}, {bad, 0.2, true}, result));
    Same(result, held);
    EXPECT_FALSE(f::UpdateConstantPlasticFailure({0.25}, {}, {0.1, bad, true}, result));
    Same(result, held);
  }
  EXPECT_FALSE(f::UpdateConstantPlasticFailure({0}, {}, {0.1, 0.2, true}, result));
  Same(result, held);
  EXPECT_FALSE(f::UpdateConstantPlasticFailure({0.25}, held.history, {0.1, 0.1, true}, result));
  Same(result, held);
  ASSERT_TRUE(f::UpdateConstantPlasticFailure({0.25}, {}, {0.125, 0.2, true}, result));
  EXPECT_EQ(result.history.damage, 0.5);
  EXPECT_TRUE(result.history.point_active);
}
} // namespace
