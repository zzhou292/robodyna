// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ContinuationCases.h"
#include "NewImpactCases.h"
#include <gtest/gtest.h>
namespace type25_selection_test {
TEST(Type25SelectionConsumer, NativeRetainedHeaderNeedsNoOracleLink) {
  const auto c=Basic();s::NativeRetainedResult result;
  EXPECT_EQ(s::EvaluateNativeRetained({},c.input,c.prior,&result),s::Status::UnsupportedProfile);
  ASSERT_EQ(s::EvaluateNativeRetained(Profile(),c.input,c.prior,&result),s::Status::Ok);
  EXPECT_TRUE(result.active);EXPECT_DOUBLE_EQ(result.classification_product,800.);
  EXPECT_TRUE(result.cache.sector[0].defined&s::ClampedBarycentricDefined);
}
TEST(Type25SelectionConsumer, NativeContinuationHeaderNeedsNoOracleLink) {
  const auto c=BasicContinuation();s::NativeContinuationResult result;
  EXPECT_EQ(s::EvaluateNativeContinuation({},c.input,c.prior,&result),s::Status::UnsupportedProfile);
  ASSERT_EQ(s::EvaluateNativeContinuation(Profile(),c.input,c.prior,&result),s::Status::Ok);
  EXPECT_TRUE(result.active);EXPECT_TRUE(result.row_replaced);
  EXPECT_DOUBLE_EQ(result.classification_product,800.);
}
TEST(Type25SelectionConsumer, NativeNewImpactHeaderNeedsNoOracleLink) {
  const auto c=BasicNewImpact();s::NativeNewImpactResult result;
  ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&result),s::Status::Ok);
  EXPECT_TRUE(result.active);EXPECT_TRUE(result.row_replaced);
  EXPECT_EQ(result.selected_side,s::ImpactSide::Primary);
}
} // namespace type25_selection_test
