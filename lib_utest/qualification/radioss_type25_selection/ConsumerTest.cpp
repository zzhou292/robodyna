// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include <gtest/gtest.h>
namespace type25_selection_test {
TEST(Type25SelectionConsumer, NativeRetainedHeaderNeedsNoOracleLink) {
  const auto c=Basic();s::NativeRetainedResult result;
  EXPECT_EQ(s::EvaluateNativeRetained({},c.input,c.prior,&result),s::Status::UnsupportedProfile);
  ASSERT_EQ(s::EvaluateNativeRetained(Profile(),c.input,c.prior,&result),s::Status::Ok);
  EXPECT_TRUE(result.active);EXPECT_DOUBLE_EQ(result.classification_product,800.);
  EXPECT_TRUE(result.cache.sector[0].defined&s::ClampedBarycentricDefined);
}
} // namespace type25_selection_test
