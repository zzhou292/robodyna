// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include <gtest/gtest.h>
namespace type25_geometry_test {
TEST(Type25GeometryConsumer, PublicHeadersAndTypedNativeBridgeNeedNoOracleLink) {
  const auto input=Quad();n::NativeRawGeometryResult result;
  EXPECT_EQ(n::EvaluateNativeRawGeometry({},input,&result),n::GeometryStatus::UnsupportedProfile);
  ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),input,&result),n::GeometryStatus::Ok);
  EXPECT_DOUBLE_EQ(result.geometric_penetration,.4);
  EXPECT_EQ(result.key.secondary_source_id,input.key.secondary_source_id);
  const auto forecast=n::PreflightNativeGeometryHistory(1,1);
  EXPECT_EQ(forecast.status,n::GeometryStatus::Ok);
  EXPECT_EQ(forecast.total_scratch_bytes,sizeof(n::NativeGeometryHistory)+sizeof(n::NativeGeometryFinalResult));
}
} // namespace type25_geometry_test
