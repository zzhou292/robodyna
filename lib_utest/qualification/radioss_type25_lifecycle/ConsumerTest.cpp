// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Lifecycle.h"
#include "Fixture.h"
#include <gtest/gtest.h>
namespace type25_lifecycle_test {
TEST(Type25LifecycleConsumer, SharedOperationsNeedNoOracleOrPhysicalOwner) {
  Fixture f;auto input=f.Input();ASSERT_EQ(l::detail::Validate(input),n::selection::Status::Ok);
  l::HostResult out;
  ASSERT_EQ(l::EvaluateNativeLifecycleHost(input,Fixture::Limits(),&out).status,n::selection::Status::Ok);
  ASSERT_EQ(out.rows.size(),1u);EXPECT_GT(out.rows[0].new_impact_count,0u);
  EXPECT_EQ(f.accepted[0].row.irtlm[0],0);
}
TEST(Type25LifecycleConsumer, UnknownRuntimePrecisionNeverSelectsDefault) {
  Fixture f;f.profile.optcd_response_precision=-1;
  EXPECT_EQ(l::detail::Validate(f.Input()),n::selection::Status::UnsupportedProfile);
}
}
