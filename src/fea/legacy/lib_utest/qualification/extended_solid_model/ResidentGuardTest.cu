// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "ResidentConfig.h"
namespace extended_model_test {
TEST(ExtendedSolidModelCuda, LegacyInitializerAllocatesNothingOnExtendedAndRetriesOriginal) {
  Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model extended,legacy;
  ASSERT_TRUE(extended.Initialize(domain,fixture.Input()));
  ASSERT_TRUE(legacy.Initialize(domain,fixture.solid_model_test::Fixture::Input()));
  const auto config = ResidentConfig(domain.node_count());
  s::Batch batch;
  EXPECT_EQ(batch.InitializeJoined(config,extended).status,s::BatchStatus::InvalidInput);
  EXPECT_EQ(batch.allocations().device_bytes,0u);
  EXPECT_EQ(batch.allocations().device_allocations,0u);
  EXPECT_EQ(batch.startup_host_bytes(),0u);
  const auto status = batch.InitializeJoined(config,legacy);
  ASSERT_TRUE(status) << status.message;
  EXPECT_GT(batch.allocations().device_bytes,0u);
  EXPECT_EQ(batch.allocations().device_allocations,1u);
}
} // namespace extended_model_test
