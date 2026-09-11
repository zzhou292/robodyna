// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace extended_resident_test {
TEST(ExtendedResidentBudget, ExactCountCapsTypedScratchAndBothProfileGuards) {
  Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain,fixture.Input()));
  auto config = Config(domain.node_count());
  d::ArenaLayout layout;
  ASSERT_TRUE(d::Plan(config,model,layout));
  EXPECT_EQ(layout.scratch44.count,1u);
  EXPECT_EQ(layout.scratch90.bytes,sizeof(d::ExtendedScratch<d::Traits18Law90>));
  EXPECT_EQ(layout.solid18_law44.staging.bytes,sizeof(d::State<d::Traits18Law44>));
  EXPECT_EQ(layout.solid18_law90.slab[1].bytes,sizeof(d::State<d::Traits18Law90>));
  s::BatchForecast forecast;
  ASSERT_TRUE(s::Batch::Forecast(config,model,forecast));
  config.limits.max_device_bytes=forecast.device_bytes;
  config.limits.max_host_bytes=forecast.startup_host_bytes;
  s::BatchForecast exact;
  ASSERT_TRUE(s::Batch::Forecast(config,model,exact));
  --config.limits.max_host_bytes;
  exact.startup_host_bytes=73;
  EXPECT_FALSE(s::Batch::Forecast(config,model,exact));
  EXPECT_EQ(exact.startup_host_bytes,73u);
  config=Config(domain.node_count());
  config.limits.max_device_bytes=forecast.device_bytes-1;
  EXPECT_FALSE(s::Batch::Forecast(config,model,exact));
  config=Config(domain.node_count());
  config.profile=s::BatchProfile::PhysicalCinV1;
  EXPECT_EQ(d::Plan(config,model,layout).status,s::BatchStatus::InvalidInput);
  solid_model_test::Fixture old;
  const auto old_domain=old.Domain();
  s::Model legacy;
  ASSERT_TRUE(legacy.Initialize(old_domain,old.Input()));
  config=Config(old_domain.node_count());
  EXPECT_EQ(d::Plan(config,legacy,layout).status,s::BatchStatus::InvalidInput);
  config.profile=s::BatchProfile::PhysicalCinV1;
  ASSERT_TRUE(d::Plan(config,legacy,layout));
  d::Counts count{0,0,0,0,0,6,1,SIZE_MAX,1,1};
  layout.bytes=99;
  EXPECT_FALSE(d::MakeLayout(count,config,layout));
  EXPECT_EQ(layout.bytes,99u);
}
TEST(ExtendedResidentBudget, QualifiedOriginalFamilyCountsFitUnchangedArenaCaps) {
  auto config=Config(372435);
  d::Counts counts{908,1309,195,1,8,1024,306,1345,2,1};
  d::ArenaLayout layout;
  ASSERT_TRUE(d::MakeLayout(counts,config,layout));
  EXPECT_EQ(layout.scratch44.count,306u);EXPECT_EQ(layout.scratch90.count,1345u);
  EXPECT_EQ(layout.solid18_law90.staging.count,1345u);
  EXPECT_LT(layout.bytes,s::BatchLimits{}.max_device_bytes);
  RecordProperty("count_only_device_forecast",layout.bytes);
  RecordProperty("count_only_staging_bytes",layout.staging_bytes);
}
} // namespace extended_resident_test
