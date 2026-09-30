// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "ResidentConfig.h"
#include "lib_src/elements/solids/resident/Arena.h"
namespace extended_model_test {
TEST(ExtendedSolidModel, LegacyResidentRejectsExtendedBeforeLayoutOrForecastPublication) {
  Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model extended,legacy;
  ASSERT_TRUE(extended.Initialize(domain,fixture.Input()));
  ASSERT_TRUE(legacy.Initialize(domain,fixture.solid_model_test::Fixture::Input()));
  auto config = ResidentConfig(domain.node_count());
  s::batch_detail::ArenaLayout layout; layout.bytes = 71;
  s::BatchForecast forecast; forecast.device_bytes = 73; forecast.startup_host_bytes = 79;
  EXPECT_EQ(s::batch_detail::Plan(config,extended,layout).status,s::BatchStatus::InvalidInput);
  EXPECT_EQ(layout.bytes,71u);
  EXPECT_EQ(s::Batch::Forecast(config,extended,forecast).status,s::BatchStatus::InvalidInput);
  EXPECT_EQ(forecast.device_bytes,73u); EXPECT_EQ(forecast.startup_host_bytes,79u);
  config.limits.max_host_bytes = 1;
  EXPECT_EQ(s::Batch::Forecast(config,extended,forecast).status,s::BatchStatus::InvalidInput);
  config = ResidentConfig(domain.node_count());
  ASSERT_TRUE(s::batch_detail::Plan(config,legacy,layout));
  ASSERT_TRUE(s::Batch::Forecast(config,legacy,forecast));
  EXPECT_GT(forecast.device_bytes,0u); EXPECT_GT(forecast.startup_host_bytes,legacy.owned_payload_bytes());
}
} // namespace extended_model_test
