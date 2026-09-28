// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../extended_solid_resident/OwnerFixture.h"
namespace controlled_resident_test {
using namespace extended_resident_test;
TEST(ControlledWorkerBudget, HighestRequestedPlanFitsDeviceAndCompleteHostOrFallsBack) {
  OwnerFixture fixture(false,true,{.001,1000,1},false,false,32);
  auto config=fixture.Configuration();config.limits.max_device_bytes=256u<<20;config.limits.max_host_bytes=384u<<20;
  s::BatchForecast forecast[4];unsigned requests[]{4,8,16,32};
  for(unsigned i=0;i<4;++i){config.limits.max_controlled_packet_blocks=requests[i];ASSERT_TRUE(s::Batch::Forecast(config,fixture.model,forecast[i]));
    EXPECT_EQ(forecast[i].controlled_packet_blocks,requests[i]);EXPECT_EQ(forecast[i].controlled_worker_slots,requests[i]);}
  config.limits.max_controlled_packet_blocks=32;config.limits.max_device_bytes=forecast[1].device_bytes;
  s::BatchForecast result;ASSERT_TRUE(s::Batch::Forecast(config,fixture.model,result));EXPECT_EQ(result.controlled_packet_blocks,8u);
  config.limits.max_device_bytes=256u<<20;config.limits.max_host_bytes=forecast[0].startup_host_bytes;
  ASSERT_TRUE(s::Batch::Forecast(config,fixture.model,result));EXPECT_EQ(result.controlled_packet_blocks,4u);
  --config.limits.max_host_bytes;result.device_bytes=37;
  EXPECT_FALSE(s::Batch::Forecast(config,fixture.model,result));EXPECT_EQ(result.device_bytes,37u);
  config.limits.max_host_bytes=384u<<20;config.limits.max_device_bytes=forecast[0].device_bytes-1;
  EXPECT_FALSE(s::Batch::Forecast(config,fixture.model,result));EXPECT_EQ(result.device_bytes,37u);
}
TEST(ControlledWorkerBudget, DefaultsLegacyIdentityInvalidCeilingsAndUsefulPacketClamp) {
  EXPECT_EQ(s::BatchLimits{}.max_controlled_packet_blocks,8u);
  OwnerFixture tiny(false,true,{1,1,1});auto config=tiny.Configuration();s::BatchForecast f;
  config.limits.max_controlled_packet_blocks=32;ASSERT_TRUE(s::Batch::Forecast(config,tiny.model,f));EXPECT_EQ(f.controlled_packet_blocks,16u);
  for(unsigned invalid:{0u,3u,7u,12u,33u}){config.limits.max_controlled_packet_blocks=invalid;EXPECT_EQ(s::Batch::Forecast(config,tiny.model,f).status,s::BatchStatus::InvalidInput);}
  auto a=tiny.Configuration(),b=a;b.limits.max_controlled_packet_blocks=4;EXPECT_FALSE(d::SameConfig(a,b));
  s::BatchDiagnostics x,y;x.controlled_packet_blocks=8;y=x;y.controlled_packet_blocks=4;EXPECT_FALSE(d::SameDiagnostics(x,y));
  y=x;y.controlled_worker_slots=1;EXPECT_FALSE(d::SameDiagnostics(x,y));
  OwnerFixture legacy;ASSERT_TRUE(s::Batch::Forecast(legacy.Configuration(),legacy.model,f));EXPECT_EQ(f.controlled_packet_blocks,0u);EXPECT_EQ(f.controlled_worker_slots,0u);
}
} // namespace controlled_resident_test
