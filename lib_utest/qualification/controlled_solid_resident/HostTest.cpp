// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../extended_solid_resident/OwnerFixture.h"
namespace controlled_resident_test {
using namespace extended_resident_test;
TEST(ControlledResidentHost, CompleteRosterProfileAndExactBudgets) {
  OwnerFixture fixture(false,true,{.001,1000,1});auto c=fixture.Configuration();s::BatchForecast f;
  ASSERT_TRUE(s::Batch::Forecast(c,fixture.model,f));
  d::ArenaLayout layout;ASSERT_TRUE(d::Plan(c,fixture.model,layout));
  EXPECT_EQ(layout.controlled.reference24.count,1u);EXPECT_EQ(layout.controlled.reference90.count,1u);
  EXPECT_EQ(layout.controlled.packets.count,5u);EXPECT_EQ(layout.controlled.members.count,5u);
  EXPECT_EQ(layout.controlled.workspace.count,2u);
  c.limits.max_device_bytes=f.device_bytes-1;EXPECT_FALSE(s::Batch::Forecast(c,fixture.model,f));
  c=fixture.Configuration();ASSERT_TRUE(s::Batch::Forecast(c,fixture.model,f));
  c.limits.max_device_bytes=f.device_bytes;c.limits.max_host_bytes=f.startup_host_bytes;
  EXPECT_TRUE(s::Batch::Forecast(c,fixture.model,f));
  c.limits.max_host_bytes=f.startup_host_bytes-1;EXPECT_FALSE(s::Batch::Forecast(c,fixture.model,f));
  c=fixture.Configuration();c.profile=s::BatchProfile::PhysicalCinExtendedLaw44Law90V2;
  EXPECT_FALSE(s::Batch::Forecast(c,fixture.model,f));
}
TEST(ControlledResidentHost, LegacyHasNoOptionalPerParentControlRegions) {
  OwnerFixture fixture;d::ArenaLayout l;ASSERT_TRUE(d::Plan(fixture.Configuration(),fixture.model,l));
  EXPECT_EQ(l.controlled.index24.count,0u);EXPECT_EQ(l.controlled.index90.count,0u);
  EXPECT_EQ(l.controlled.reference24.count,0u);EXPECT_EQ(l.controlled.reference90.count,0u);
  EXPECT_EQ(l.controlled.workspace.count,0u);EXPECT_EQ(l.controlled.packets.count,0u);
  auto c=fixture.Configuration();c.profile=s::BatchProfile::PhysicalCinSourceControlsV3;s::BatchForecast f;
  EXPECT_FALSE(s::Batch::Forecast(c,fixture.model,f));
}
TEST(ControlledResidentHost, CollapsedReferenceRetainsEightMassOccurrences) {
  OwnerFixture fixture(false,true,{.001,1000,1},true);
  ASSERT_TRUE(fixture.model.prepared());const auto& p=fixture.model.solid24()[0];
  EXPECT_EQ(p.domain_nodes[4],p.domain_nodes[5]);EXPECT_EQ(p.domain_nodes[6],p.domain_nodes[7]);
  d::ArenaLayout l;EXPECT_TRUE(d::Plan(fixture.Configuration(),fixture.model,l));
}
} // namespace controlled_resident_test
