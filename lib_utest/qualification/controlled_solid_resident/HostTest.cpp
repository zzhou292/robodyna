// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../extended_solid_resident/OwnerFixture.h"
#include "../solid_control_schedule/Fixture.h"
namespace controlled_resident_test {
using namespace extended_resident_test;
TEST(ControlledResidentHost, CompleteRosterProfileAndExactBudgets) {
  OwnerFixture fixture(false,true,{.001,1000,1});auto c=fixture.Configuration();s::BatchForecast f;
  ASSERT_TRUE(s::Batch::Forecast(c,fixture.model,f));
  d::ArenaLayout layout;ASSERT_TRUE(d::Plan(c,fixture.model,layout));
  EXPECT_EQ(layout.controlled.reference24.count,1u);EXPECT_EQ(layout.controlled.reference90.count,1u);
  EXPECT_EQ(layout.controlled.packets.count,5u);EXPECT_EQ(layout.controlled.members.count,5u);
  EXPECT_EQ(layout.controlled.workspace.count,1u);
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
TEST(ControlledResidentHost, UnsupportedUnitsAndS6ControlFailAdmission) {
  s::BatchForecast f;control_schedule_test::Fixture source;s::Model invalid;
  auto unsupported=source.ControlledInput();unsupported.controls.units={.001,.001,.001};
  EXPECT_FALSE(invalid.Initialize(source.Domain(),unsupported));
  s::Model model;
  ASSERT_TRUE(model.Initialize(source.Domain(),source.ControlledInput()));
  auto c=Config(model.domain()->node_count());c.profile=s::BatchProfile::PhysicalCinSourceControlsV3;
  const auto status=s::Batch::Forecast(c,model,f);
  EXPECT_FALSE(status);EXPECT_EQ(status.family,s::Family::Solid6z);
}
TEST(ControlledResidentHost, NativePacketIsNeverRechunkedToFitCta) {
  control_schedule_test::Fixture source;
  for(auto& row:source.source)if(row.icontrol&&row.element_id==103)row.icontrol=0;
  source.packets[4].icontrol=0;
  const auto original=source.input24[0];
  for(unsigned i=0;i<127;++i){auto p=original;auto ref=p.reference.input();ref.source_element_id=10000+i;
    ASSERT_EQ(fe::solid24::InitializeReference(ref,p.reference),fe::solid24::Status::Success);
    source.input24.push_back(p);source.source.push_back({ref.source_element_id,ref.source_part_id,ref.source_section_id,ref.source_material_id,ref.source_section_id,1});
    source.members.insert(source.members.begin()+3+i,ref.source_element_id);}
  source.packets[1].member_count+=127;
  for(unsigned p=2;p<5;++p){source.packets[p].native_first+=127;source.packets[p].member_begin+=127;}
  source.partitions[0].member_count+=127;
  auto input=source.ControlledInput();input.controls.native_nvsiz=129;input.controls.compiled_mvsiz=130;
  s::Model model;ASSERT_TRUE(model.Initialize(source.Domain(),input));
  auto c=Config(model.domain()->node_count());c.profile=s::BatchProfile::PhysicalCinSourceControlsV3;s::BatchForecast f;
  const auto status=s::Batch::Forecast(c,model,f);EXPECT_FALSE(status);
  EXPECT_STREQ(status.message,"Native NEL exceeds admitted controlled CTA width");
}
} // namespace controlled_resident_test
