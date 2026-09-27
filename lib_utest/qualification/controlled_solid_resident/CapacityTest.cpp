// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../extended_solid_resident/OwnerFixture.h"
namespace controlled_resident_test {
using namespace extended_resident_test;
TEST(ControlledResidentCapacity, ExplicitEnvelopePreservesLegacyLimitsAndHardCeilings) {
  EXPECT_EQ(s::BatchLimits{}.max_device_bytes,128u<<20);
  EXPECT_EQ(s::BatchLimits{}.max_host_bytes,256u<<20);
  const auto limits=s::SourceControlledBatchLimits();
  EXPECT_EQ(limits.max_device_bytes,192u<<20);EXPECT_EQ(limits.max_host_bytes,256u<<20);
  OwnerFixture fixture(false,true,{.001,1000,1});auto c=fixture.Configuration();c.limits=limits;
  s::BatchForecast value;ASSERT_TRUE(s::Batch::Forecast(c,fixture.model,value));
  c.limits.max_device_bytes=limits.max_device_bytes+1;EXPECT_FALSE(s::Batch::Forecast(c,fixture.model,value));
  c.limits=limits;c.limits.max_host_bytes=limits.max_host_bytes+1;EXPECT_FALSE(s::Batch::Forecast(c,fixture.model,value));
  OwnerFixture legacy;auto old=legacy.Configuration();old.limits=limits;
  EXPECT_FALSE(s::Batch::Forecast(old,legacy.model,value));
}
TEST(ControlledResidentCapacity, CompleteControlledCountShapeRejects128AndFits192WithExactBoundary) {
  auto c=Config(376934);c.profile=s::BatchProfile::PhysicalCinSourceControlsV3;
  c.cin_attachment_count=11165;
  // Complete bound-model family/material/native-packet counts. The maximum
  // admitted curve pool is used, so actual curve counts cannot exceed this arena.
  d::Counts n{908,2341,0,1,12,s::BatchLimits{}.max_curve_points,386,1345,3,1};
  n.controlled.h24=2341;n.controlled.foam=1345;n.controlled.packets=47;
  n.controlled.members=4980;n.controlled.workers=512;
  for(unsigned i=0;i<4;++i)n.controlled.worker_begin[i]=128*i;
  d::ArenaLayout layout;layout.bytes=77;
  EXPECT_FALSE(d::MakeLayout(n,c,layout));EXPECT_EQ(layout.bytes,77u);
  c.limits=s::SourceControlledBatchLimits();ASSERT_TRUE(d::MakeLayout(n,c,layout));
  EXPECT_EQ(layout.bytes,152470008u);EXPECT_EQ(layout.staging_bytes,19880688u);
  EXPECT_EQ(layout.controlled.workspace.bytes,17993728u);
  RecordProperty("complete_count_shape_max_curve_device_bytes",layout.bytes);
  RecordProperty("complete_count_shape_host_staging_bytes",layout.staging_bytes);
  RecordProperty("complete_count_shape_live_proof_bytes",layout.proof.bytes);
  const auto exact=layout.bytes;c.limits.max_device_bytes=exact;
  ASSERT_TRUE(d::MakeLayout(n,c,layout));--c.limits.max_device_bytes;
  layout.bytes=91;EXPECT_FALSE(d::MakeLayout(n,c,layout));EXPECT_EQ(layout.bytes,91u);
  c.limits=s::SourceControlledBatchLimits();c.profile=s::BatchProfile::PhysicalCinExtendedLaw44Law90V2;
  EXPECT_FALSE(d::MakeLayout(n,c,layout));
}
} // namespace controlled_resident_test
