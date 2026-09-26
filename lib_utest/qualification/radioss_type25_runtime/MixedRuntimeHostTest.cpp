// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MixedRuntimeFixture.h"
namespace type25_source_test {
TEST(NativeMixedRuntimeSource, CompleteRealOriginsAndSignedInternalFaceStageWithoutFakeParent) {
  FullLedgerFixture f(true,true);MixedRuntimeSource source(f);n::runtime_detail::SourceStaging staged;
  const auto report=n::runtime_detail::PrepareSource(source.Config(),source.Source(),f.physical,{},staged);
  ASSERT_EQ(report.status,n::TransactionStatus::Ok)<<report.message;
  EXPECT_TRUE(staged.moving.mixed);EXPECT_EQ(staged.primary.size(),3u);
  EXPECT_EQ(staged.moving.topology.main_count,5u);EXPECT_EQ(staged.moving.topology.mixed_maps.primary_to_partner[2],0u);
  EXPECT_EQ(staged.primary[2].source_id,3u);EXPECT_LT(staged.main_stiffness[2],0);
  EXPECT_EQ(staged.moving.free_main_ids,(std::vector<std::uint32_t>{1,2,4,5}));
}
TEST(NativeMixedRuntimeSource, ForeignPhysicalOwnerAndMismatchedCacheCannotPublish) {
  FullLedgerFixture f(true,true);MixedRuntimeSource source(f);n::runtime_detail::SourceStaging out;
  out.bytes=999;
  for(unsigned fault=0;fault<4;++fault) {
    auto value=source.Source();auto config=source.Config();std::array<s::PostGapmMainSupport,5> changed=source.supports;
    auto post=source.post;
    if(fault==0)value.primary_parent_ids=f.parents.data();
    if(fault==1){config.lifecycle.main_coefficient_domain=n::MainCoefficientDomain::Nonnegative;}
    if(fault==2){changed[2].first.source_element_id=9999;post.final_support=changed.data();value.starter.post_gapm=&post;}
    if(fault==3)f.mains[0].normal_slot[0].z+=1;
    const auto report=n::runtime_detail::PrepareSource(config,value,f.physical,{},out);
    EXPECT_NE(report.status,n::TransactionStatus::Ok);EXPECT_EQ(out.bytes,999u);
    if(fault==3)f.mains[0].normal_slot[0]=source.starter.starter.face_normals[0];
  }
  EXPECT_EQ(n::runtime_detail::PrepareSource(source.Config(),source.Source(),f.physical,{},out).status,n::TransactionStatus::Ok);
}
}
