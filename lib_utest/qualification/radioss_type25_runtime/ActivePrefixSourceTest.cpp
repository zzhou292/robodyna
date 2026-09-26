// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FullLedgerFixture.h"
namespace type25_source_test {
TEST(NativeActivePrefixSource, RealQbatAndOnePointPoliciesRetainTheirOrderedPhysicalSource) {
  FullLedgerFixture f(true);auto config=f.Config();auto source=f.Contact();
  config.activity=n::ContactActivityPolicy::AllActivePrefix;source.contact_thickness_update=0;
  rd::SourceStaging out;const auto report=rd::PrepareSource(config,source,f.physical,{},out);
  ASSERT_EQ(report.status,n::TransactionStatus::Ok)<<report.message;
  ASSERT_EQ(out.primary.size(),2u);
  const auto& q=f.shells.qbat_nodes(0);
  for(unsigned slot=0;slot<4;++slot)
    EXPECT_EQ(out.ids[out.primary[0].nodes[slot]],f.shells.active_nodes()[q[slot]].source_id);
  EXPECT_EQ(f.failure.parent_count(),4u);
  EXPECT_EQ(f.failure.parent(2)->policy,tl::fea::ShellFailurePolicy::ConstantAllPoints);
  EXPECT_EQ(f.failure.parent(3)->policy,tl::fea::ShellFailurePolicy::ConstantAllPoints);
}
TEST(NativeActivePrefixSource, MissingEvolutionAndWrongScopeRemainClosedWithoutPublication) {
  FullLedgerFixture f(true);
  for(unsigned variant=0;variant<6;++variant) {
    auto config=f.Config();auto source=f.Contact();
    config.activity=n::ContactActivityPolicy::AllActivePrefix;source.contact_thickness_update=0;
    if(variant==0)source.contact_thickness_update=-1;
    if(variant==1)source.contact_thickness_update=1;
    if(variant==2)config.activity=static_cast<n::ContactActivityPolicy>(99);
    if(variant==3)config.physical_source=n::PhysicalSourceProfile::QephT3Only;
    if(variant==4)config.response_mass=n::ResponseMassPolicy::StaticPhysicalLedger;
    if(variant==5)config.activity=n::ContactActivityPolicy::NoDeclaredFailure;
    rd::SourceStaging out;out.bytes=91;out.ids={88};
    EXPECT_EQ(rd::PrepareSource(config,source,f.physical,{},out).status,n::TransactionStatus::UnsupportedProfile)<<variant;
    EXPECT_EQ(out.bytes,91u);EXPECT_EQ(out.ids,(std::vector<std::uint64_t>{88}));
  }
}
} // namespace type25_source_test
