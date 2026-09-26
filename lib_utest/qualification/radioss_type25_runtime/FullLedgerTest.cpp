// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FullLedgerFixture.h"
namespace type25_source_test {
namespace {
rd::SourceStaging Sentinel(){rd::SourceStaging out;out.bytes=991;out.ids={88};return out;}
void Unchanged(const rd::SourceStaging& out){EXPECT_EQ(out.bytes,991u);EXPECT_EQ(out.ids,(std::vector<std::uint64_t>{88}));}
}
TEST(NativeType25FullLedger, ActualContributorsAndQbatConnectivityStayInOneSourceDomain) {
  FullLedgerFixture f;const auto& scope=f.ledger.scope();
  ASSERT_EQ(scope.uncovered_nodes,0u);EXPECT_EQ(scope.qbat_parents,1u);
  EXPECT_GT(scope.type25_connections,0u);EXPECT_GT(scope.type13_connections,0u);EXPECT_EQ(scope.element_mass_records,1u);
  EXPECT_EQ(scope.solid18_parents,1u);EXPECT_EQ(scope.solid24_parents,1u);EXPECT_EQ(scope.solid6z_parents,1u);
  rd::SourceStaging output;
  const auto report=rd::PrepareSource(f.Config(),f.Contact(),f.physical,{},output);
  ASSERT_EQ(report.status,n::TransactionStatus::Ok)<<report.message;
  EXPECT_EQ(output.ids,f.ids);EXPECT_TRUE(output.native_mass.empty()); // Actual owner mass is borrowed during assembly.
  ASSERT_EQ(output.primary.size(),4u);
  const auto& qbat=f.source.shells.qbat_nodes(0);
  for(unsigned k=0;k<4;++k)
    EXPECT_EQ(output.ids[output.primary.back().nodes[k]],f.source.shells.active_nodes()[qbat[k]].source_id);
  EXPECT_GT(f.ledger.nodes()[f.domain.Find(10)].coefficients.solid18_mass,0);
  EXPECT_GT(f.ledger.nodes()[f.domain.Find(777)].coefficients.element_mass,0);
}
TEST(NativeType25FullLedger, LegacyDefaultStaticMassAndUnknownProfilesRemainRejected) {
  FullLedgerFixture f;
  for(unsigned variant=0;variant<3;++variant) {
    auto config=f.Config();auto out=Sentinel();
    if(variant==0)config.physical_source=n::PhysicalSourceProfile::QephT3Only;
    if(variant==1)config.response_mass=n::ResponseMassPolicy::StaticPhysicalLedger;
    if(variant==2)config.physical_source=static_cast<n::PhysicalSourceProfile>(99);
    EXPECT_EQ(rd::PrepareSource(config,f.Contact(),f.physical,{},out).status,n::TransactionStatus::UnsupportedProfile);
    Unchanged(out);
  }
}
TEST(NativeType25FullLedger, QbatSourceIdentityAndOrderedNodesCannotBeReplacedByAnotherFamily) {
  FullLedgerFixture f;const auto last=f.parents.size()-1;
  auto out=Sentinel();f.parents[last]=999;
  EXPECT_EQ(rd::PrepareSource(f.Config(),f.Contact(),f.physical,{},out).status,n::TransactionStatus::SourceMismatch);Unchanged(out);
  f.parents[last]=103;
  std::swap(f.mains[last].nodes[0],f.mains[last].nodes[1]);
  EXPECT_NE(rd::PrepareSource(f.Config(),f.Contact(),f.physical,{},out).status,n::TransactionStatus::Ok);Unchanged(out);
  std::swap(f.mains[last].nodes[0],f.mains[last].nodes[1]);
  EXPECT_EQ(rd::PrepareSource(f.Config(),f.Contact(),f.physical,{},out).status,n::TransactionStatus::Ok);
}
TEST(NativeType25FullLedger, MaterialFailureNeedsItsOwnLifecycleAdmission) {
  FullLedgerFixture f(true);auto out=Sentinel();
  const auto report=rd::PrepareSource(f.Config(),f.Contact(),f.physical,{},out);
  EXPECT_EQ(report.status,n::TransactionStatus::UnsupportedProfile);EXPECT_STREQ(report.message,"Contact activity changes are not admitted");Unchanged(out);
}
TEST(NativeType25FullLedger, CompleteParentAllocationIsBoundedAndRejectionDoesNotPublish) {
  FullLedgerFixture f;rd::SourceStaging valid;
  ASSERT_EQ(rd::PrepareSource(f.Config(),f.Contact(),f.physical,{},valid).status,n::TransactionStatus::Ok);
  n::TransactionLimits limits;limits.max_host_bytes=valid.bytes;
  auto out=Sentinel();ASSERT_EQ(rd::PrepareSource(f.Config(),f.Contact(),f.physical,limits,out).status,n::TransactionStatus::Ok);
  --limits.max_host_bytes;out=Sentinel();
  EXPECT_EQ(rd::PrepareSource(f.Config(),f.Contact(),f.physical,limits,out).status,n::TransactionStatus::ResourceLimit);Unchanged(out);
}
} // namespace type25_source_test
