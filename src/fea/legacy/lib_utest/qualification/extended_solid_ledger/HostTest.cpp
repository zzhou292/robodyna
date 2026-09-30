// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/extended_solid_coefficients/Fixture.h"
using namespace extended_solid_test;
namespace {
using CS=fe::CoefficientStatus;
constexpr auto Order=fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_V4;
fe::SolidNodeContributions Snapshot(const Fixture& f,const fe::NodalNodeDomain& domain) {
  fe::SolidNodeContributions result;
  const auto report=result.Initialize(domain,f.Input());
  EXPECT_TRUE(report)<<report.message;
  return result;
}
}
TEST(ExtendedSolidLedger, NativeSourceSlotsAddInOrderAndKeepZeroRotationalMass) {
  Fixture f;const auto domain=f.Domain();const auto map=f.Map(domain);
  const auto solids=Snapshot(f,domain);
  const auto node=solids.parents()[3].domain_node[4];
  // The independent mass namespace may reuse a solid EID at a repeated slot.
  const fe::ElementMassSource source{f.rear_input.source_element_id,
    domain.nodes()[node].source_id,node,.002};
  fe::ElementMassContributions mass;
  ASSERT_TRUE(mass.Initialize(domain,{1,1000,&source,1}));
  fe::NodalCoefficientLedger base,joined;
  ASSERT_TRUE(base.InitializeWithElementMass({{&map},&mass}));
  ASSERT_TRUE(joined.InitializeWithExtendedSolids({{&map},&mass,&solids}));
  EXPECT_EQ(joined.order(),Order);
  EXPECT_EQ(joined.scope().solid18_law44_parents,1u);
  EXPECT_EQ(joined.scope().solid18_law90_parents,1u);
  EXPECT_EQ(joined.scope().occurrences.solid18_law44,8u);
  EXPECT_EQ(joined.scope().occurrences.solid18_law90,8u);
  EXPECT_EQ(joined.nodes()[node].occurrences.solid18_law44,2u);
  double total=0,rear_total=0,foam_total=0;
  for(std::size_t n=0;n<domain.node_count();++n) {
    double value=base.nodes()[n].coefficients.mass,rear=0,foam=0;
    for(const auto& parent:solids.parents()) {
      for(unsigned k=0;k<parent.node_count;++k)if(parent.domain_node[k]==n) {
        value+=parent.mass_kg[k];
        if(parent.family==Family::Solid18Law44)rear+=parent.mass_kg[k];
        if(parent.family==Family::Solid18Law90)foam+=parent.mass_kg[k];
      }
    }
    const auto& actual=joined.nodes()[n].coefficients;
    EXPECT_EQ(coefficient_test::Bits(actual.mass),coefficient_test::Bits(value));
    EXPECT_EQ(coefficient_test::Bits(actual.solid18_law44_mass),coefficient_test::Bits(rear));
    EXPECT_EQ(coefficient_test::Bits(actual.solid18_law90_mass),coefficient_test::Bits(foam));
    EXPECT_EQ(coefficient_test::Bits(actual.isotropic_inertia),
              coefficient_test::Bits(base.nodes()[n].coefficients.isotropic_inertia));
    total+=value;rear_total+=rear;foam_total+=foam;
  }
  EXPECT_EQ(coefficient_test::Bits(joined.totals().mass),coefficient_test::Bits(total));
  EXPECT_EQ(coefficient_test::Bits(joined.totals().solid18_law44_mass),coefficient_test::Bits(rear_total));
  EXPECT_EQ(coefficient_test::Bits(joined.totals().solid18_law90_mass),coefficient_test::Bits(foam_total));
  EXPECT_EQ(joined.nodes()[node].coefficients.isotropic_inertia,0);
  EXPECT_EQ(joined.scope().covered_nodes+joined.scope().uncovered_nodes,domain.node_count());
}
TEST(ExtendedSolidLedger, VersionIdentityAndRetainedBackingSurviveBorrowedInputLifetime) {
  const auto make=[] {
    Fixture f;const auto domain=f.Domain();const auto map=f.Map(domain);
    const auto solids=Snapshot(f,domain);fe::NodalCoefficientLedger result;
    EXPECT_TRUE(result.InitializeWithExtendedSolids({{&map},nullptr,&solids}));
    return result;
  };
  const auto first=make(),independent=make();const auto copy=first;
  EXPECT_TRUE(first.Matches(independent));EXPECT_TRUE(first.Matches(copy));
  EXPECT_TRUE(first.MatchesWithExtendedSolids({{independent.shells()},nullptr,independent.solids()}));
  EXPECT_FALSE(first.MatchesWithSolids({{independent.shells()},nullptr,independent.solids()}));
  Fixture f;const auto domain=f.Domain();const auto map=f.Map(domain);
  const auto original=f.Solids(domain),extended=Snapshot(f,domain);
  fe::NodalCoefficientLedger retry;
  EXPECT_EQ(retry.InitializeWithSolids({{&map},nullptr,&extended}).status,CS::IdentityMismatch);
  EXPECT_EQ(retry.InitializeWithExtendedSolids({{&map},nullptr,&original}).status,CS::IdentityMismatch);
  EXPECT_EQ(retry.InitializeWithExtendedSolids({{&map},nullptr,nullptr}).status,CS::IdentityMismatch);
  EXPECT_FALSE(retry.prepared());
  ASSERT_TRUE(retry.InitializeWithSolids({{&map},nullptr,&original}));
  EXPECT_FALSE(first.Matches(retry));EXPECT_FALSE(retry.Matches(first));
  EXPECT_EQ(retry.totals().solid18_law44_mass,0);
  EXPECT_EQ(retry.totals().solid18_law90_mass,0);
  EXPECT_EQ(retry.scope().solid18_law44_parents,0u);
  EXPECT_EQ(retry.scope().solid18_law90_parents,0u);
}
TEST(ExtendedSolidLedger, FinalFamilyStructuralCollisionAndWrongDomainLeaveRetryIntact) {
  Fixture f;const auto domain=f.Domain();const auto map=f.Map(domain);
  const auto good=Snapshot(f,domain);
  for(const auto family:{Family::Solid18Law44,Family::Solid18Law90}) {
    Fixture bad;
    if(family==Family::Solid18Law44) {
      bad.rear_input.source_element_id=f.shells.qeph_source_id(0);
      ASSERT_EQ(fe::solid18::law44::InitializeReference(bad.rear_input,bad.rear),fe::solid18::Status::Success);
    } else {
      bad.foam_input.source_element_id=f.shells.qeph_source_id(0);
      ASSERT_EQ(fe::solid18::total_strain::InitializeReference90(bad.foam_input,bad.foam),fe::solid18::Status::Success);
    }
    const auto collision=Snapshot(bad,domain);fe::NodalCoefficientLedger retry;
    const auto report=retry.InitializeWithExtendedSolids({{&map},nullptr,&collision});
    EXPECT_EQ(report.status,CS::DuplicateIdentity);
    EXPECT_EQ(report.producer,family==Family::Solid18Law44?
      fe::CoefficientProducer::Solid18Law44:fe::CoefficientProducer::Solid18Law90);
    EXPECT_FALSE(retry.prepared());
    ASSERT_TRUE(retry.InitializeWithExtendedSolids({{&map},nullptr,&good}));
  }
  const auto wrong_domain=f.Domain(2);auto input=f.Input();input.source_instance_id=2;
  fe::SolidNodeContributions wrong;ASSERT_TRUE(wrong.Initialize(wrong_domain,input));
  fe::NodalCoefficientLedger retry;
  EXPECT_EQ(retry.InitializeWithExtendedSolids({{&map},nullptr,&wrong}).status,CS::IdentityMismatch);
  ASSERT_TRUE(retry.InitializeWithExtendedSolids({{&map},nullptr,&good}));
}
TEST(ExtendedSolidLedger, CompleteBackingExactCapAndParentCapAreEnforcedBeforePublication) {
  Fixture f;const auto domain=f.Domain();const auto map=f.Map(domain);
  const auto solid=Snapshot(f,domain);fe::NodalCoefficientLedger reference,retry;
  const fe::NodalCoefficientSourcesWithSolids input{{&map},nullptr,&solid};
  ASSERT_TRUE(reference.InitializeWithExtendedSolids(input));
  fe::CoefficientLimits limits;limits.max_solid_parents=4;
  EXPECT_EQ(retry.InitializeWithExtendedSolids(input,limits).status,CS::ResourceLimit);
  limits.max_solid_parents=5;limits.max_host_bytes=reference.startup_payload_bytes()-1;
  EXPECT_EQ(retry.InitializeWithExtendedSolids(input,limits).status,CS::ResourceLimit);
  EXPECT_FALSE(retry.prepared());
  ++limits.max_host_bytes;
  ASSERT_TRUE(retry.InitializeWithExtendedSolids(input,limits));
  EXPECT_TRUE(reference.Matches(retry));
  EXPECT_EQ(retry.owned_payload_bytes(),reference.owned_payload_bytes());
  EXPECT_EQ(retry.InitializeWithExtendedSolids(input).status,CS::AlreadyInitialized);
}
