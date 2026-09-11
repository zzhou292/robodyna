// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
using namespace extended_solid_test;
TEST(ExtendedSolidCoefficients, FiveTypedFamiliesRetainSourceOrderAndRepeatedMassSlots) {
  Fixture f;auto domain=f.Domain();fe::SolidNodeContributions result;
  ASSERT_TRUE(result.Initialize(domain,f.Input()));
  EXPECT_EQ(result.profile(),Profile::ExtendedLaw44Law90);
  ASSERT_EQ(result.parents().size(),5u);
  const Family families[]{Family::Solid18,Family::Solid24,Family::Solid6z,Family::Solid18Law44,Family::Solid18Law90};
  for(unsigned e=0;e<5;++e) {
    EXPECT_EQ(result.parents()[e].family,families[e]);
    EXPECT_EQ(result.parent_count(families[e]),1u);
    EXPECT_EQ(result.parents()[e].node_count,e==2?6u:8u);
  }
  const auto& rear=result.parents()[3];
  EXPECT_EQ(rear.domain_node[4],rear.domain_node[5]);
  EXPECT_EQ(rear.domain_node[6],rear.domain_node[7]);
  for(unsigned k=0;k<8;++k) {
    EXPECT_EQ(coefficient_test::Bits(rear.mass_kg[k]),coefficient_test::Bits(f.rear.mass().source_nodal_mass_kg[k]));
    EXPECT_EQ(domain.nodes()[rear.domain_node[k]].source_id,f.rear_input.source_node_id[k]);
    EXPECT_EQ(coefficient_test::Bits(result.parents()[4].mass_kg[k]),
              coefficient_test::Bits(f.foam.mass().source_nodal_mass_kg[k]));
  }
  double node_mass=0;unsigned occurrences=0;
  for(unsigned k=0;k<8;++k)if(rear.domain_node[k]==rear.domain_node[4]) {
    node_mass+=rear.mass_kg[k];++occurrences;
  }
  EXPECT_EQ(occurrences,2u);
  EXPECT_DOUBLE_EQ(node_mass,f.rear.mass().source_nodal_mass_kg[4]+f.rear.mass().source_nodal_mass_kg[5]);
  EXPECT_EQ(coefficient_test::Bits(rear.isotropic_inertia_kg_m2()),coefficient_test::Bits(0.0));
}
TEST(ExtendedSolidCoefficients, LegacyProfileAndLedgerCannotSilentlyAdmitNewFamilies) {
  Fixture f;auto domain=f.Domain();fe::SolidNodeContributions result;
  auto input=f.Input();input.profile=Profile::OriginalThreeFamilies;
  EXPECT_EQ(result.Initialize(domain,input).status,Status::InvalidInput);
  input=f.Input();input.profile=static_cast<Profile>(99);
  EXPECT_EQ(result.Initialize(domain,input).status,Status::InvalidInput);
  input=f.Input();input.law44=nullptr;input.law44_count=0;input.law90=nullptr;input.law90_count=0;
  EXPECT_EQ(result.Initialize(domain,input).status,Status::InvalidInput);
  ASSERT_TRUE(result.Initialize(domain,f.Input()));
  auto shells=f.Map(domain);fe::NodalCoefficientLedger ledger;
  EXPECT_EQ(ledger.InitializeWithSolids({{&shells},nullptr,&result}).status,fe::CoefficientStatus::IdentityMismatch);
  EXPECT_FALSE(ledger.prepared());
  auto legacy=f.Solids(domain);
  ASSERT_TRUE(ledger.InitializeWithSolids({{&shells},nullptr,&legacy}));
  EXPECT_EQ(legacy.profile(),Profile::OriginalThreeFamilies);
}
TEST(ExtendedSolidCoefficients, LastFamilyIdentityCoordinatesAndCountsRejectWithRetry) {
  Fixture f;auto domain=f.Domain();fe::SolidNodeContributions result;
  auto input=f.Input();input.law90=nullptr;
  EXPECT_EQ(result.Initialize(domain,input).status,Status::InvalidInput);
  input=f.Input();input.law90_count=SIZE_MAX;
  EXPECT_EQ(result.Initialize(domain,input).status,Status::ResourceLimit);
  auto changed=f.foam_input;changed.source_element_id=f.rear_input.source_element_id;
  fe::solid18::total_strain::Reference bad;
  ASSERT_EQ(fe::solid18::total_strain::InitializeReference90(changed,bad),fe::solid18::Status::Success);
  input=f.Input();input.law90=&bad;
  auto report=result.Initialize(domain,input);
  EXPECT_EQ(report.status,Status::DuplicateIdentity);EXPECT_EQ(report.node,4u);
  changed=f.foam_input;changed.position_m[7].z=std::nextafter(changed.position_m[7].z,2.0);
  ASSERT_EQ(fe::solid18::total_strain::InitializeReference90(changed,bad),fe::solid18::Status::Success);
  report=result.Initialize(domain,input);
  EXPECT_EQ(report.status,Status::PositionMismatch);EXPECT_EQ(report.node,4u);
  changed=f.foam_input;changed.source_node_id[7]+=1000000;
  ASSERT_EQ(fe::solid18::total_strain::InitializeReference90(changed,bad),fe::solid18::Status::Success);
  EXPECT_EQ(result.Initialize(domain,input).status,Status::MissingSource);
  EXPECT_FALSE(result.prepared());ASSERT_TRUE(result.Initialize(domain,f.Input()));
}
TEST(ExtendedSolidCoefficients, ExactByteCapRetainedIdentityAndBorrowedLifetime) {
  Fixture f;auto domain=f.Domain();fe::SolidNodeContributions result;
  ASSERT_TRUE(result.Initialize(domain,f.Input()));
  fe::SolidCoefficientLimits limits;limits.max_host_bytes=result.startup_payload_bytes()-1;
  fe::SolidNodeContributions retry;
  EXPECT_EQ(retry.Initialize(domain,f.Input(),limits).status,Status::ResourceLimit);
  EXPECT_FALSE(retry.prepared());++limits.max_host_bytes;
  ASSERT_TRUE(retry.Initialize(domain,f.Input(),limits));EXPECT_TRUE(retry.Matches(result));
  const auto previous=coefficient_test::Bits(result.parents()[4].mass_kg[0]);
  auto changed=f.foam_input;changed.density_kg_m3*=2;
  ASSERT_EQ(fe::solid18::total_strain::InitializeReference90(changed,f.foam),fe::solid18::Status::Success);
  fe::SolidNodeContributions other;ASSERT_TRUE(other.Initialize(domain,f.Input()));
  EXPECT_FALSE(other.Matches(result));EXPECT_EQ(coefficient_test::Bits(result.parents()[4].mass_kg[0]),previous);
  auto copy=result;EXPECT_TRUE(copy.Matches(result));
}
