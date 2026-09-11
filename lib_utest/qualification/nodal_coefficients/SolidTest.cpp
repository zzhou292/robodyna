// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SolidFixture.h"

namespace coefficient_test {
TEST(SolidCoefficients, RealFormulationsSharedTopologySourceSlotsAndNoInventedInertia) {
  SolidFixture f;
  auto domain=f.Domain();
  auto solid=f.Solids(domain);
  ASSERT_TRUE(solid.prepared());
  ASSERT_EQ(solid.parents().size(),3u);
  const double expected[3]{1070*.04*.03*.02/8,1980*.04*.03*.02/8,1980*.04*.03*.02/12};
  for(unsigned e=0;e<3;++e) {
    const auto& parent=solid.parents()[e];
    EXPECT_EQ(parent.source_element_id,9100+e);
    EXPECT_EQ(parent.node_count,e==2?6u:8u);
    for(unsigned k=0;k<parent.node_count;++k) {
      EXPECT_EQ(domain.nodes()[parent.domain_node[k]].source_id,parent.source_node_id[k]);
      EXPECT_NEAR(parent.mass_kg[k],expected[e],1e-14);
    }
    EXPECT_EQ(Bits(parent.isotropic_inertia_kg_m2()),Bits(0.0));
  }
  auto map=f.Map(domain);
  fe::NodalCoefficientLedger old, joined;
  ASSERT_TRUE(old.Initialize({&map}));
  ASSERT_TRUE(joined.InitializeWithSolids({{&map},nullptr,&solid}));
  EXPECT_EQ(joined.scope().occurrences.solid18,8u);
  EXPECT_EQ(joined.scope().occurrences.solid24,8u);
  EXPECT_EQ(joined.scope().occurrences.solid6z,6u);
  EXPECT_EQ(joined.scope().solid18_parents,1u);
  EXPECT_EQ(joined.scope().solid24_parents,1u);
  EXPECT_EQ(joined.scope().solid6z_parents,1u);
  for(std::size_t n=0;n<domain.node_count();++n) {
    double sum=old.nodes()[n].coefficients.mass;
    for(const auto& parent:solid.parents())
      for(unsigned k=0;k<parent.node_count;++k)
        if(parent.domain_node[k]==n) sum+=parent.mass_kg[k];
    EXPECT_EQ(Bits(joined.nodes()[n].coefficients.mass),Bits(sum));
    EXPECT_EQ(Bits(joined.nodes()[n].coefficients.isotropic_inertia),
              Bits(old.nodes()[n].coefficients.isotropic_inertia));
  }
  const auto interior=domain.Find(f.a.source_node_id[6]);
  EXPECT_GT(joined.nodes()[interior].coefficients.mass,0);
  EXPECT_EQ(Bits(joined.nodes()[interior].coefficients.isotropic_inertia),Bits(0.0));
  EXPECT_FALSE(joined.Matches({&map}));
  EXPECT_FALSE(joined.MatchesWithElementMass({{&map},nullptr}));
  EXPECT_TRUE(joined.MatchesWithSolids({{&map},nullptr,&solid}));
  auto independent=f.Solids(domain);
  EXPECT_TRUE(solid.Matches(independent));
}

TEST(SolidCoefficients, LastReferenceIdentityCoordinateAndCountFailuresPreserveRetry) {
  SolidFixture f;
  auto domain=f.Domain();
  fe::solid18::Reference a; fe::solid24::Reference b; fe::solid6z::Reference c;
  ASSERT_EQ(fe::solid18::InitializeReference(f.a,a),fe::solid18::Status::Success);
  ASSERT_EQ(fe::solid24::InitializeReference(f.b,b),fe::solid24::Status::Success);
  auto wrong=f.c;
  wrong.position_m[5].z=std::nextafter(wrong.position_m[5].z,1.0);
  ASSERT_EQ(fe::solid6z::InitializeReference(wrong,c),fe::solid6z::Status::Success);
  fe::SolidNodeContributions failed;
  auto report=failed.Initialize(domain,{1,&a,1,&b,1,&c,1});
  EXPECT_EQ(report.status,fe::NodalDomainStatus::PositionMismatch);
  EXPECT_EQ(report.node,2u); EXPECT_FALSE(failed.prepared());
  wrong=f.c; wrong.source_element_id=f.a.source_element_id;
  ASSERT_EQ(fe::solid6z::InitializeReference(wrong,c),fe::solid6z::Status::Success);
  EXPECT_EQ(failed.Initialize(domain,{1,&a,1,&b,1,&c,1}).status,fe::NodalDomainStatus::DuplicateIdentity);
  ASSERT_EQ(fe::solid6z::InitializeReference(f.c,c),fe::solid6z::Status::Success);
  fe::SolidCoefficientLimits limits; limits.max_parents=2;
  EXPECT_EQ(failed.Initialize(domain,{1,&a,1,&b,1,&c,1},limits).status,fe::NodalDomainStatus::ResourceLimit);
  EXPECT_EQ(failed.Initialize(domain,{1,&a,SIZE_MAX}).status,fe::NodalDomainStatus::ResourceLimit);
  ASSERT_TRUE(failed.Initialize(domain,{1,&a,1,&b,1,&c,1}));
  EXPECT_EQ(failed.Initialize(domain,{1,&a,1}).status,fe::NodalDomainStatus::AlreadyInitialized);
  fe::SolidNodeContributions empty;
  fe::solid18::Reference virgin;
  EXPECT_FALSE(empty.Initialize(domain,{1,&virgin,1}));
  EXPECT_FALSE(empty.Initialize(domain,{1,&a,0}));
  EXPECT_FALSE(empty.Initialize(domain,{2,&a,1}));
  ASSERT_TRUE(empty.Initialize(domain,{1,&a,1}));
}

TEST(SolidCoefficients, SingleLedgerRejectsCrossFamilyCollisionAndIncludesExactBoundedBacking) {
  SolidFixture f;
  auto domain=f.Domain();
  auto map=f.Map(domain);
  auto solids=f.Solids(domain);
  const fe::ElementMassSource point{9800,f.a.source_node_id[0],domain.Find(f.a.source_node_id[0]),.002};
  fe::ElementMassContributions mass;
  ASSERT_TRUE(mass.Initialize(domain,{1,1000,&point,1}));
  fe::NodalCoefficientLedger joined;
  ASSERT_TRUE(joined.InitializeWithSolids({{&map},&mass,&solids}));
  EXPECT_EQ(joined.totals().element_mass,2.0);
  fe::CoefficientLimits limits; limits.max_host_bytes=joined.startup_payload_bytes()-1;
  fe::NodalCoefficientLedger retry;
  EXPECT_EQ(retry.InitializeWithSolids({{&map},&mass,&solids},limits).status,S::ResourceLimit);
  EXPECT_FALSE(retry.prepared());
  ++limits.max_host_bytes;
  ASSERT_TRUE(retry.InitializeWithSolids({{&map},&mass,&solids},limits));
  EXPECT_TRUE(joined.Matches(retry));
  // A new prepared solid may be valid by itself but reuse a structural shell EID.
  f.c.source_element_id=f.shells.qeph_source_id(0);
  auto collision=f.Solids(domain);
  fe::NodalCoefficientLedger failed;
  const auto report=failed.InitializeWithSolids({{&map},&mass,&collision});
  EXPECT_EQ(report.status,S::DuplicateIdentity);
  EXPECT_EQ(report.producer,fe::CoefficientProducer::Solid6z);
  EXPECT_FALSE(failed.prepared());
  ASSERT_TRUE(failed.InitializeWithSolids({{&map},&mass,&solids}));
  // Coefficient-only identity retains exact domain/source/mass, not an asserted
  // equivalence between mechanics profiles which happen to have equal mass.
  auto other_domain=f.Domain(2);
  fe::SolidNodeContributions wrong_domain;
  fe::solid18::Reference ref;
  ASSERT_EQ(fe::solid18::InitializeReference(f.a,ref),fe::solid18::Status::Success);
  ASSERT_TRUE(wrong_domain.Initialize(other_domain,{2,&ref,1}));
  fe::NodalCoefficientLedger mismatch;
  EXPECT_EQ(mismatch.InitializeWithSolids({{&map},nullptr,&wrong_domain}).status,S::IdentityMismatch);
}
}  // namespace coefficient_test
