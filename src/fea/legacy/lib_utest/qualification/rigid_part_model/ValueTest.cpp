// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"

namespace rigid_part_model_test {
TEST(RigidPartModel, CompleteTopologyPrimaryMergeAndCoefficientPartitions) {
  Fixture f;
  const auto ledger=f.Ledger();const auto topology=f.Topology();
  Model model;
  const auto report=model.Initialize(*topology,ledger,f.units);
  ASSERT_TRUE(report)<<report.message;
  ASSERT_EQ(model.original_bodies().size(),4u);ASSERT_EQ(model.roots().size(),2u);
  EXPECT_TRUE(model.coefficients()->Matches(ledger));
  EXPECT_NE(model.topology(),topology.get());
  EXPECT_TRUE(model.Matches(*topology,ledger,f.units));
  for(std::size_t p=0;p<f.part.size();++p) {
    const auto& body=model.original_bodies()[p];
    EXPECT_EQ(body.raw.ledger.primaries,1u);
    EXPECT_EQ(Bits(body.primary.mass),Bits(1e-20*1000));
    EXPECT_EQ(Bits(body.primary.inertia),Bits((1e-20*1000)*.001*.001));
    const auto& source=topology->parts()[p];
    std::uint64_t previous=0;
    for(std::size_t n=0;n<source.member_count;++n) {
      const auto global=model.members()[source.member_offset+n].domain_node;
      const auto id=ledger.domain()->nodes()[global].source_id;
      EXPECT_GT(id,previous);previous=id;
      EXPECT_EQ(model.RootForDomainNode(global),source.root_index);
    }
  }
  for(const auto& root:model.roots()) {
    EXPECT_EQ(root.value.raw.ledger.primaries,2u);
    EXPECT_EQ(Bits(root.value.regularization.primary_mass_kg),Bits(2*(1e-20*1000)));
  }
  EXPECT_EQ(model.RootForDomainNode(4),SIZE_MAX); // Independent shell node outside these bodies.
  EXPECT_EQ(model.RootForDomainNode(SIZE_MAX),SIZE_MAX);
  const auto n=ledger.domain()->Find(f.extra[0][0]);
  EXPECT_EQ(Bits(model.coefficients()->nodes()[n].coefficients.isotropic_inertia),Bits(0.0));
  EXPECT_GT(model.coefficients()->nodes()[0].coefficients.shell.physical_inertia,0);
}
TEST(RigidPartModel, SourceOrderIdentityAndSortedReductionAreSeparate) {
  Fixture f;
  const auto ledger=f.Ledger();const auto topology=f.Topology();
  Model a;ASSERT_TRUE(a.Initialize(*topology,ledger,f.units));
  for(auto& p:f.part) std::reverse(p.begin(),p.end());
  const auto reversed=f.Topology();
  Model b;ASSERT_TRUE(b.Initialize(*reversed,ledger,f.units));
  EXPECT_FALSE(a.Matches(b));
  for(std::size_t p=0;p<a.original_bodies().size();++p) {
    const auto x=RawValues(a.original_bodies()[p].raw),y=RawValues(b.original_bodies()[p].raw);
    for(unsigned k=0;k<x.size();++k) EXPECT_EQ(Bits(x[k]),Bits(y[k]));
  }
  const auto retained=[] {
    Fixture local;Model out;
    EXPECT_TRUE(out.Initialize(*local.Topology(),local.Ledger(),local.units));
    return out;
  }();
  EXPECT_TRUE(retained.Matches(a));
  EXPECT_TRUE(Model(a).Matches(a));
  f.nodes.back().position.x=-0.0;
  const auto minus=f.Ledger();f.nodes.back().position.x=0.0;
  const auto plus=f.Ledger();
  Model m,z;ASSERT_TRUE(m.Initialize(*reversed,minus,f.units));
  ASSERT_TRUE(z.Initialize(*reversed,plus,f.units));
  EXPECT_FALSE(m.Matches(z));
}
TEST(RigidPartModel, LateMissingMemberCoverageUnitsAndExactBudgetRetry) {
  Fixture f;
  const auto ledger=f.Ledger();const auto topology=f.Topology();
  Model expected;ASSERT_TRUE(expected.Initialize(*topology,ledger,f.units));
  using S=r::PartAssemblyStatus;
  for(unsigned kind=0;kind<4;++kind) {
    auto input=f.Topology(kind==0?2:1);
    if(kind==1) {
      const auto saved=f.part.back().back();
      f.part.back().back()=999999;input=f.Topology();f.part.back().back()=saved;
    }
    Model model;const auto before=Bytes(model);
    const auto missing=f.Ledger(false);
    auto units=f.units;if(kind==3) units.length_to_m=0;
    const auto report=model.Initialize(*input,kind==2?missing:ledger,units);
    EXPECT_FALSE(report)<<kind;EXPECT_EQ(Bytes(model),before);EXPECT_FALSE(model.prepared());
    ASSERT_TRUE(model.Initialize(*topology,ledger,f.units));EXPECT_TRUE(model.Matches(expected));
    EXPECT_EQ(model.Initialize(*topology,ledger,f.units).status,S::AlreadyInitialized);
  }
  Model limited;
  auto cap=r::PartAssemblyLimits{};cap.max_host_bytes=expected.startup_payload_bytes()-1;
  EXPECT_EQ(limited.Initialize(*topology,ledger,f.units,cap).status,S::ResourceLimit);
  ++cap.max_host_bytes;
  ASSERT_TRUE(limited.Initialize(*topology,ledger,f.units,cap));
  EXPECT_EQ(limited.startup_payload_bytes(),cap.max_host_bytes);
  EXPECT_LT(limited.owned_payload_bytes(),cap.max_host_bytes);
  EXPECT_TRUE(limited.Matches(expected));
}
TEST(RigidPartModel, TwentyTwoPrimariesTwentyRootsAnd796MemberCapacity) {
  std::vector<std::size_t> counts(22,2);counts[0]=399;counts[1]=396; // +one parent extra=796.
  Fixture f(counts);
  Model model;const auto c=f.Ledger();const auto t=f.Topology();
  const auto report=model.Initialize(*t,c,f.units);ASSERT_TRUE(report)<<report.message;
  EXPECT_EQ(model.original_bodies().size(),22u);EXPECT_EQ(model.roots().size(),20u);
  EXPECT_EQ(model.topology()->roots()[0].member_count,796u);
  std::size_t primaries=0,members=0;
  for(const auto& root:model.roots()) {
    primaries+=root.value.raw.ledger.primaries;
    members+=root.value.raw.ledger.part_members+root.value.raw.ledger.extra_members;
  }
  EXPECT_EQ(primaries,22u);EXPECT_EQ(members,t->member_count());
  EXPECT_LT(model.startup_payload_bytes(),4u*1024*1024);
  RecordProperty("retained_bytes",model.owned_payload_bytes());
  RecordProperty("startup_bytes",model.startup_payload_bytes());
}
TEST(RigidPartModel, LastBodyFiniteCoordinateOverflowDoesNotPublishEarlierBodies) {
  Fixture f;
  const auto clean=f.Ledger();const auto topology=f.Topology();
  Model expected;ASSERT_TRUE(expected.Initialize(*topology,clean,f.units));
  f.nodes.back().position.x=1e200;
  const auto overflow=f.Ledger();
  Model model;const auto before=Bytes(model);
  const auto report=model.Initialize(*topology,overflow,f.units);
  EXPECT_EQ(report.status,r::PartAssemblyStatus::NonfiniteResult);
  EXPECT_EQ(report.part,3u);EXPECT_EQ(Bytes(model),before);
  EXPECT_EQ(model.original_bodies().size(),0u);EXPECT_EQ(model.roots().size(),0u);
  ASSERT_TRUE(model.Initialize(*topology,clean,f.units));EXPECT_TRUE(model.Matches(expected));
}
} // namespace rigid_part_model_test
