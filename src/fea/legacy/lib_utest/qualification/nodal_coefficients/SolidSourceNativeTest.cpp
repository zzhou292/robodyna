// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../solid18_reference/SourceFixture.h"
#include "../solid18_reference/NativeOracle.h"
#include "../solid24_reference/SourceFixture.h"
#include "../solid24_reference/NativeOracle.h"
#include "../solid6z_reference/SourceFixture.h"
#include "../solid6z_reference/NativeOracle.h"
#include <map>

namespace coefficient_test {
TEST(SolidCoefficientsSource, All2412SelectedOriginalSolidsJoinOneDomainWithNativeMass) {
  Fixture shell_fixture;
  std::map<std::uint64_t,tl::math::Vec3> source_nodes;
  for(const auto& n:shell_fixture.nodes) source_nodes.emplace(n.source_id,n.position);
  auto collect=[&](const auto& input,unsigned arity) {
    for(unsigned k=0;k<arity;++k) {
      const auto inserted=source_nodes.emplace(input.source_node_id[k],input.position_m[k]);
      if(!inserted.second) {
        const auto a=inserted.first->second,b=input.position_m[k];
        EXPECT_EQ(Bits(a.x),Bits(b.x)); EXPECT_EQ(Bits(a.y),Bits(b.y)); EXPECT_EQ(Bits(a.z),Bits(b.z));
      }
    }
  };
  std::vector<fe::solid18::Reference> a(solid18_test::SourceCount);
  std::vector<fe::solid24::Reference> b;
  std::vector<fe::solid6z::Reference> c;
  std::map<std::uint64_t,long double> native_mass;
  long double expected_family[3]{};
  for(unsigned i=0;i<a.size();++i) {
    const auto input=solid18_test::Source(i);
    collect(input,8);
    ASSERT_EQ(fe::solid18::InitializeReference(input,a[i]),fe::solid18::Status::Success);
    const auto native=solid18_test::Native(input);
    ASSERT_EQ(native.status,0);
    for(unsigned k=0;k<8;++k) {
      const double mass=native.values[138+k];
      native_mass[input.source_node_id[k]]+=mass; expected_family[0]+=mass;
    }
  }
  for(unsigned i=0;i<solid24_test::SourceCount;++i) {
    const auto input=solid24_test::Source(i);
    if(solid24_test::IsBrick(input)) {
      collect(input,8);
      fe::solid24::Reference ref;
      ASSERT_EQ(fe::solid24::InitializeReference(input,ref),fe::solid24::Status::Success);
      b.push_back(ref);
      const auto native=solid24_test::Native(input);
      ASSERT_EQ(native.status,0);
      for(unsigned k=0;k<8;++k) {
        const double mass=native.values[35+k];
        native_mass[input.source_node_id[k]]+=mass; expected_family[1]+=mass;
      }
    } else {
      fe::solid6z::ReferenceInput wedge;
      ASSERT_TRUE(solid6z_test::Source(i,wedge)); collect(wedge,6);
      fe::solid6z::Reference ref;
      ASSERT_EQ(fe::solid6z::InitializeReference(wedge,ref),fe::solid6z::Status::Success);
      c.push_back(ref);
      const auto native=solid6z_test::Native(wedge);
      ASSERT_EQ(native.status,0);
      for(unsigned k=0;k<6;++k) {
        const double mass=native.values[40+k];
        native_mass[wedge.source_node_id[k]]+=mass; expected_family[2]+=mass;
      }
    }
  }
  ASSERT_EQ(a.size(),908u); ASSERT_EQ(b.size(),1309u); ASSERT_EQ(c.size(),195u);
  std::vector<fe::NodalDomainNode> nodes;
  for(auto it=source_nodes.rbegin();it!=source_nodes.rend();++it)
    nodes.push_back({it->first,it->second}); // Non-native order exercises the NID map.
  fe::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({1,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle()));
  fe::SolidNodeContributions solids;
  ASSERT_TRUE(solids.Initialize(domain,{1,a.data(),a.size(),b.data(),b.size(),c.data(),c.size()}));
  fe::ShellNodeMap shells;
  ASSERT_TRUE(shells.Initialize(shell_fixture.shells,domain,fe::ShellNodeMapLimits::Vehicle()));
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.InitializeWithSolids({{&shells},nullptr,&solids},fe::CoefficientLimits::Vehicle()));
  EXPECT_EQ(ledger.scope().solid18_parents,908u);
  EXPECT_EQ(ledger.scope().solid24_parents,1309u);
  EXPECT_EQ(ledger.scope().solid6z_parents,195u);
  for(const auto& [id,expected]:native_mass) {
    const auto n=domain.Find(id);
    const auto& row=ledger.nodes()[n];
    // These actual solid NIDs do not overlap the deliberately separate tiny
    // synthetic shell fixture; all their nodal scalar inertias must remain zero.
    ASSERT_EQ(row.occurrences.qeph+row.occurrences.t3+row.occurrences.qbat,0u);
    ASSERT_EQ(Bits(row.coefficients.isotropic_inertia),Bits(0.0));
    ASSERT_NEAR(row.coefficients.mass,static_cast<double>(expected),2e-11*static_cast<double>(expected))<<id;
  }
  EXPECT_NEAR(ledger.totals().solid18_mass,static_cast<double>(expected_family[0]),2e-11*expected_family[0]);
  EXPECT_NEAR(ledger.totals().solid24_mass,static_cast<double>(expected_family[1]),2e-11*expected_family[1]);
  EXPECT_NEAR(ledger.totals().solid6z_mass,static_cast<double>(expected_family[2]),2e-11*expected_family[2]);
  RecordProperty("original_solid_parents",solids.parents().size());
  RecordProperty("original_solid_nodes",native_mass.size());
  RecordProperty("ledger_retained_bytes",ledger.owned_payload_bytes());
  RecordProperty("ledger_startup_bytes",ledger.startup_payload_bytes());
  RecordProperty("solid18_mass_kg",std::to_string(ledger.totals().solid18_mass));
  RecordProperty("solid24_mass_kg",std::to_string(ledger.totals().solid24_mass));
  RecordProperty("solid6z_mass_kg",std::to_string(ledger.totals().solid6z_mass));
}
}  // namespace coefficient_test
