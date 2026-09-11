// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/solid_model/OriginalFixture.h"
#include "lib_utest/qualification/solid18_law44_reference/TestSupport.h"
#include "lib_utest/qualification/solid_law44_point/TestSupport.h"
#include "lib_utest/qualification/law90_solid18_reference/SourceFixture.h"
#include "lib_utest/qualification/law90_preparation/TestSupport.h"
#include "RearOriginalFixture.h"
#include <gtest/gtest.h>
namespace extended_model_source_test {
namespace fe = tl::fea;
namespace s = fe::solids;
using solid_model_test::Bits;
using solid_model_test::Require;
template<class Reference> void Collect(const Reference& reference,
    std::map<std::uint64_t,tl::math::Vec3>& unique) {
  const auto& source = reference.input();
  for (unsigned slot = 0; slot < 8; ++slot) {
    const auto x = source.position_m[slot];
    const auto added = unique.emplace(source.source_node_id[slot],x);
    if (!added.second) {
      const auto y = added.first->second;
      Require(Bits(x.x,y.x) && Bits(x.y,y.y) && Bits(x.z,y.z));
    }
  }
}
template<class Parent, class Reference> void Mapping(const Parent& parent,
    const Reference& expected, const fe::NodalNodeDomain& domain,
    const fe::SolidCoefficientParent& mass) {
  const auto& a = parent.reference.input(); const auto& b = expected.input();
  EXPECT_EQ(a.source_element_id,b.source_element_id);
  EXPECT_EQ(a.source_part_id,b.source_part_id);
  EXPECT_EQ(a.source_section_id,b.source_section_id);
  EXPECT_EQ(a.source_material_id,b.source_material_id);
  for (unsigned n = 0; n < 8; ++n) {
    EXPECT_EQ(a.source_node_id[n],b.source_node_id[n]);
    EXPECT_EQ(parent.domain_nodes[n],domain.Find(b.source_node_id[n]));
    EXPECT_TRUE(Bits(a.position_m[n].x,b.position_m[n].x));
    EXPECT_TRUE(Bits(a.position_m[n].y,b.position_m[n].y));
    EXPECT_TRUE(Bits(a.position_m[n].z,b.position_m[n].z));
    EXPECT_EQ(mass.domain_node[n],parent.domain_nodes[n]);
    EXPECT_TRUE(Bits(mass.mass_kg[n],expected.mass().source_nodal_mass_kg[n]));
  }
  EXPECT_TRUE(Bits(mass.isotropic_inertia_kg_m2(),0.0));
}
TEST(ExtendedSolidModelSource, All4063OriginalParentsRetainTypedMaterialsAndExactSourceSlots) {
  solid_model_test::OriginalFixture original;
  std::map<std::uint64_t,tl::math::Vec3> unique;
  for (const auto& node : original.nodes) unique.emplace(node.source_id,node.position);
  std::vector<s::Input18Law44> rear;
  std::vector<s::Input18Law90> foam;
  for (unsigned i = 0; i < std::size(rear18_test::original::Cells); ++i) {
    const auto input = rear18_test::original::Input(i);
    s::Input18Law44 parent;
    Require(fe::solid18::law44::InitializeReference(input,parent.reference)==fe::solid18::Status::Success);
    const bool bar = input.source_part_id == 2000016;
    Require(bar || input.source_part_id == 2000392);
    parent.material = law44_solid_test::Parameters(bar);
    // Authenticated MAT024 cards: PID16 line478 E50000; PID392 line7895 E200000 MPa.
    Require(Bits(parent.material.material.young_pa, (bar ? 50000.0 : 200000.0)*1e6));
    Collect(parent.reference,unique); rear.push_back(parent);
  }
  tl::material::law90::PreparedMaterial material90;
  Require(tl::material::law90::PrepareSI(law90_test::OriginalInput(),law90_test::OriginalCurve(),material90)==tl::material::law90::Status::Ok);
  for (unsigned i = 0; i < std::size(radiator_source_fixture::solid_records_u64); ++i) {
    s::Input18Law90 parent;
    Require(fe::solid18::total_strain::InitializeReference90(law90_reference_test::Original(i),parent.reference)==fe::solid18::Status::Success);
    parent.material = material90;
    Collect(parent.reference,unique); foam.push_back(parent);
  }
  ASSERT_EQ(rear.size(),306u); ASSERT_EQ(foam.size(),1345u);
  std::vector<fe::NodalDomainNode> nodes;
  for (auto it = unique.rbegin(); it != unique.rend(); ++it) nodes.push_back({it->first,it->second});
  fe::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({777,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle()));
  s::ModelInput input{777,{original.a.data(),original.a.size()},
      {original.b.data(),original.b.size()},{original.c.data(),original.c.size()},
      {rear.data(),rear.size()},{foam.data(),foam.size()},s::ModelProfile::ExtendedLaw44Law90};
  s::Model model;
  const auto report = model.Initialize(domain,input);
  ASSERT_TRUE(report) << report.message << " family " << unsigned(report.family) << " row " << report.parent;
  ASSERT_EQ(model.contributions()->parents().size(),4063u);
  ASSERT_EQ(model.materials44().size(),2u); ASSERT_EQ(model.materials90().size(),1u);
  EXPECT_NE(model.materials90()[0].value.curve().compression_strain,material90.curve().compression_strain);
  unsigned repeated = 0;
  for (unsigned i = 0; i < rear.size(); ++i) {
    const auto& parent = model.solid18_law44()[i]; SCOPED_TRACE(parent.reference.input().source_element_id);
    Mapping(parent,rear[i].reference,domain,model.contributions()->parents()[2412+i]);
    EXPECT_EQ(rear18_test::Values(parent.reference),rear18_test::Values(rear[i].reference));
    const auto& material = model.materials44()[parent.material_index].value;
    EXPECT_TRUE(Bits(material.material.young_pa,rear[i].material.material.young_pa));
    EXPECT_EQ(material.material.native_units,tl::material::law44::solid::WorkingUnits::TonneMillimetreSecond);
    EXPECT_NE(material.curve.plastic_strain,rear[i].material.curve.plastic_strain);
    for (unsigned n = 0; n < material.curve.count; ++n) {
      EXPECT_TRUE(Bits(material.curve.plastic_strain[n],rear[i].material.curve.plastic_strain[n]));
      EXPECT_TRUE(Bits(material.curve.yield_stress_pa[n],rear[i].material.curve.yield_stress_pa[n]));
    }
    repeated += parent.reference.topology()==fe::solid18::law44::SourceTopology::RepeatedPairs56And78;
  }
  for (unsigned i = 0; i < foam.size(); ++i) {
    const auto& parent = model.solid18_law90()[i]; SCOPED_TRACE(parent.reference.input().source_element_id);
    Mapping(parent,foam[i].reference,domain,model.contributions()->parents()[2718+i]);
    EXPECT_EQ(law90_reference_test::ReferenceValues(parent.reference),law90_reference_test::ReferenceValues(foam[i].reference));
    EXPECT_EQ(parent.material_index,0u);
  }
  double expected[33],actual[33];
  law90_test::Pack(material90,expected); law90_test::Pack(model.materials90()[0].value,actual);
  for (unsigned i = 0; i < 33; ++i) EXPECT_TRUE(Bits(expected[i],actual[i]));
  for (unsigned i = 0; i < material90.curve().count; ++i) {
    EXPECT_TRUE(Bits(model.materials90()[0].value.curve().compression_strain[i],material90.curve().compression_strain[i]));
    EXPECT_TRUE(Bits(model.materials90()[0].value.curve().stress_pa[i],material90.curve().stress_pa[i]));
  }
  EXPECT_EQ(repeated,109u);
  RecordProperty("parents",4063); RecordProperty("rear",rear.size()); RecordProperty("foam",foam.size());
  RecordProperty("repeated_rear",repeated); RecordProperty("domain_nodes",nodes.size());
  RecordProperty("owned_payload_bytes",model.owned_payload_bytes());
  RecordProperty("startup_payload_bytes",model.startup_payload_bytes());
}
} // namespace extended_model_source_test
