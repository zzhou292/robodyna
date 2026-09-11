// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "../solid18_reference/SourceFixture.h"
#include "../solid24_reference/SourceFixture.h"
#include "../solid24_reference/JacobianSupport.h"
#include "../solid6z_reference/SourceFixture.h"
#include "../solid_law36_point/source_fixture/OriginalMaterial.h"
#include <gtest/gtest.h>
#include <map>

namespace solid_model_test {
template<class Values> void SameValues(const Values& a,const Values& b) {
  ASSERT_EQ(a.size(),b.size());
  for(std::size_t i=0;i<a.size();++i)EXPECT_TRUE(Bits(a[i],b[i]))<<i;
}
template<class Parent,class Input> void SameSource(const Parent& parent,const Input& input,
                                                 unsigned arity,const fe::NodalNodeDomain& domain) {
  const auto& a=parent.reference.input();const auto& b=input.reference.input();
  EXPECT_EQ(a.source_element_id,b.source_element_id);EXPECT_EQ(a.source_part_id,b.source_part_id);
  EXPECT_EQ(a.source_section_id,b.source_section_id);EXPECT_EQ(a.source_material_id,b.source_material_id);
  for(unsigned n=0;n<arity;++n) {
    EXPECT_EQ(a.source_node_id[n],b.source_node_id[n]);
    EXPECT_EQ(parent.domain_nodes[n],domain.Find(b.source_node_id[n]));
    EXPECT_EQ(parent.reference.source_slot(n),input.reference.source_slot(n));
    EXPECT_TRUE(Bits(a.position_m[n].x,b.position_m[n].x));
    EXPECT_TRUE(Bits(a.position_m[n].y,b.position_m[n].y));
    EXPECT_TRUE(Bits(a.position_m[n].z,b.position_m[n].z));
  }
}
TEST(SolidModelSource, All2412OriginalReferencesMaterialsAndDomainSlotsAreRetained) {
  std::vector<s::Input18> a;
  std::vector<s::Input24> b;
  std::vector<s::Input6z> c;
  std::map<std::uint64_t,tl::math::Vec3> unique;
  const auto collect=[&](const auto& input,unsigned arity) {
    for(unsigned n=0;n<arity;++n) {
      const auto added=unique.emplace(input.source_node_id[n],input.position_m[n]);
      if(!added.second) {
        const auto x=added.first->second,y=input.position_m[n];
        Require(Bits(x.x,y.x) && Bits(x.y,y.y) && Bits(x.z,y.z));
      }
    }
  };
  for(unsigned i=0;i<solid18_test::SourceCount;++i) {
    const auto input=solid18_test::Source(i);collect(input,8);
    s::Input18 parent;
    ASSERT_EQ(fe::solid18::InitializeReference(input,parent.reference),fe::solid18::Status::Success);
    ASSERT_EQ(tl::material::law36::Prepare(law36_test::E,law36_test::Nu,input.density_kg_m3,
      law36_test::Curve,parent.material),tl::material::law36::Status::Ok);
    a.push_back(parent);
  }
  for(unsigned i=0;i<solid24_test::SourceCount;++i) {
    auto input=solid24_test::Source(i);
    fe::solid24::Material material;
    ASSERT_EQ(tl::material::law42::Prepare(24e6,.463,input.density_kg_m3,1e26,material),tl::material::law42::Status::Ok);
    if(solid24_test::IsBrick(input)) {
      collect(input,8);
      input.profile.reference_strain=fe::solid24::ReferenceStrain::TotalLagrangian10;
      input.profile.working_length=fe::solid24::WorkingLengthUnit::Millimetre;
      s::Input24 parent;parent.material=material;
      ASSERT_EQ(fe::solid24::InitializeReference(input,parent.reference),fe::solid24::Status::Success);
      b.push_back(parent);
    } else {
      fe::solid6z::ReferenceInput wedge;
      ASSERT_TRUE(solid6z_test::Source(i,wedge));collect(wedge,6);
      s::Input6z parent;parent.material=material;
      ASSERT_EQ(fe::solid6z::InitializeReference(wedge,parent.reference),fe::solid6z::Status::Success);
      c.push_back(parent);
    }
  }
  ASSERT_EQ(a.size(),908u);ASSERT_EQ(b.size(),1309u);ASSERT_EQ(c.size(),195u);
  std::vector<fe::NodalDomainNode> nodes;
  for(auto it=unique.rbegin();it!=unique.rend();++it)nodes.push_back({it->first,it->second});
  fe::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({777,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle()));
  s::ModelInput input{777,{a.data(),a.size()},{b.data(),b.size()},{c.data(),c.size()}};
  s::Model model;
  const auto report=model.Initialize(domain,input);
  ASSERT_TRUE(report)<<report.message<<" family "<<int(report.family)<<" parent "<<report.parent;
  ASSERT_EQ(model.contributions()->parents().size(),2412u);
  EXPECT_EQ(model.materials36().size(),1u);EXPECT_EQ(model.materials42().size(),8u);
  for(std::size_t i=0;i<a.size();++i) {
    SCOPED_TRACE(a[i].reference.input().source_element_id);
    SameSource(model.solid18()[i],a[i],8,domain);
    SameValues(solid18_test::Values(model.solid18()[i].reference),solid18_test::Values(a[i].reference));
    const auto curve=model.materials36()[model.solid18()[i].material_index].value.curve;
    ASSERT_EQ(curve.count,law36_test::Curve.count);
    for(unsigned k=0;k<curve.count;++k) {
      EXPECT_TRUE(Bits(curve.plastic_strain[k],law36_test::X[k]));
      EXPECT_TRUE(Bits(curve.yield_stress_pa[k],law36_test::Y[k]));
    }
  }
  for(std::size_t i=0;i<b.size();++i) {
    SCOPED_TRACE(b[i].reference.input().source_element_id);
    SameSource(model.solid24()[i],b[i],8,domain);
    SameValues(solid24_test::Values(model.solid24()[i].reference),solid24_test::Values(b[i].reference));
    SameValues(solid24_test::JacobianValues(model.solid24()[i].reference),solid24_test::JacobianValues(b[i].reference));
  }
  for(std::size_t i=0;i<c.size();++i) {
    SCOPED_TRACE(c[i].reference.input().source_element_id);
    SameSource(model.solid6z()[i],c[i],6,domain);
    SameValues(solid6z_test::Values(model.solid6z()[i].reference),solid6z_test::Values(c[i].reference));
    EXPECT_EQ(model.solid6z()[i].profile.stabilization,c[i].profile.stabilization);
  }
  RecordProperty("solid18_parents",a.size());RecordProperty("solid24_parents",b.size());
  RecordProperty("solid6z_parents",c.size());RecordProperty("domain_nodes",nodes.size());
  RecordProperty("retained_payload_bytes",model.owned_payload_bytes());
  RecordProperty("startup_payload_bytes",model.startup_payload_bytes());
}
} // namespace solid_model_test
