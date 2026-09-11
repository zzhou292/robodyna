// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OriginalFixture.h"
#include <gtest/gtest.h>

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
  OriginalFixture fixture;
  const auto& a=fixture.a;
  const auto& b=fixture.b;
  const auto& c=fixture.c;
  const auto& nodes=fixture.nodes;
  const auto& domain=fixture.domain;
  const auto& model=fixture.model;
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
