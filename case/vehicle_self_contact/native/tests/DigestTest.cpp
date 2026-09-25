#include "Fixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <functional>
namespace crash::cases::vehicle_self_contact::native::test {
TEST(NativeTopologyDigest, CompleteFieldCoverageIncludesSignedZerosAndExcludesAddresses) {
  Fixture f;Built built(f);const auto binding=d::InputDigest(built.input,{},1u<<20).sha256;
  const auto expected=d::TopologyDigest(built.view,binding,1u<<20);
  const Copied independent(built.view);EXPECT_EQ(d::TopologyDigest(independent.View(),binding,1u<<20).sha256,expected.sha256);
  const std::vector<std::function<void(Copied&)>> changes{
    [](auto& v){++v.source.node_count;},[](auto& v){++v.source.source_generation;},
    [](auto& v){v.source.profile=s::Profile::OrdinaryExteriorFixedMain;},[](auto& v){v.source.topology=s::TopologyPolicy::ManifoldTwoSided;},
    [](auto& v){++v.mains[0].source_id;},[](auto& v){++v.mains[0].nodes[0];},[](auto& v){++v.mains[0].global_id;},
    [](auto& v){--v.mains[0].segment_type;},[](auto& v){++v.mains[0].neighbors[0];},[](auto& v){++v.mains[0].neighbor_edges[0];},
    [](auto& v){++v.mains[0].normal_reference[0];},[](auto& v){++v.expanded[0];},[](auto& v){++v.partners[0];},
    [](auto& v){++v.offsets[0];},[](auto& v){++v.incidence[0];},
    [](auto& v){v.normals[0].x=std::copysign(0.f,-std::copysign(1.f,v.normals[0].x));},
    [](auto& v){++v.references[0].boundary;},[](auto& v){v.references[0].bisector[1].z+=.25f;}};
  for(std::size_t i=0;i<changes.size();++i) {
    SCOPED_TRACE(i);
    Copied value(built.view);changes[i](value);
    // Digest sensitivity is a value-representation test; mutated topology is
    // not being admitted or used to calculate geometry/forces.
    EXPECT_NE(d::TopologyDigest(value.View(),binding,1u<<20).sha256,expected.sha256);
  }
  EXPECT_NE(d::TopologyDigest(built.view,std::string(64,'a'),1u<<20).sha256,expected.sha256);
}
TEST(NativeTopologyDigest, ChunkBoundaryAndLastPartialWordAreCovered) {
  d::Inputs input;input.units={.001,1000,1};input.node_ids.resize(65537);input.positions.resize(3*input.node_ids.size());
  for(std::size_t i=0;i<input.node_ids.size();++i)input.node_ids[i]=i+1;
  const auto first=d::InputDigest(input,{},1u<<20);
  const auto field=std::find_if(first.fields.begin(),first.fields.end(),[](const auto& x){return x.name=="node_ids";});
  ASSERT_NE(field,first.fields.end());EXPECT_EQ(field->chunks,2u);EXPECT_EQ(field->rows,65537u);
  for(auto position:{65535u,65536u}) {
    ++input.node_ids[position];EXPECT_NE(d::InputDigest(input,{},1u<<20).sha256,first.sha256);--input.node_ids[position];
  }
  EXPECT_EQ(d::InputDigest(input,{},1u<<20).sha256,first.sha256);
  EXPECT_THROW(d::InputDigest(input,{},64),std::exception);
}
TEST(NativeTopologyDigest, InputSourceRowsUnusedNodesAndUnitContextAreBound) {
  Fixture f;auto input=f.Inputs();const auto original=d::InputDigest(input,{},1u<<20).sha256;
  input.positions.back()=.125;EXPECT_NE(d::InputDigest(input,{},1u<<20).sha256,original);
  input=f.Inputs();input.origin[0].source_line++;EXPECT_NE(d::InputDigest(input,{},1u<<20).sha256,original);
  input=f.Inputs();input.units.length_m=1.;EXPECT_NE(d::InputDigest(input,{},1u<<20).sha256,original);
  input=f.Inputs();input.positions[0]=0.;EXPECT_NE(d::InputDigest(input,{},1u<<20).sha256,original);
  input=f.Inputs();input.positions.pop_back();
  EXPECT_THROW(d::InputDigest(input,{},1u<<20),std::exception);
}
} // namespace crash::cases::vehicle_self_contact::native::test
