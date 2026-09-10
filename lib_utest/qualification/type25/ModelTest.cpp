// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <type_traits>

namespace type25_test {
static_assert(std::is_nothrow_copy_constructible_v<spring::Model>);
static_assert(std::is_nothrow_move_constructible_v<spring::Model>);
static_assert(!std::is_copy_assignable_v<spring::Model>);
static_assert(!std::is_move_assignable_v<spring::Model>);
TEST(Type25Model,OwnsCompleteInputAndSeparateNativeEndpointCoefficients) {
  Fixture f;spring::Model model;ASSERT_TRUE(model.Initialize(f.Input()));
  EXPECT_EQ(model.source_instance_id(),17u);EXPECT_EQ(model.connection_count(),3u);
  EXPECT_EQ(model.property_count(),2u);EXPECT_EQ(model.global_node_count(),6u);
  for(unsigned i=0;i<3;++i)for(unsigned k=0;k<2;++k) {
    const auto& m=model.endpoint_mass()[2*i+k];
    EXPECT_EQ(m.source_element_id,101+i);EXPECT_EQ(m.source_node_id,201+2*i+k);EXPECT_EQ(m.global_node,2*i+k);
    EXPECT_DOUBLE_EQ(m.mass_kg,.0005);EXPECT_DOUBLE_EQ(m.isotropic_inertia_kg_m2,5e-9);
    EXPECT_EQ(m.source_property_id,41+i%2);EXPECT_TRUE(model.initial_histories()[i].active);
  }
  const auto last=model.connections()[2].position[1];f.connections[2].position[1].z=999;
  EXPECT_DOUBLE_EQ(model.connections()[2].position[1].z,last.z);
  spring::Model copy(model),moved(std::move(model));EXPECT_TRUE(copy.Matches(moved));EXPECT_TRUE(model.Matches(copy));
  EXPECT_FALSE(model.Initialize(f.Input()));EXPECT_DOUBLE_EQ(model.connections()[2].position[1].z,last.z);
}
TEST(Type25Model,CompleteIdentityUsesSourceCoordinatesPropertiesOrderAndUnits) {
  Fixture f;spring::Model base;ASSERT_TRUE(base.Initialize(f.Input()));
  spring::Model same;ASSERT_TRUE(same.Initialize(f.Input()));EXPECT_TRUE(base.Matches(same));
  for(unsigned mutation=0;mutation<6;++mutation) {
    Fixture changed;auto input=changed.Input();
    if(mutation==0)input.source_instance_id++;
    if(mutation==1)input.source_units.time_to_s=2;
    if(mutation==2)changed.properties[1].property.failure_positive[3]=std::nextafter(1e30,0);
    if(mutation==3)changed.connections[2].position[1].z=std::nextafter(changed.connections[2].position[1].z,1);
    if(mutation==4)std::swap(changed.connections[0],changed.connections[1]);
    if(mutation==5)changed.connections[0].position[0].x=-0.0;
    spring::Model other;ASSERT_TRUE(other.Initialize(input));EXPECT_FALSE(base.Matches(other))<<mutation;
  }
}
TEST(Type25Model,SharedEndpointIsRetainedAsTwoPhysicalContributions) {
  Fixture f;auto& c=f.connections[2];const auto& p=f.connections[0];
  c.source_node_id[0]=p.source_node_id[0];c.global_node[0]=p.global_node[0];c.position[0]=p.position[0];
  spring::Model model;ASSERT_TRUE(model.Initialize(f.Input()));
  EXPECT_EQ(model.endpoint_mass()[0].global_node,model.endpoint_mass()[4].global_node);
  EXPECT_DOUBLE_EQ(model.endpoint_mass()[0].mass_kg,model.endpoint_mass()[4].mass_kg);
}
TEST(Type25Model,LateInvalidInputLeavesUnpreparedAndAllowsExactRetry) {
  for(unsigned mutation=0;mutation<10;++mutation) {
    Fixture f;const auto clean=f;auto input=f.Input();
    if(mutation==0)f.connections[2].source_element_id=f.connections[0].source_element_id;
    if(mutation==1)f.connections[2].source_node_id[1]=f.connections[0].source_node_id[0];
    if(mutation==2)f.connections[2].global_node[1]=f.connections[0].global_node[0];
    if(mutation==3)f.connections[2].position[1]=f.connections[2].position[0];
    if(mutation==4)f.connections[2].position[1].z=std::numeric_limits<double>::infinity();
    if(mutation==5)f.properties[1].property.mass_kg=-1;
    if(mutation==6)f.properties[1].property.isotropic_inertia_kg_m2=std::numeric_limits<double>::denorm_min();
    if(mutation==7)f.properties[1].source_property_id=f.properties[0].source_property_id;
    if(mutation==8)f.properties[1].property.failure_negative[3]=1;
    if(mutation==9)f.connections[2].seed.y=f.connections[2].seed.x;
    spring::Model model;EXPECT_FALSE(model.Initialize(input))<<mutation;EXPECT_FALSE(model.prepared());
    EXPECT_EQ(model.connections(),nullptr);EXPECT_EQ(model.endpoint_mass(),nullptr);
    ASSERT_TRUE(model.Initialize(clean.Input()))<<mutation;
  }
}
TEST(Type25Model,CountsAndCompleteByteBudgetPrecedeBorrowedReads) {
  Fixture f;spring::Model sized;ASSERT_TRUE(sized.Initialize(f.Input()));
  for(unsigned mutation=0;mutation<4;++mutation) {
    auto input=f.Input();input.connections=reinterpret_cast<const spring::ConnectionInput*>(1);
    input.properties=reinterpret_cast<const spring::PropertyInput*>(1);
    if(mutation==0)input.connection_count=SIZE_MAX;
    if(mutation==1)input.limits.max_host_bytes=sized.startup_payload_bytes()-1;
    if(mutation==2)input.property_count=65;
    if(mutation==3)input.global_node_count=2049;
    spring::Model model;EXPECT_EQ(model.Initialize(input).status,spring::Status::ResourceLimit)<<mutation;
    EXPECT_FALSE(model.prepared());
  }
  auto input=f.Input();input.limits.max_host_bytes=sized.startup_payload_bytes();
  spring::Model exact;ASSERT_TRUE(exact.Initialize(input));EXPECT_TRUE(exact.Matches(sized));
}
} // namespace type25_test
