#include "VehicleModelFixture.h"
#include <cstring>
#include <limits>
#include <gtest/gtest.h>

namespace type25_test {
TEST(Type25VehicleModel,SourceCountsAndMaximumTailRetainNativeCoefficientsAndOwnedIdentity) {
  for(const auto sizes:std::array<std::array<std::size_t,2>,2>{{{2828,359785},{4096,524288}}}) {
    VehicleModelFixture f(sizes[0],sizes[1]);spring::Model model;const auto in=f.Input();
    ASSERT_TRUE(model.Initialize(in));EXPECT_EQ(model.connection_count(),sizes[0]);EXPECT_EQ(model.global_node_count(),sizes[1]);
    const auto last=2*sizes[0]-1;
    EXPECT_EQ(model.endpoint_mass()[last].global_node,sizes[1]-1);
    EXPECT_EQ(model.endpoint_mass()[last].source_node_id,2000000+sizes[1]-1);
    EXPECT_DOUBLE_EQ(model.endpoint_mass()[last].mass_kg,.5*f.properties[(sizes[0]-1)%2].property.mass_kg);
    spring::Model copy(model),equal;ASSERT_TRUE(equal.Initialize(in));
    EXPECT_TRUE(model.SharesStorage(copy));EXPECT_FALSE(model.SharesStorage(equal));EXPECT_TRUE(model.Matches(equal));
    f.connections.back().source_element_id=0;EXPECT_NE(model.connections()[sizes[0]-1].source_element_id,0u);
    EXPECT_GT(model.startup_payload_bytes(),model.owned_payload_bytes());
    RecordProperty("last_model_startup_bytes",std::to_string(model.startup_payload_bytes()));
  }
}
TEST(Type25VehicleModel,IndexedPriorFailureOrderMatchesLegacyForCompetingDefects) {
  for(unsigned fault=0;fault<5;++fault) {
    Fixture f;
    if(fault==0) {f.connections[2].source_element_id=f.connections[1].source_element_id;f.connections[2].source_node_id[0]=f.connections[0].source_node_id[0];}
    if(fault==1) {f.connections[2].source_element_id=f.connections[0].source_element_id;f.connections[2].global_node[1]=f.connections[1].global_node[1];}
    if(fault==2) {f.connections[2].global_node[1]=f.connections[0].global_node[1];f.connections[2].source_node_id[1]=f.connections[0].source_node_id[1];}
    if(fault==3)f.connections[2].position[1].z=std::numeric_limits<double>::infinity();
    if(fault==4)f.properties[1].source_property_id=f.properties[0].source_property_id;
    auto in=f.Input();spring::Model old,vehicle;const auto expected=old.Initialize(in);
    in.limits=spring::ModelLimits::Vehicle();const auto actual=vehicle.Initialize(in);
    EXPECT_EQ(actual.status,expected.status);EXPECT_EQ(actual.connection,expected.connection);EXPECT_STREQ(actual.message,expected.message);
    EXPECT_FALSE(vehicle.prepared());Fixture clean;in=clean.Input();in.limits=spring::ModelLimits::Vehicle();ASSERT_TRUE(vehicle.Initialize(in));
  }
  Fixture f;auto in=f.Input();spring::Model old,vehicle;ASSERT_TRUE(old.Initialize(in));
  in.limits=spring::ModelLimits::Vehicle();ASSERT_TRUE(vehicle.Initialize(in));EXPECT_TRUE(old.Matches(vehicle));
  for(std::size_t e=0;e<6;++e) {
    EXPECT_DOUBLE_EQ(old.endpoint_mass()[e].mass_kg,vehicle.endpoint_mass()[e].mass_kg);
    EXPECT_DOUBLE_EQ(old.endpoint_mass()[e].isotropic_inertia_kg_m2,vehicle.endpoint_mass()[e].isotropic_inertia_kg_m2);
  }
}
TEST(Type25VehicleModel,ExplicitProfileCapsAndLateSharedEndpointFailurePreserveRetry) {
  VehicleModelFixture f;auto in=f.Input();spring::Model measured;ASSERT_TRUE(measured.Initialize(in));
  for(unsigned fault=0;fault<6;++fault) {
    spring::Model model;auto bad=in;
    if(fault==0)bad.limits.profile=spring::CapacityProfile::Legacy;
    if(fault==1)bad.connection_count=4097;
    if(fault==2)bad.global_node_count=524289;
    if(fault==3)bad.limits.max_host_bytes=measured.startup_payload_bytes()-1;
    if(fault==4)bad.connections=reinterpret_cast<const spring::ConnectionInput*>(UINTPTR_MAX-4);
    if(fault==5)bad.limits.profile=static_cast<spring::CapacityProfile>(99);
    EXPECT_FALSE(model.Initialize(bad));EXPECT_FALSE(model.prepared());
    auto exact=in;exact.limits.max_host_bytes=measured.startup_payload_bytes();ASSERT_TRUE(model.Initialize(exact));
  }
  auto& last=f.connections.back();last.global_node[0]=f.connections[0].global_node[0];
  last.source_node_id[0]=f.connections[0].source_node_id[0];last.position[0]=f.connections[0].position[0];
  spring::Model shared;ASSERT_TRUE(shared.Initialize(in)); // repeated producer contribution remains separate
  EXPECT_EQ(shared.endpoint_mass()[0].global_node,shared.endpoint_mass()[2*f.connections.size()-2].global_node);
  last.position[0].x=-0.;spring::Model invalid;
  const auto rejected=invalid.Initialize(in);EXPECT_EQ(rejected.status,spring::Status::DuplicateIdentity);
  EXPECT_EQ(rejected.connection,f.connections.size()-1);EXPECT_FALSE(invalid.prepared());
}
} // namespace type25_test
