#include "lib_utest/qualification/type25_batch/VehicleMassInput.h"
#include <cstring>

namespace {
namespace fe=tl::fea;namespace spring=fe::type25;
TEST(NodalMassVehicle,CompleteSourceSizedNodeCoverageRetainsFinalEndpointAndSourceOrderedTotals) {
  type25_batch_test::VehicleMassInput fixture(359785,2828);ASSERT_TRUE(fixture.Initialize());
  EXPECT_EQ(fixture.mass.node_count(),359785u);EXPECT_EQ(fixture.model.connection_count(),2828u);
  const auto last=fixture.mass.nodes()[359784];
  EXPECT_EQ(last.source_id,2359784u);
  EXPECT_DOUBLE_EQ(last.coefficients.connector_mass,.5*fixture.source.properties[1].property.mass_kg);
  double mass=0,inertia=0;
  for(std::size_t e=0;e<fixture.model.connection_count();++e)for(unsigned a=0;a<2;++a) {
    mass+=fixture.model.endpoint_mass()[2*e+a].mass_kg;inertia+=fixture.model.endpoint_mass()[2*e+a].isotropic_inertia_kg_m2;
  }
  EXPECT_DOUBLE_EQ(fixture.mass.totals().connector_mass,mass);EXPECT_DOUBLE_EQ(fixture.mass.totals().connector_inertia,inertia);
  for(std::size_t n=0;n<fixture.mass.node_count();++n) {
    const auto& actual=fixture.mass.nodes()[n];const auto& reference=fixture.shells.nodes()[n];
    EXPECT_EQ(actual.source_id,reference.source_id);EXPECT_EQ(std::memcmp(&actual.coefficients.shell,&reference.native,sizeof(reference.native)),0);
  }
  fe::NodalMassBinding legacy;EXPECT_EQ(legacy.Initialize(fixture.shells,fixture.model).status,fe::NodalMassStatus::ResourceLimit);
  auto limits=fe::NodalMassLimits::Vehicle();limits.max_host_bytes=fixture.mass.host_bytes()-1;
  fe::NodalMassBinding retry;EXPECT_EQ(retry.Initialize(fixture.shells,fixture.model,limits).status,fe::NodalMassStatus::ResourceLimit);
  EXPECT_FALSE(retry.prepared());++limits.max_host_bytes;ASSERT_TRUE(retry.Initialize(fixture.shells,fixture.model,limits));
  EXPECT_TRUE(retry.Matches(fixture.mass));RecordProperty("combined_payload_bytes",std::to_string(fixture.mass.host_bytes()));
}
TEST(NodalMassVehicle,SharedBackingPredicateNeverConfusesIndependentEquivalentModels) {
  type25_batch_test::VehicleMassInput f(2052,1025);ASSERT_TRUE(f.Initialize());
  spring::Model copy(f.model),equal;ASSERT_TRUE(equal.Initialize(f.source.Input()));
  EXPECT_TRUE(f.mass.SharesConnectorStorage(copy));EXPECT_FALSE(f.mass.SharesConnectorStorage(equal));EXPECT_TRUE(f.mass.Matches(equal));
  fe::NodalMassBinding separately;ASSERT_TRUE(separately.Initialize(f.shells,equal,fe::NodalMassLimits::Vehicle()));
  EXPECT_TRUE(separately.Matches(f.mass));EXPECT_TRUE(separately.SharesConnectorStorage(equal));EXPECT_FALSE(separately.SharesConnectorStorage(copy));
  const auto saved=f.source.connections.back().position[1];f.source.connections.back().position[1].x=std::nextafter(saved.x,INFINITY);
  spring::Model wrong;ASSERT_TRUE(wrong.Initialize(f.source.Input()));fe::NodalMassBinding rejected;
  EXPECT_EQ(rejected.Initialize(f.shells,wrong,fe::NodalMassLimits::Vehicle()).status,fe::NodalMassStatus::PositionMismatch);
  EXPECT_FALSE(rejected.prepared());ASSERT_TRUE(rejected.Initialize(f.shells,f.model,fe::NodalMassLimits::Vehicle()));
}
} // namespace
