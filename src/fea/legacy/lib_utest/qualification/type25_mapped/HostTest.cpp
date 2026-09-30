// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <limits>

namespace type25_mapped_test {
TEST(Type25MappedHost, CompletePhysicalForecastExactCapsAndLatePolicyFailureRetry) {
  Fixture f;
  auto config=f.Config();
  mapped::Forecast forecast;
  ASSERT_EQ(mapped::MakeForecast(config,f.physical,f.Witnesses(),spring::CapacityProfile::Legacy,
      128,forecast).status,spring::BatchStatus::Success);
  auto tight=config;
  tight.max_host_bytes=forecast.host_bytes;
  tight.max_device_bytes=forecast.device.bytes;
  mapped::Forecast sentinel;
  sentinel.host_bytes=73;
  --tight.max_host_bytes;
  EXPECT_EQ(mapped::MakeForecast(tight,f.physical,f.Witnesses(),spring::CapacityProfile::Legacy,
      128,sentinel).status,spring::BatchStatus::ResourceLimit);
  EXPECT_EQ(sentinel.host_bytes,73u);
  ++tight.max_host_bytes;
  --tight.max_device_bytes;
  EXPECT_EQ(mapped::MakeForecast(tight,f.physical,f.Witnesses(),spring::CapacityProfile::Legacy,
      128,sentinel).status,spring::BatchStatus::ResourceLimit);
  ++tight.max_device_bytes;
  ASSERT_EQ(mapped::MakeForecast(tight,f.physical,f.Witnesses(),spring::CapacityProfile::Legacy,
      128,sentinel).status,spring::BatchStatus::Success);
  EXPECT_EQ(sentinel.host_bytes,forecast.host_bytes);
  Fixture unsupported(false,true);
  const auto rejected=mapped::ValidateModel(unsupported.model);
  EXPECT_EQ(rejected.status,spring::BatchStatus::InvalidInput);
  EXPECT_EQ(rejected.element,unsupported.model.connection_count()-1);
  EXPECT_EQ(mapped::ValidateModel(f.model).status,spring::BatchStatus::Success);
  config.max_nodes=spring::LegacyCapacity.nodes+1;
  EXPECT_EQ(mapped::MakeForecast(config,{}, {},spring::CapacityProfile::Legacy,128,sentinel).status,
      spring::BatchStatus::ResourceLimit);
}
TEST(Type25MappedHost, RetainedCaseCountDeviceLayoutKeepsExplicitVehicleAndCompleteSlabs) {
  batch::ArenaLayout result;
  result.bytes=59;
  EXPECT_FALSE(batch::MakeLayout(1,2828,372435,32u<<20,result));
  EXPECT_EQ(result.bytes,59u);
  ASSERT_TRUE(batch::MakeLayout(1,2828,372435,32u<<20,result,spring::CapacityProfile::Vehicle));
  EXPECT_EQ(result.slab[0].count,2828u);
  EXPECT_EQ(result.slab[1].count,2828u);
  EXPECT_EQ(result.nodes.count,372435u);
  RecordProperty("type25_payload_bytes",std::to_string(result.bytes));
  auto sentinel=result;
  EXPECT_FALSE(batch::MakeLayout(1,2828,372435,result.bytes-1,sentinel,spring::CapacityProfile::Vehicle));
  EXPECT_EQ(sentinel.bytes,result.bytes);
}
TEST(Type25MappedHost, InitialPacketRetainsExactPhysicalZeroInertiaAndNativeVirginCache) {
  Fixture f;
  const auto config=f.Config();
  mapped::Forecast forecast;
  ASSERT_EQ(mapped::MakeForecast(config,f.physical,f.Witnesses(),spring::CapacityProfile::Legacy,
      128,forecast).status,spring::BatchStatus::Success);
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(forecast.device.bytes));
  batch::Storage header;
  spring::BatchDiagnostics diagnostics;
  ASSERT_EQ(mapped::BuildModel(config,f.physical,arena,forecast.device,header,diagnostics).status,
      spring::BatchStatus::Success);
  const auto solid_only=f.base.mechanics.domain.Find(9305);
  EXPECT_EQ(header.model.nodes[solid_only].inertia,0);
  EXPECT_GT(header.model.nodes[solid_only].mass,0);
  EXPECT_FALSE(diagnostics.has_completed_interval);
  EXPECT_FALSE(diagnostics.accepted_force_assembled);
  for (std::size_t e=0;e<config.element_count;++e) {
    const auto& result=header.slab[0].element[e];
    Exact(result,header.slab[1].element[e]);
    EXPECT_TRUE(result.history.active);
    EXPECT_EQ(result.history.failure_criterion,0);
    for (const auto& rhs:result.endpoints) {
      EXPECT_EQ(tl::math::fixed3::Norm(rhs.force_N),0);
      EXPECT_EQ(tl::math::fixed3::Norm(rhs.couple_Nm),0);
    }
    for (unsigned slot=0;slot<2;++slot) EXPECT_EQ(header.model.elements[e].nodes[slot],
        f.model.connections()[e].global_node[slot]);
  }
}
TEST(Type25MappedHost, NewOffZerosStiffnessWhileFailedIntervalKeepsForceAndNextClearsIt) {
  auto p=type25_test::Property();
  for (auto& k:p.stiffness) k=1000;
  for (auto& threshold:p.failure_positive) threshold=1e-8;
  const spring::Vec3 x[2]{{0,0,0},{.01,0,0}};
  spring::Reference reference;
  ASSERT_EQ(spring::InitializeReference({1,1,1},x,{},reference),spring::Status::Success);
  spring::History history;
  history.transverse_axis=reference.transverse_axis;
  spring::EndpointKinematics nodes[2]{{x[0],{},{}},{{.011,0,0},{1,0,0},{}}};
  spring::Evaluation failed;
  ASSERT_EQ(spring::Evaluate({1,1,1},p,reference,history,nodes,.001,failed),spring::Status::Success);
  ASSERT_FALSE(failed.history.active);
  ASSERT_GT(tl::math::fixed3::Norm(failed.endpoints[0].force_N),0);
  mapped::NodalStiffness stiffness{17,19};
  ASSERT_TRUE(mapped::AcceptedStiffness(p,failed,stiffness));
  EXPECT_EQ(stiffness.translation,0);
  EXPECT_EQ(stiffness.rotation,0);
  spring::Evaluation inactive;
  ASSERT_EQ(spring::Evaluate({1,1,1},p,reference,failed.history,nodes,.001,inactive),spring::Status::Success);
  EXPECT_EQ(tl::math::fixed3::Norm(inactive.endpoints[0].force_N),0);
  p.damping[3]=1;
  stiffness={17,19};
  EXPECT_FALSE(mapped::AcceptedStiffness(p,failed,stiffness));
  EXPECT_EQ(stiffness.translation,17);
  p.damping[3]=0;
  failed.translation_stiffness_N_per_m=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(mapped::AcceptedStiffness(p,failed,stiffness));
  EXPECT_EQ(stiffness.translation,17);
}
TEST(Type25MappedHost, OrderedNodalAdditionsAreAtomicAndDoNotUseTheDtFloor) {
  double translation[3]{1e16,1,0},rotation[3]{};
  const std::size_t first[2]{2,0},second[2]{0,1};
  ASSERT_TRUE(mapped::AddStiffness(first,{1,2},translation,rotation,3));
  ASSERT_TRUE(mapped::AddStiffness(second,{3,4},translation,rotation,3));
  EXPECT_EQ(translation[0],(1e16+1)+3);
  EXPECT_EQ(translation[2],1);
  EXPECT_EQ(rotation[0],6);
  const auto before=std::array<double,3>{translation[0],translation[1],translation[2]};
  rotation[1]=std::numeric_limits<double>::max();
  EXPECT_FALSE(mapped::AddStiffness(second,{5,std::numeric_limits<double>::max()},translation,rotation,3));
  for (unsigned n=0;n<3;++n) EXPECT_EQ(Bits(translation[n]),Bits(before[n]));
  auto p=type25_test::Property();
  for (auto& k:p.stiffness) k=1e-20;
  spring::Stability coefficient;
  ASSERT_EQ(spring::CriticalStep({1,1,1},p,.25,coefficient),spring::Status::Success);
  spring::Evaluation value;
  value.translation_stiffness_N_per_m=coefficient.translation_stiffness_N_per_m;
  value.rotation_stiffness_Nm_per_rad=coefficient.rotation_stiffness_Nm_per_rad;
  mapped::NodalStiffness result;
  ASSERT_TRUE(mapped::AcceptedStiffness(p,value,result));
  EXPECT_EQ(result.translation,1e-20);
  EXPECT_LT(result.rotation,1e-15);
}
} // namespace type25_mapped_test
