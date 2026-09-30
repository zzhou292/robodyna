// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "lib_src/elements/solids/resident/ResultChecks.h"
#include "lib_src/solvers/NodalForceAssembly.h"
#include <gtest/gtest.h>

namespace solid_resident_test {
namespace {
template<class Traits> void Trajectory(const typename Traits::Parent& parent,
    const typename Traits::Material& material) {
  detail::State<Traits> state;
  ASSERT_EQ(detail::InitializeState<Traits>(parent, material, {11.123, -.37, .129}, state), 0);
  EXPECT_EQ(state.history.stamp().sample_index, 0u);
  EXPECT_EQ(state.history.stamp().time_s, 0);
  ASSERT_TRUE(detail::ValidResult(parent, material, state, 0, 0));
  for (unsigned step = 0; step < 3; ++step) {
    auto interval = Traits::Phase(state.history.stamp().time_s, 1e-8, step);
    Fill<Traits>(parent, interval, 1 + .0001 * double(step + 1));
    typename Traits::Trial direct;
    ASSERT_EQ(Traits::Evaluate(parent, material, state.history, interval, direct), Traits::success);
    detail::State<Traits> next;
    ASSERT_EQ(detail::UpdateState<Traits>(parent, material, state, interval, next), 0);
    EXPECT_EQ(next.history.stamp().sample_index, step + 1);
    EXPECT_EQ(next.history.stamp().time_s, interval.base_time_s + interval.dt_s);
    ASSERT_TRUE(detail::ValidResult(parent, material, next, interval.base_time_s + interval.dt_s, step + 1));
    for (unsigned n = 0; n < Traits::nodes; ++n) {
      EXPECT_TRUE(fe::shell_startup_detail::SameVector(next.cache.rhs_force_n[n], direct.rhs_force_n[n]));
    }
    const auto saved = next;
    auto bad = interval;
    Traits::Node(bad, Traits::nodes - 1, {0, 0, std::numeric_limits<double>::quiet_NaN()}, {});
    EXPECT_NE(detail::UpdateState<Traits>(parent, material, state, bad, next), 0);
    EXPECT_EQ(next.history.stamp().sample_index, saved.history.stamp().sample_index);
    for (unsigned n = 0; n < Traits::nodes; ++n)
      EXPECT_TRUE(fe::shell_startup_detail::SameVector(next.cache.rhs_force_n[n], saved.cache.rhs_force_n[n]));
    ASSERT_EQ(detail::UpdateState<Traits>(parent, material, state, interval, next), 0);
    state = next;
  }
}
TEST(SolidResidentValues, ThreeTypedFamiliesShareOwnerPhaseWithoutSharingHistories) {
  solid_model_test::Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain, fixture.Input()));
  Trajectory<detail::Traits18>(model.solid18()[0], model.materials36()[0].value);
  Trajectory<detail::Traits24>(model.solid24()[0], model.materials42()[0].value);
  Trajectory<detail::Traits6z>(model.solid6z()[0], model.materials42()[0].value);
}
TEST(SolidResidentValues, CountsAndExactDeviceCapAreCheckedBeforeAllocation) {
  auto config = Config(9);
  detail::Counts count{1, 1, 1, 1, 1, 3};
  detail::ArenaLayout layout;
  ASSERT_TRUE(detail::MakeLayout(count, config, layout));
  EXPECT_GT(layout.bytes, 0u);
  EXPECT_GT(layout.proof.bytes, 0u);
  config.limits.max_device_bytes = layout.bytes;
  detail::ArenaLayout exact;
  ASSERT_TRUE(detail::MakeLayout(count, config, exact));
  EXPECT_EQ(exact.bytes, layout.bytes);
  --config.limits.max_device_bytes;
  exact.bytes = 73;
  EXPECT_FALSE(detail::MakeLayout(count, config, exact));
  EXPECT_EQ(exact.bytes, 73u);
  config = Config(9);
  count.solid6z = SIZE_MAX;
  EXPECT_FALSE(detail::MakeLayout(count, config, exact));
  EXPECT_EQ(exact.bytes, 73u);
  count = {0, 1, 0, 0, 1, 0};
  ASSERT_TRUE(detail::MakeLayout(count, config, layout));
  auto fake = std::make_unique<unsigned char[]>(layout.bytes);
  const auto header = detail::RebasedHeader(fake.get(), layout);
  EXPECT_EQ(header.solid18.parents, nullptr);
  EXPECT_EQ(header.solid6z.slab[0], nullptr);
  EXPECT_EQ(header.material36, nullptr);
  EXPECT_EQ(header.solid24.count, 1u);
}
TEST(SolidResidentValues, UploadRetainsCompleteMappingAndRebasesOneOwnedCurvePool) {
  solid_model_test::Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain, fixture.Input()));
  auto config = Config(domain.node_count());
  detail::ArenaLayout layout;
  ASSERT_TRUE(detail::Plan(config, model, layout));
  fixture.stress[2] = std::numeric_limits<double>::quiet_NaN();
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(layout.bytes));
  detail::Storage header;
  ASSERT_TRUE(detail::BuildUpload(config, model, arena, layout, header));
  EXPECT_EQ(header.material36[0].curve.yield_stress_pa[2], 3e6);
  for (unsigned n = 0; n < 8; ++n) {
    EXPECT_EQ(header.solid18.parents[0].domain_nodes[n], model.solid18()[0].domain_nodes[n]);
    EXPECT_EQ(header.solid24.parents[0].domain_nodes[n], model.solid24()[0].domain_nodes[n]);
    EXPECT_EQ(header.solid6z.parents[0].domain_nodes[n], model.solid6z()[0].domain_nodes[n]);
  }
  auto device = std::make_unique<unsigned char[]>(layout.bytes);
  detail::RebaseCurves(model, layout, device.get(), header);
  EXPECT_EQ(header.material36[0].curve.plastic_strain,
      tl::util::ArenaPointer<double>(device.get(), layout.curves));
  EXPECT_EQ(header.material36[0].curve.yield_stress_pa,
      tl::util::ArenaPointer<double>(device.get(), layout.curves) + 3);
  EXPECT_EQ(model.materials36()[0].value.curve.yield_stress_pa[2], 3e6);
  EXPECT_FALSE(header.solid18.slab[0][0].history.prepared());
}
TEST(SolidResidentValues, StartupRejectsLateIdentityAndRetainedBudgetBeforeUpload) {
  solid_model_test::Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain, fixture.Input()));
  auto config = Config(domain.node_count());
  detail::ArenaLayout layout;
  layout.bytes = 81;
  config.limits.max_host_bytes = model.owned_payload_bytes() - 1;
  EXPECT_EQ(detail::Plan(config, model, layout).status, s::BatchStatus::ResourceLimit);
  EXPECT_EQ(layout.bytes, 81u);
  config = Config(domain.node_count());
  config.profile = s::BatchProfile::Unspecified;
  EXPECT_EQ(detail::Plan(config, model, layout).status, s::BatchStatus::InvalidInput);
  EXPECT_EQ(layout.bytes, 81u);
  config = Config(domain.node_count());
  ASSERT_TRUE(detail::Plan(config, model, layout));
}
TEST(SolidResidentValues, FullHostForecastIncludesRetainedModelStagingAndOwnerProof) {
  solid_model_test::Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain, fixture.Input()));
  auto config = Config(domain.node_count());
  detail::ArenaLayout layout;
  ASSERT_TRUE(detail::Plan(config, model, layout));
  s::BatchForecast forecast;
  ASSERT_TRUE(s::Batch::Forecast(config, model, forecast));
  EXPECT_GT(forecast.startup_host_bytes,
      model.owned_payload_bytes() + layout.bytes + layout.staging_bytes + layout.proof.bytes);
  config.limits.max_host_bytes = forecast.startup_host_bytes;
  s::BatchForecast exact;
  ASSERT_TRUE(s::Batch::Forecast(config, model, exact));
  EXPECT_EQ(exact.startup_host_bytes, forecast.startup_host_bytes);
  --config.limits.max_host_bytes;
  exact.startup_host_bytes = 99;
  EXPECT_EQ(s::Batch::Forecast(config, model, exact).status, s::BatchStatus::ResourceLimit);
  EXPECT_EQ(exact.startup_host_bytes, 99u);
}
template<unsigned Count> void TranslationalScatter() {
  double fields[6][8]{};
  std::size_t nodes[Count];
  fe::solid18::Vec3 force[Count];
  for (unsigned n = 0; n < Count; ++n) {
    nodes[n] = n;
    force[n] = {1, 2, 3};
  }
  for (unsigned n = 0; n < 8; ++n) {
    fields[3][n] = -0.0;
    fields[4][n] = 17;
    fields[5][n] = std::numeric_limits<double>::quiet_NaN();
  }
  fe::DeviceNodalForceView view{fields[0],fields[1],fields[2],fields[3],fields[4],fields[5],8,0};
  ASSERT_EQ(fe::AccumulateNodalTranslationalForces<Count>(nodes,force,view),
      fe::NodalForceAssemblyStatus::Success);
  for (unsigned n = 0; n < Count; ++n) {
    EXPECT_EQ(fields[0][n],1);
    EXPECT_TRUE(std::signbit(fields[3][n]));
    EXPECT_EQ(fields[4][n],17);
    EXPECT_TRUE(std::isnan(fields[5][n]));
  }
  double saved[6][8];
  fields[0][Count-1] = std::numeric_limits<double>::max();
  std::memcpy(saved,fields,sizeof(fields));
  force[Count-1].x = std::numeric_limits<double>::max();
  EXPECT_EQ(fe::AccumulateNodalTranslationalForces<Count>(nodes,force,view),
      fe::NodalForceAssemblyStatus::NonfiniteResult);
  EXPECT_EQ(std::memcmp(fields,saved,sizeof(fields)),0);
  force[Count-1].x = 1;
  nodes[Count-1] = nodes[0];
  EXPECT_EQ(fe::AccumulateNodalTranslationalForces<Count>(nodes,force,view),
      fe::NodalForceAssemblyStatus::InvalidConnectivity);
  EXPECT_EQ(std::memcmp(fields,saved,sizeof(fields)),0);
  nodes[Count-1] = Count-1;
  fields[0][Count-1] = 1;
  ASSERT_EQ(fe::AccumulateNodalTranslationalForces<Count>(nodes,force,view),
      fe::NodalForceAssemblyStatus::Success);
}
TEST(SolidResidentValues, SixAndEightSlotScatterPreservesAllCouplesAndRejectsAtomically) {
  TranslationalScatter<6>();
  TranslationalScatter<8>();
}
TEST(SolidResidentValues, LegacySixChannelScatterStillRejectsLateCoupleBeforeAnyWrite) {
  double fields[6][4]{};
  const std::size_t nodes[]{2,0,3,1};
  fe::solid18::Vec3 forces[4]{{1,2,3},{4,5,6},{7,8,9},{10,11,12}};
  fe::solid18::Vec3 couples[4]{{13,14,15},{16,17,18},{19,20,21},{22,23,24}};
  fe::DeviceNodalForceView view{fields[0],fields[1],fields[2],fields[3],fields[4],fields[5],4,0};
  couples[3].z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(fe::AccumulateNodalForces<4>(nodes,forces,couples,view),
      fe::NodalForceAssemblyStatus::NonfiniteResult);
  for (const auto& channel:fields) for (double value:channel) EXPECT_EQ(value,0);
  couples[3].z=24;
  ASSERT_EQ(fe::AccumulateNodalForces<4>(nodes,forces,couples,view),
      fe::NodalForceAssemblyStatus::Success);
  EXPECT_EQ(fields[0][1],10);
  EXPECT_EQ(fields[5][1],24);
}
TEST(SolidResidentValues, PartMetadataFitsButDoesNotReplaceLiveBindingProof) {
  solid_model_test::Fixture fixture;
  const auto domain=fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain,fixture.Input()));
  auto config=Config(domain.node_count());
  config.owner.rigid_groups={777,2,4,1,888};
  detail::ArenaLayout layout;
  ASSERT_TRUE(detail::Plan(config,model,layout));
  const auto bytes=layout.bytes;
  config.owner.rigid_groups.plain_source_instance_id=0;
  EXPECT_EQ(detail::Plan(config,model,layout).status,s::BatchStatus::InvalidInput);
  EXPECT_EQ(layout.bytes,bytes);
}
TEST(SolidResidentValues, LastTypedCachedFieldAndHistoryStampRemainValidated) {
  solid_model_test::Fixture fixture;
  const auto domain=fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain,fixture.Input()));
  const auto& parent=model.solid6z()[0];
  const auto& material=model.materials42()[parent.material_index].value;
  detail::State<detail::Traits6z> state;
  ASSERT_EQ(detail::InitializeState<detail::Traits6z>(parent,material,{},state),0);
  ASSERT_TRUE(detail::ValidResult(parent,material,state,0,0));
  const auto saved=state;
  state.cache.stabilization.modal_force_n[2][3]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(detail::ValidResult(parent,material,state,0,0));
  state=saved;
  EXPECT_FALSE(detail::ValidResult(parent,material,state,0,1));
  state.cache.material.point.stress_pa[5]=1;
  EXPECT_FALSE(detail::ValidResult(parent,material,state,0,0));
  state=saved;
  EXPECT_TRUE(detail::ValidResult(parent,material,state,0,0));
}
} // namespace
} // namespace solid_resident_test
