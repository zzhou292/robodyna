// SPDX-License-Identifier: AGPL-3.0-or-later
// Reuse the complete existing mixed PART/plain/ordinary/CIN owner rejection
// and retry witnesses. Their actual constructor now selects the gather arena.
#include "../cin_force_transfers/OwnerTest.cu"
#include "lib_src/solvers/cin_advance/ForceGatherLayout.h"

namespace tl::fea::cin_parallel_test {
TEST(CinMasterGatherOwner,TightCapSerialAndExactFitGatherKeepIdenticalNativeStateAndCapture) {
  old::Fixture fixture;
  fixture.DependentInverses(true);
  auto config=fixture.Config();config.capture_force_stage_accelerations=true;
  const auto cin=fixture.Cin();
  const auto forecast=FENodalState::ForecastAssemblyCin(config,fixture.binding,cin,true);
  ASSERT_EQ(forecast.report.status,NodalStatus::Ok)<<forecast.report.message;
  cin_advance::force_gather::Layout gathered;
  ASSERT_TRUE(gathered.Initialize(0,fixture.m.size(),cin.model->rows().count,128u<<20,128u<<20));
  ASSERT_GT(forecast.device_bytes,gathered.device_bytes);
  auto tight=config;tight.max_device_bytes=forecast.device_bytes-gathered.device_bytes;
  const auto serial_forecast=FENodalState::ForecastAssemblyCin(tight,fixture.binding,cin,true);
  ASSERT_EQ(serial_forecast.report.status,NodalStatus::Ok);
  EXPECT_EQ(serial_forecast.device_bytes,tight.max_device_bytes);
  EXPECT_EQ(forecast.device_bytes,serial_forecast.device_bytes+gathered.device_bytes);
  EXPECT_EQ(forecast.source_host_bytes,serial_forecast.source_host_bytes);
  EXPECT_EQ(forecast.owner_host_bytes,
      serial_forecast.owner_host_bytes+gathered.host_bytes);
  EXPECT_EQ(forecast.startup_scratch_bytes,
      serial_forecast.startup_scratch_bytes+gathered.temporary_bytes);
  EXPECT_EQ(forecast.startup_host_bytes,
      serial_forecast.startup_host_bytes+gathered.host_bytes+gathered.temporary_bytes);
  EXPECT_EQ(forecast.startup_host_bytes,
      forecast.source_host_bytes+forecast.owner_host_bytes+forecast.startup_scratch_bytes);
  config.max_device_bytes=forecast.device_bytes;
  FENodalState serial,parallel;
  ASSERT_EQ(serial.Initialize(tight,fixture.Kinematics(),fixture.im.data(),fixture.Dofs(),fixture.binding,&cin).status,NodalStatus::Ok);
  ASSERT_EQ(parallel.Initialize(config,fixture.Kinematics(),fixture.im.data(),fixture.Dofs(),fixture.binding,&cin).status,NodalStatus::Ok);
  EXPECT_EQ(serial.allocations().device_bytes,serial_forecast.device_bytes);
  EXPECT_EQ(parallel.allocations().device_bytes,forecast.device_bytes);
  EXPECT_EQ(serial.allocations().device_allocations,parallel.allocations().device_allocations);
  const auto n=fixture.m.size(),r=cin.model->rows().count;
  fields::Snapshot a(n,r),b(n,r);
  std::vector<double> rigid_a(36),rigid_b(36),capture_a(6*n+12),capture_b(6*n+12);
  auto loads=old::Loads(fixture);
  for (std::size_t row=0;row<r;++row) {
    const auto node=cin.model->rows().data[row].secondary_domain_node;
    loads[node]=.25*(row+1);loads[3*n+node]=.002*(row+1);
  }
  for (unsigned epoch=0;epoch<3;++epoch) {
    NodalTrialToken token_a,token_b;NodalAssemblyView assembly_a,assembly_b;
    old::Begin(serial,fixture,loads,true,token_a,assembly_a);
    old::Begin(parallel,fixture,loads,true,token_b,assembly_b);
    ASSERT_EQ(old::Advance(serial,token_a,assembly_a,true).status,NodalStatus::Ok);
    ASSERT_EQ(old::Advance(parallel,token_b,assembly_b,true).status,NodalStatus::Ok);
    NodalPreparedView receipt_a,receipt_b;
    Prepared(serial,token_a,a,rigid_a,receipt_a);Prepared(parallel,token_b,b,rigid_b,receipt_b);
    Capture(serial,token_a,capture_a);Capture(parallel,token_b,capture_b);
    fields::Same(a,b);SameDoubles(rigid_a,rigid_b);SameDoubles(capture_a,capture_b);
    EXPECT_EQ(receipt_a.proposed_time,receipt_b.proposed_time);EXPECT_EQ(receipt_a.kinematics.base_epoch,receipt_b.kinematics.base_epoch);
    old::Commit(serial,token_a,assembly_a);old::Commit(parallel,token_b,assembly_b);
  }
  EXPECT_EQ(serial.allocations().device_bytes,serial_forecast.device_bytes);
  EXPECT_EQ(parallel.allocations().device_bytes,forecast.device_bytes);
}
} // namespace tl::fea::cin_parallel_test
