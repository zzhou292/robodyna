// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Snapshot.h"
#include <sstream>
namespace controlled_resident_test {
TEST_F(ControlledResidentCuda, FourEightSixteenThirtyTwoHaveIdenticalPhysicsAndCarriedNativeFields) {
  for(const auto units:{s::control::UnitScale{1,1,1},s::control::UnitScale{.001,1000,1}})for(bool collapsed:{false,true}){
    std::string baseline;
    for(unsigned blocks:{4u,8u,16u,32u}){
      SCOPED_TRACE(blocks);Rig rig(units,collapsed,blocks,32);ASSERT_TRUE(rig.Initialize());
      s::BatchForecast forecast;ASSERT_TRUE(s::Batch::Forecast(rig.config,rig.fixture.model,forecast));EXPECT_EQ(forecast.controlled_packet_blocks,blocks);
      Results results(rig.fixture.model);s::BatchDiagnostics diagnostics;ASSERT_TRUE(rig.Read(results,diagnostics));
      std::ostringstream values;
      for(unsigned step=0;step<=16;++step){
        if(step){fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
          ASSERT_TRUE(rig.Begin(token,assembly));ASSERT_TRUE(rig.Prepare(token,assembly,prepared));s::BatchDiagnostics proposed;
          ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&proposed)));
          ASSERT_TRUE(Good(Peer::Commit(rig.batch,rig.owner,token,prepared,proposed)));ASSERT_TRUE(rig.Read(results,diagnostics));}
        EXPECT_EQ(diagnostics.controlled_packet_blocks,blocks);EXPECT_EQ(diagnostics.controlled_worker_slots,forecast.controlled_worker_slots);
        // Different live owners and resource choices are deliberate; all
        // physics, native numeric histories, clocks and other diagnostics agree.
        auto compare=diagnostics;compare.owner_id=0;
        snapshot::Emit(values,std::to_string(step),results,compare);
      }
      if(blocks==4)baseline=values.str();else EXPECT_EQ(values.str(),baseline);
    }
  }
}
TEST_F(ControlledResidentCuda, ConstructorAndDiagnosticsUseTheForecastBudgetFallback) {
  Rig rig({.001,1000,1},true,32,32);auto config=rig.fixture.Configuration();config.limits.max_controlled_packet_blocks=4;
  s::BatchForecast floor;ASSERT_TRUE(s::Batch::Forecast(config,rig.fixture.model,floor));
  rig.limits.max_device_bytes=floor.device_bytes;rig.limits.max_host_bytes=floor.startup_host_bytes;
  ASSERT_TRUE(rig.Initialize());Results r(rig.fixture.model);s::BatchDiagnostics d;ASSERT_TRUE(rig.Read(r,d));
  EXPECT_EQ(d.controlled_packet_blocks,4u);EXPECT_EQ(d.controlled_worker_slots,4u);
  EXPECT_EQ(rig.batch.allocations().device_bytes,floor.device_bytes);
}
TEST_F(ControlledResidentCuda, CompiledPacketOccupancyIsRecordedWithoutChangingWork) {
  d::ControlledKernelResources value;ASSERT_EQ(d::InspectControlledKernel(value),cudaSuccess);
  ASSERT_GT(value.multiprocessors,0);ASSERT_GT(value.active_blocks_per_multiprocessor,0);
  EXPECT_GE(value.attributes.maxThreadsPerBlock,int(d::controlled::Threads));
  RecordProperty("packet_registers",value.attributes.numRegs);
  RecordProperty("packet_local_size_bytes",std::uint64_t(value.attributes.localSizeBytes));
  RecordProperty("packet_shared_size_bytes",std::uint64_t(value.attributes.sharedSizeBytes));
  RecordProperty("packet_active_ctas_per_sm",value.active_blocks_per_multiprocessor);
  RecordProperty("device_multiprocessors",value.multiprocessors);
  RecordProperty("device_max_threads_per_sm",value.maximum_threads_per_multiprocessor);
}
} // namespace controlled_resident_test
