// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
namespace controlled_resident_test {
TEST_F(ControlledResidentCuda, SameOwnerTypedNativeHistoryAssemblyAndRepeatedSlots) {
  for(const auto units:{s::control::UnitScale{1,1,1},s::control::UnitScale{.001,1000,1}})for(bool collapsed:{false,true}){
    SCOPED_TRACE(units.length_m);
    SCOPED_TRACE(collapsed);Rig rig(units,collapsed);ASSERT_TRUE(rig.Initialize());
    Results accepted(rig.fixture.model);s::BatchDiagnostics initial;ASSERT_TRUE(rig.Read(accepted,initial));rig.Compare(accepted);ASSERT_FALSE(HasFailure());
    const auto allocations=rig.batch.allocations();
    for(unsigned step=0;step<4;++step){
      fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
      ASSERT_TRUE(rig.Begin(token,assembly));rig.CheckAssembly(token,assembly,accepted);ASSERT_FALSE(HasFailure());
      ASSERT_TRUE(rig.Prepare(token,assembly,prepared));ASSERT_TRUE(rig.ReferenceStep(prepared));
      s::BatchDiagnostics proposed;ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&proposed)));
      Results next(rig.fixture.model);ASSERT_TRUE(Good(rig.batch.CopyPreparedResultsWithControls(proposed,next.Buffers())));rig.Compare(next);ASSERT_FALSE(HasFailure());
      double hwork=0,fwork=0;for(const auto& x:next.h24)hwork+=x.cache.response.distortion_work_j;for(const auto& x:next.foam)fwork+=x.cache.response.distortion_work_j;
      EXPECT_EQ(proposed.distortion_work_increment_j[1],hwork);EXPECT_EQ(proposed.distortion_work_increment_j[4],fwork);
      Results old(rig.fixture.model);s::BatchDiagnostics unchanged;ASSERT_TRUE(rig.Read(old,unchanged));EXPECT_EQ(unchanged.epoch,step);
      EXPECT_EQ(std::memcmp(&old.h24[0].history.native()->values,&accepted.h24[0].history.native()->values,sizeof(old.h24[0].history.native()->values)),0);
      ASSERT_TRUE(Good(Peer::Commit(rig.batch,rig.owner,token,prepared,proposed)));
      ASSERT_TRUE(rig.Read(accepted,initial));EXPECT_EQ(initial.epoch,step+1);rig.Compare(accepted);
      EXPECT_EQ(rig.batch.allocations().device_bytes,allocations.device_bytes);EXPECT_EQ(rig.batch.allocations().device_allocations,1u);
    }
  }
}
TEST_F(ControlledResidentCuda, LegacyReadbackLateFailureAndWholeOutputRollback) {
  Rig rig({.001,1000,1});ASSERT_TRUE(rig.Initialize());
  extended_resident_test::Results legacy(rig.fixture.model);s::BatchDiagnostics untouched;untouched.epoch=987;
  EXPECT_EQ(rig.batch.CopyAcceptedResults(rig.owner.accepted(),legacy.Buffers(),&untouched).status,s::BatchStatus::InvalidInput);EXPECT_EQ(untouched.epoch,987u);
  for(unsigned attempt=0;attempt<2;++attempt){
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Begin(token,assembly));ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
    s::BatchDiagnostics next;ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&next)));
    EXPECT_EQ(rig.batch.CopyPreparedResults(next,legacy.Buffers()).status,s::BatchStatus::InvalidInput);
    Results result(rig.fixture.model);result.h24[0].cache.rhs_force_n[0].x=123;result.foam[0].cache.rhs_force_n[0].x=456;
    auto buffers=result.Buffers();buffers.count18_law90=0;
    EXPECT_EQ(rig.batch.CopyPreparedResultsWithControls(next,buffers).status,s::BatchStatus::InvalidInput);
    EXPECT_EQ(result.h24[0].cache.rhs_force_n[0].x,123);EXPECT_EQ(result.foam[0].cache.rhs_force_n[0].x,456);
    if(!attempt){
      EXPECT_EQ(Peer::Commit(rig.batch,rig.owner,token,prepared,next,false).status,s::BatchStatus::ElementFailure);EXPECT_EQ(rig.owner.accepted().epoch,0u);
      s::BatchDiagnostics d;ASSERT_TRUE(rig.Read(result,d));EXPECT_EQ(d.epoch,0u);
    }else{
      const double bad=std::numeric_limits<double>::quiet_NaN();auto* field=Peer::PreparedLastLaw90CacheField(rig.batch);
      ASSERT_EQ(cudaMemcpyAsync(field,&bad,sizeof(bad),cudaMemcpyHostToDevice,prepared.stream),cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(prepared.stream),cudaSuccess);
      EXPECT_EQ(rig.batch.CopyPreparedResultsWithControls(next,result.Buffers()).status,s::BatchStatus::NonfiniteResult);
      EXPECT_EQ(result.h24[0].cache.rhs_force_n[0].x,123);EXPECT_EQ(result.foam[0].cache.rhs_force_n[0].x,456);
      rig.owner.Discard();rig.batch.DiscardTrial();s::BatchDiagnostics d;ASSERT_TRUE(rig.Read(result,d));EXPECT_EQ(d.epoch,0u);
    }
  }
}
TEST_F(ControlledResidentCuda, ExactDeviceCapRejectsWithoutAllocationAndRetries) {
  OwnerFixture fixture(false,true,{.001,1000,1});auto c=fixture.Configuration();c.limits=s::SourceControlledBatchLimits();c.limits.max_controlled_packet_blocks=4;s::BatchForecast f;s::Batch batch;
  ASSERT_TRUE(s::Batch::Forecast(c,fixture.model,f));c.limits.max_device_bytes=f.device_bytes-1;
  EXPECT_EQ(batch.InitializeJoined(c,fixture.model).status,s::BatchStatus::ResourceLimit);EXPECT_EQ(batch.allocations().device_bytes,0u);
  c.limits.max_device_bytes=f.device_bytes;c.limits.max_host_bytes=f.startup_host_bytes;
  ASSERT_TRUE(Good(batch.InitializeJoined(c,fixture.model)));EXPECT_EQ(batch.allocations().device_bytes,f.device_bytes);
}
} // namespace controlled_resident_test
