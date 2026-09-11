// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
namespace extended_resident_test {
TEST_F(ExtendedResidentCuda, FiveFamiliesUseNativeCarriedHistoriesAndOneOwnerCommit) {
  Rig rig;ASSERT_TRUE(rig.Initialize());
  Results accepted(rig.fixture.model);s::BatchDiagnostics diagnostics;
  ASSERT_TRUE(rig.Read(accepted,diagnostics));ASSERT_TRUE(rig.native.Compare(accepted,false));
  const auto allocation=rig.batch.allocations();
  for(unsigned step=0;step<8;++step) {
    SCOPED_TRACE(step);
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Begin(token,assembly));rig.CheckAssembly(token,assembly,accepted);ASSERT_FALSE(HasFailure());
    ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
    s::BatchDiagnostics candidate;ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate)));
    Results next(rig.fixture.model);ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(candidate,next.Buffers())));
    ASSERT_TRUE(rig.Compare(prepared,next));
    Results unchanged(rig.fixture.model);s::BatchDiagnostics old;
    ASSERT_TRUE(rig.Read(unchanged,old));ASSERT_TRUE(rig.native.Compare(unchanged,false));
    EXPECT_EQ(rig.owner.accepted().epoch,step);
    ASSERT_TRUE(Good(Peer::Commit(rig.batch,rig.owner,token,prepared,candidate)));
    rig.native.Commit();ASSERT_TRUE(rig.Read(accepted,diagnostics));ASSERT_TRUE(rig.native.Compare(accepted,false));
    EXPECT_EQ(diagnostics.epoch,step+1);EXPECT_TRUE(diagnostics.has_completed_interval);
    for(auto count:diagnostics.parent_count)EXPECT_EQ(count,1u);
    EXPECT_EQ(rig.batch.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(rig.batch.allocations().device_allocations,1u);
  }
}
TEST_F(ExtendedResidentCuda, ForeignLegacyAuthorityLatePublicationAndReadbackRejectBeforeRetry) {
  Rig rig;ASSERT_TRUE(rig.Initialize(false));
  auto& old=rig.fixture.Mechanics();
  EXPECT_EQ(Peer::PreflightAttach(rig.batch,rig.owner,old.ledger,old.binding,rig.fixture.Witnesses(),
    rig.fixture.model,rig.config).status,s::BatchStatus::InvalidInput);
  ASSERT_TRUE(rig.Attach());
  Results accepted(rig.fixture.model);s::BatchDiagnostics base;ASSERT_TRUE(rig.Read(accepted,base));
  for(unsigned attempt=0;attempt<2;++attempt) {
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Begin(token,assembly));ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
    s::BatchDiagnostics candidate;ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate)));
    Results next(rig.fixture.model);auto buffers=next.Buffers();buffers.count18_law90=0;
    const auto saved=next.foam[0];
    EXPECT_EQ(rig.batch.CopyPreparedResults(candidate,buffers).status,s::BatchStatus::InvalidInput);
    EXPECT_EQ(std::memcmp(&saved,&next.foam[0],sizeof(saved)),0);
    buffers=next.Buffers();buffers.solid18_law90=reinterpret_cast<s::Result18Law90*>(
      const_cast<s::Parent18Law90*>(rig.fixture.model.solid18_law90().data()));
    EXPECT_EQ(rig.batch.CopyPreparedResults(candidate,buffers).status,s::BatchStatus::InvalidInput);
    auto wrong=candidate;wrong.parent_count[4]++;
    EXPECT_EQ(Peer::Preflight(rig.batch,rig.owner,token,prepared,wrong).status,s::BatchStatus::StaleTrial);
    ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(candidate,next.Buffers())));
    ASSERT_TRUE(rig.Compare(prepared,next));
    if(!attempt) {
      EXPECT_EQ(Peer::Commit(rig.batch,rig.owner,token,prepared,candidate,false).status,s::BatchStatus::ElementFailure);
      EXPECT_EQ(rig.owner.accepted().epoch,0u);s::BatchDiagnostics unchanged;
      ASSERT_TRUE(rig.Read(accepted,unchanged));ASSERT_TRUE(rig.native.Compare(accepted,false));
    } else {
      ASSERT_TRUE(Good(Peer::Commit(rig.batch,rig.owner,token,prepared,candidate)));
      rig.native.Commit();ASSERT_TRUE(rig.Read(accepted,base));ASSERT_TRUE(rig.native.Compare(accepted,false));
    }
  }
}
TEST_F(ExtendedResidentCuda, ConstructorProfileAndExactCapRejectThenRetryWithoutAllocation) {
  OwnerFixture fixture;auto config=fixture.Configuration();s::Batch batch;
  config.profile=s::BatchProfile::PhysicalCinV1;
  EXPECT_EQ(batch.InitializeJoined(config,fixture.model).status,s::BatchStatus::InvalidInput);
  EXPECT_EQ(batch.allocations().device_bytes,0u);
  config=fixture.Configuration();s::BatchForecast forecast;
  ASSERT_TRUE(s::Batch::Forecast(config,fixture.model,forecast));
  config.limits.max_device_bytes=forecast.device_bytes-1;
  EXPECT_EQ(batch.InitializeJoined(config,fixture.model).status,s::BatchStatus::ResourceLimit);
  EXPECT_EQ(batch.allocations().device_bytes,0u);
  config.limits.max_device_bytes=forecast.device_bytes;config.limits.max_host_bytes=forecast.startup_host_bytes;
  ASSERT_TRUE(Good(batch.InitializeJoined(config,fixture.model)));
  Results result(fixture.model);s::BatchDiagnostics diagnostics;
  ASSERT_TRUE(Good(Peer::ReadConstructed(batch,result.Buffers(),diagnostics)));
  NativeChecks native;ASSERT_TRUE(native.Initialize(fixture.model,fixture.foam_input,{}));
  EXPECT_TRUE(native.Compare(result,false));
}
TEST_F(ExtendedResidentCuda, LastFoamReadbackFailureKeepsAcceptedStateAndWholeOutputBeforeRetry) {
  Rig rig;ASSERT_TRUE(rig.Initialize());
  for(unsigned attempt=0;attempt<2;++attempt) {
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Begin(token,assembly));ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
    s::BatchDiagnostics candidate;ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate)));
    Results next(rig.fixture.model);
    if(!attempt) {
      const auto bad=std::numeric_limits<double>::quiet_NaN();
      ASSERT_EQ(cudaMemcpy(Peer::PreparedLastLaw90CacheField(rig.batch),&bad,sizeof(bad),cudaMemcpyHostToDevice),cudaSuccess);
      next.rear[0].cache.rhs_force_n[0].x=123;
      next.foam[0].cache.rhs_force_n[7].z=456;
      EXPECT_EQ(rig.batch.CopyPreparedResults(candidate,next.Buffers()).status,s::BatchStatus::NonfiniteResult);
      EXPECT_EQ(next.rear[0].cache.rhs_force_n[0].x,123);
      EXPECT_EQ(next.foam[0].cache.rhs_force_n[7].z,456);
      rig.owner.Discard();rig.batch.DiscardTrial();
      s::BatchDiagnostics accepted;ASSERT_TRUE(rig.Read(next,accepted));
      EXPECT_EQ(accepted.epoch,0u);ASSERT_TRUE(rig.native.Compare(next,false));
    } else {
      ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(candidate,next.Buffers())));
      ASSERT_TRUE(rig.Compare(prepared,next));
      ASSERT_TRUE(Good(Peer::Commit(rig.batch,rig.owner,token,prepared,candidate)));
      rig.native.Commit();
    }
  }
}
TEST_F(ExtendedResidentCuda, SeededEightSlotAssemblyPreservesCoupleAndRotationalStiffnessBits) {
  Rig rig;ASSERT_TRUE(rig.Initialize());
  Results accepted(rig.fixture.model);s::BatchDiagnostics diagnostics;ASSERT_TRUE(rig.Read(accepted,diagnostics));
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
  ASSERT_TRUE(Good(rig.owner.BeginTrial(&token,&assembly)));
  fe::NodalCinAssemblyView cin;ASSERT_TRUE(Good(rig.owner.BorrowCinAssembly(token,&cin)));
  const auto n=rig.config.owner.node_count;
  std::vector<double> seed(n,.125),sentinel(n),read(n);
  for(std::size_t i=0;i<n;++i)sentinel[i]=i%2?-0.0:17.0;
  ASSERT_EQ(cudaMemcpyAsync(cin.translational_stiffness,seed.data(),n*sizeof(double),cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
  double* untouched[]{assembly.forces.couple_x,assembly.forces.couple_y,assembly.forces.couple_z,cin.rotational_stiffness};
  for(auto* p:untouched)ASSERT_EQ(cudaMemcpyAsync(p,sentinel.data(),n*sizeof(double),cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
  ASSERT_TRUE(Good(rig.batch.AssembleAccepted(rig.owner,token,assembly)));
  rig.CheckAssembly(token,assembly,accepted,.125);
  for(auto* p:untouched) {
    ASSERT_EQ(cudaMemcpyAsync(read.data(),p,n*sizeof(double),cudaMemcpyDeviceToHost,assembly.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
    EXPECT_EQ(std::memcmp(read.data(),sentinel.data(),n*sizeof(double)),0);
  }
  rig.owner.Discard();rig.batch.DiscardTrial();
}
} // namespace extended_resident_test
