#include "ForceStageCaptureFixture.h"

namespace force_stage_capture_test {
TEST_F(Cuda,ActualFirstAndLaterCaptureLeavesDisabledOwnerBitwiseUnchangedAndAllocationsStable) {
  Fixture fixture;fe::FENodalState captured,disabled;
  ASSERT_EQ(Initialize(fixture,captured,Config(fixture)).status,Code::Ok);
  ASSERT_EQ(fixture.Initialize(disabled).status,Code::Ok);
  const auto allocated=captured.allocations(),legacy=disabled.allocations();
  EXPECT_EQ(allocated.device_allocations,legacy.device_allocations);
  EXPECT_EQ(allocated.device_bytes-legacy.device_bytes,6*(fixture.input.n+fixture.count)*sizeof(double));
  for(unsigned step=0;step<40;++step) {
    const auto loads=fixture.Load(step);fe::NodalTrialToken token;fe::NodalAssemblyView view;
    nt::Snapshot before,still_accepted;ASSERT_TRUE(nt::Read(captured,before));
    ASSERT_TRUE(Prepare(captured,loads,token,view));Capture sample;
    ASSERT_EQ(captured.CopyPreparedForceStage(token,sample.buffer(),&sample.prepared).status,Code::Ok);
    EXPECT_EQ(sample.prepared.base_time,step*fixture.input.h);
    EXPECT_EQ(sample.prepared.kick_dt,step?fixture.input.h:.5*fixture.input.h);
    EXPECT_TRUE(fe::SameRigidGroupInfo(sample.prepared.rigid_groups,{Source,2,8}));
    for(unsigned g=0;g<2;++g) {
      EXPECT_EQ(sample.groups[g].source_group_id,300+g);EXPECT_EQ(sample.groups[g].source_node_set_id,400+g);
      EXPECT_EQ(sample.groups[g].member_count,4u);
    }
    const auto ordinary=fixture.input.n-1;
    EXPECT_EQ(sample.a[3*ordinary],2);EXPECT_EQ(sample.a[3*ordinary+1],0);EXPECT_EQ(sample.a[3*ordinary+2],0);
    EXPECT_EQ(sample.ar[3*ordinary],0);EXPECT_EQ(sample.ar[3*ordinary+1],0);EXPECT_EQ(sample.ar[3*ordinary+2],.25);
    ASSERT_TRUE(nt::Read(captured,still_accepted));nt::SameState(before,still_accepted);
    if(!step) {
      ASSERT_EQ(fe::CompleteNodalValidation(captured,token,
        {view.owner_id,view.accepted.base_epoch,view.attempt,Qualification,true}).status,Code::Ok);
      Capture ready;ASSERT_EQ(captured.CopyPreparedForceStage(token,ready.buffer(),&ready.prepared).status,Code::Ok);
      SameCapture(sample,ready);ASSERT_EQ(captured.Commit(token).status,Code::Ok);
    } else ASSERT_TRUE(Accept(captured,token,view));
    ASSERT_TRUE(Step(disabled,loads));SameOwners(captured,disabled);
    const auto unchanged=sample;
    EXPECT_EQ(captured.CopyPreparedForceStage(token,sample.buffer(),&sample.prepared).status,Code::StaleTrial);
    SameCapture(sample,unchanged);
    EXPECT_EQ(captured.allocations().device_bytes,allocated.device_bytes);
    EXPECT_EQ(captured.allocations().device_allocations,allocated.device_allocations);
  }
}
TEST_F(Cuda,FixedAndComponentFixedOrdinaryDofsCaptureTheirActualZeroAcceleration) {
  for(bool fully_fixed:{false,true}) {
    Fixture fixture;const auto n=fixture.input.n-1;fixture.input.fixed[n]=fully_fixed?7:1;
    fixture.input.v[3*n]=0;fixture.input.rotation_fixed[n]=1;fixture.input.inverse_inertia[n]=0;
    if(fully_fixed)fixture.input.inverse[n]=0;
    fe::FENodalState owner;ASSERT_EQ(Initialize(fixture,owner,Config(fixture)).status,Code::Ok);
    auto load=fixture.Load();load.force[3*n+1]=6;load.force[3*n+2]=-2;
    fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(Prepare(owner,load,token,view));Capture sample;
    ASSERT_EQ(owner.CopyPreparedForceStage(token,sample.buffer(),&sample.prepared).status,Code::Ok);
    EXPECT_EQ(sample.a[3*n],0);EXPECT_EQ(sample.a[3*n+1],fully_fixed?0:3);EXPECT_EQ(sample.a[3*n+2],fully_fixed?0:-1);
    for(unsigned axis=0;axis<3;++axis)EXPECT_EQ(sample.ar[3*n+axis],0);
    ASSERT_TRUE(Accept(owner,token,view));
  }
}
TEST_F(Cuda,ActualCaptureRetainsSubUlpAccelerationLostByVelocityDifference) {
  Fixture fixture;const auto n=fixture.input.n-1;fixture.input.v[3*n]=std::ldexp(1.,50);
  fe::FENodalState owner;ASSERT_EQ(Initialize(fixture,owner,Config(fixture)).status,Code::Ok);
  nt::Loads loads;loads.force[3*n]=std::ldexp(1.,-40);
  fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(Prepare(owner,loads,token,view));Capture sample;
  ASSERT_EQ(owner.CopyPreparedForceStage(token,sample.buffer(),&sample.prepared).status,Code::Ok);
  nt::Snapshot candidate;fe::NodalPreparedView same;
  ASSERT_EQ(owner.CopyPrepared(token,candidate.buffer(),&same).status,Code::Ok);
  EXPECT_EQ(candidate.v[3*n],fixture.input.v[3*n]);
  EXPECT_EQ(sample.a[3*n],std::ldexp(1.,-41));EXPECT_GT(sample.a[3*n],0);
  EXPECT_NE(sample.a[3*n],(candidate.v[3*n]-fixture.input.v[3*n])/sample.prepared.kick_dt);
}
} // namespace force_stage_capture_test
