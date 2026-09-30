#include "Fixture.h"
#include "lib_utest/qualification/nodal_rigid_group/GroupOwnerFixture.h"
#include <limits>
#include "lib_utest/qualification/nodal_rigid_group/PreparedSnapshotCudaProbe.h"
namespace motion_observer_test {
namespace group=rigid_owner_test;
namespace nt=tl_test::nodal_temporal;
TEST_F(Cuda,CompleteRigidPreparedObservationMatchesFullReadbackThroughDiscardAndCommit) {
  group::Fixture fixture;fe::FENodalState owner;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  fe::NodalUniformMotionObserver observer;
  ASSERT_EQ(observer.Initialize(owner).status,Code::Ok);
  const auto allocation=observer.allocations();
  const auto owner_allocation=owner.allocations();
  for(unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step);
    nt::Snapshot initial,held;ASSERT_TRUE(nt::Read(owner,initial));
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
    ASSERT_TRUE(group::Prepare(owner,fixture.Load(step),token,assembly));
    Snapshot fields(fixture.input.n);fe::NodalPreparedView prepared;
    ASSERT_EQ(owner.CopyPrepared(token,fields.Buffer(),&prepared).status,Code::Ok);
    auto first=Sentinel();
    ASSERT_EQ(observer.ObservePrepared(owner,token,{.3,0,0},&first).status,Code::Ok);
    Same(first.motion,Oracle(fixture.input.x.data(),fields,{.3,0,0},prepared.proposed_time));
    EXPECT_TRUE(fe::trial_identity::SamePrepared(first.prepared,prepared));
    EXPECT_EQ(first.motion.nodes,fixture.input.n);
    EXPECT_GT(first.motion.maximum_spin,0);
    ASSERT_TRUE(nt::Read(owner,held));nt::SameState(initial,held);
    owner.Discard();
    ASSERT_TRUE(group::Prepare(owner,fixture.Load(step),token,assembly));
    auto retry=Sentinel();
    ASSERT_EQ(observer.ObservePrepared(owner,token,{.3,0,0},&retry).status,Code::Ok);
    Same(first.motion,retry.motion);
    EXPECT_NE(first.prepared.attempt,retry.prepared.attempt);
    // Observation does not grant the required nonlinear-history validation.
    EXPECT_EQ(owner.Commit(token).status,Code::MissingCandidateValidation);
    ASSERT_TRUE(group::Prepare(owner,fixture.Load(step),token,assembly));
    ASSERT_TRUE(group::Accept(owner,token,assembly));
    EXPECT_EQ(owner.accepted().epoch,step+1);
  }
  EXPECT_EQ(observer.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_bytes,owner_allocation.device_bytes);
}
TEST_F(Cuda,ReactionsAndRigidTailRemainValidatedBeforeQuaternionAndMotionChecks) {
  group::Fixture fixture;fe::FENodalState owner;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  fe::NodalUniformMotionObserver observer;ASSERT_EQ(observer.Initialize(owner).status,Code::Ok);
  const auto n=fixture.input.n;
  for(unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault);
    nt::Snapshot accepted,held;ASSERT_TRUE(nt::Read(owner,accepted));
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
    ASSERT_TRUE(group::Prepare(owner,fixture.Load(),token,assembly));
    fe::NodalPreparedView p;ASSERT_EQ(owner.BorrowPrepared(token,&p).status,Code::Ok);
    const double nan=std::numeric_limits<double>::quiet_NaN(),bad_quaternion=2;
    const auto tail=19*n+fe::rigid::GroupStateValues*fixture.count-1;
    auto* slab=const_cast<double*>(p.kinematics.position_xyz);
    // Deliberate qualification-only corruption, no production write hook.
    if(fault!=2) {
      const auto at=fault==0?19*n-1:tail;
      ASSERT_EQ(cudaMemcpyAsync(slab+at,&nan,sizeof(nan),cudaMemcpyHostToDevice,p.stream),cudaSuccess);
    }
    if(fault>=2)
      ASSERT_EQ(cudaMemcpyAsync(slab+9*n+4*(n-1),&bad_quaternion,sizeof(double),cudaMemcpyHostToDevice,p.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess);
    auto output=Sentinel();const auto untouched=output;
    const auto report=observer.ObservePrepared(owner,token,{.3,0,0},&output);
    EXPECT_EQ(report.status,Code::InvalidOutput);
    EXPECT_STREQ(report.message,fault==2?"Prepared quaternion readback is not unit":"Prepared readback is nonfinite");
    SameOutput(output,untouched);owner.Discard();
    ASSERT_TRUE(nt::Read(owner,held));nt::SameState(accepted,held);
    ASSERT_TRUE(group::Prepare(owner,fixture.Load(),token,assembly));
    ASSERT_EQ(observer.ObservePrepared(owner,token,{.3,0,0},&output).status,Code::Ok);
    owner.Discard();
  }
}
TEST_F(Cuda,HostArgumentRejectionsPreservePreparedAttemptAndStaleCapabilitiesPublishNothing) {
  group::Fixture fixture;fe::FENodalState owner,foreign;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  ASSERT_EQ(fixture.Initialize(foreign).status,Code::Ok);
  fe::NodalUniformMotionObserver observer;ASSERT_EQ(observer.Initialize(owner).status,Code::Ok);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
  ASSERT_TRUE(group::Prepare(owner,fixture.Load(),token,assembly));
  auto output=Sentinel();const auto untouched=output;
  EXPECT_EQ(observer.ObservePrepared(owner,token,{},nullptr).status,Code::InvalidInput);
  EXPECT_EQ(observer.ObservePrepared(owner,token,{},reinterpret_cast<fe::NodalUniformMotionObservation*>(&token)).status,Code::InvalidInput);
  EXPECT_EQ(observer.ObservePrepared(owner,token,{},reinterpret_cast<fe::NodalUniformMotionObservation*>(UINTPTR_MAX-8)).status,Code::InvalidInput);
  EXPECT_EQ(observer.ObservePrepared(owner,token,{std::numeric_limits<double>::infinity(),0,0},&output).status,Code::InvalidInput);
  EXPECT_EQ(observer.ObservePrepared(foreign,token,{},&output).status,Code::StaleTrial);
  SameOutput(output,untouched);
  ASSERT_EQ(observer.ObservePrepared(owner,token,{},&output).status,Code::Ok);
  auto repeated=Sentinel();
  ASSERT_EQ(observer.ObservePrepared(owner,token,{},&repeated).status,Code::Ok);
  SameOutput(output,repeated);
  const auto old=token;owner.Discard();
  ASSERT_TRUE(group::Prepare(owner,fixture.Load(),token,assembly));
  output=untouched;
  EXPECT_EQ(observer.ObservePrepared(owner,old,{},&output).status,Code::StaleTrial);
  SameOutput(output,untouched);
  owner.Discard();
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Code::Ok);
  EXPECT_EQ(observer.ObservePrepared(owner,token,{},&output).status,Code::WrongPhase);
  SameOutput(output,untouched);
}
TEST_F(Cuda,FreshReferenceCapacityAdmissionIsExactAndRetriedWithoutChangingOwner) {
  group::Fixture fixture;fe::FENodalState owner;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  const auto stamp=owner.accepted();
  auto forecast=fe::NodalUniformMotionObserver::Preflight(fixture.input.n);
  ASSERT_EQ(forecast.report.status,Code::Ok);
  fe::NodalUniformMotionLimits limits;
  limits.max_device_bytes=forecast.device_bytes-1;
  fe::NodalUniformMotionObserver observer;
  EXPECT_EQ(observer.Initialize(owner,limits).status,Code::ResourceLimit);
  EXPECT_EQ(observer.allocations().device_bytes,0);
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,owner.accepted()));
  limits.max_device_bytes=forecast.device_bytes;limits.max_host_bytes=forecast.host_bytes;
  ASSERT_EQ(observer.Initialize(owner,limits).status,Code::Ok);
  EXPECT_EQ(observer.allocations().device_bytes,forecast.device_bytes);
  EXPECT_EQ(observer.Initialize(owner,limits).status,Code::WrongPhase);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
  ASSERT_TRUE(group::Prepare(owner,fixture.Load(),token,assembly));
  fe::NodalUniformMotionObserver during_trial;
  EXPECT_EQ(during_trial.Initialize(owner).status,Code::WrongPhase);
  ASSERT_TRUE(group::Accept(owner,token,assembly));
  fe::NodalUniformMotionObserver late;
  EXPECT_EQ(late.Initialize(owner).status,Code::WrongPhase);
}
TEST_F(Cuda,OrdinaryFixedComponentsAndSignedZeroUseTheSameIndependentComponentScan) {
  nt::Initial fixture;fixture.n=4;fixture.h=1e-6;
  fixture.fixed[0]=7;fixture.inverse[0]=0;
  fixture.rotation_fixed[0]=1;fixture.inverse_inertia[0]=0;
  fixture.fixed[1]=2;fixture.v[3]=.25;fixture.v[5]=-.125;
  fixture.v[6]=-.0;fixture.x[8]=-.0;
  fe::FENodalState owner;ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  fe::NodalUniformMotionObserver observer;ASSERT_EQ(observer.Initialize(owner).status,Code::Ok);
  nt::Loads load;load.force[0]=7;load.couple[2]=3;load.force[7]=.25;load.couple[11]=.125;
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView p;
  ASSERT_TRUE(nt::BeginLoad(owner,load,token,assembly));ASSERT_TRUE(nt::Prepare(owner,token,assembly,p));
  Snapshot fields(fixture.n);ASSERT_EQ(owner.CopyPrepared(token,fields.Buffer(),&p).status,Code::Ok);
  fe::NodalUniformMotionObservation output;
  ASSERT_EQ(observer.ObservePrepared(owner,token,{.25,0,0},&output).status,Code::Ok);
  Same(output.motion,Oracle(fixture.x.data(),fields,{.25,0,0},p.proposed_time));
  EXPECT_NE(fields.values[13*fixture.n],0);
  EXPECT_NE(fields.values[16*fixture.n+2],0);
  ASSERT_EQ(owner.Commit(token).status,Code::Ok);
}
TEST_F(Cuda,FiniteExpectedPositionAndDifferenceOverflowStillRejectWithoutPublishing) {
  for(unsigned fault=0;fault<2;++fault) {
    SCOPED_TRACE(fault);
    nt::Initial fixture;fixture.n=1;fixture.h=1;
    fixture.x[0]=fault?std::numeric_limits<double>::max():0;
    fe::FENodalState owner;ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
    fe::NodalUniformMotionObserver observer;ASSERT_EQ(observer.Initialize(owner).status,Code::Ok);
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView p;
    ASSERT_TRUE(nt::BeginLoad(owner,{},token,assembly));ASSERT_TRUE(nt::Prepare(owner,token,assembly,p));
    const double actual=-std::numeric_limits<double>::max();
    if(!fault) {
      ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(p.kinematics.position_xyz),&actual,sizeof(actual),cudaMemcpyHostToDevice,p.stream),cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess);
    }
    Snapshot fields(1);ASSERT_EQ(owner.CopyPrepared(token,fields.Buffer(),&p).status,Code::Ok);
    const tl::math::Vec3 velocity{std::numeric_limits<double>::max(),0,0};
    EXPECT_THROW(Oracle(fixture.x.data(),fields,velocity,p.proposed_time),std::runtime_error);
    auto output=Sentinel();const auto untouched=output;
    EXPECT_EQ(observer.ObservePrepared(owner,token,velocity,&output).status,Code::InvalidOutput);
    SameOutput(output,untouched);
    owner.Discard();
  }
}
TEST_F(Cuda,LegacyTranslationOwnerHasNoInventedOrientationOrSpinChannels) {
  nt::Initial fixture;fixture.n=3;
  fe::FENodalState owner;fe::NodalStateConfig config;
  config.node_count=fixture.n;config.fixed_dt=fixture.h;
  ASSERT_EQ(owner.Initialize(config,{fixture.x.data(),fixture.v.data(),nullptr,fixture.n},
      fixture.inverse.data(),fixture.fixed.data()).status,Code::Ok);
  fe::NodalUniformMotionObserver observer;ASSERT_EQ(observer.Initialize(owner).status,Code::Ok);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Code::Ok);
  ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  ASSERT_EQ(fe::AdvanceTranslations(owner,token).status,Code::Ok);
  fe::NodalUniformMotionObservation out;
  ASSERT_EQ(observer.ObservePrepared(owner,token,{},&out).status,Code::Ok);
  EXPECT_EQ(out.motion.nodes,fixture.n);
  EXPECT_EQ(out.motion.maximum_orientation_error,0);
  EXPECT_EQ(out.motion.maximum_spin,0);
  ASSERT_EQ(owner.Commit(token).status,Code::Ok);
}
TEST_F(Cuda,CompletedPrivateSummaryCopyFailurePoisonsOwnerWithoutPublishing) {
  group::Fixture fixture;fe::FENodalState owner;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  fe::NodalUniformMotionObserver observer;ASSERT_EQ(observer.Initialize(owner).status,Code::Ok);
  const auto stamp=owner.accepted();
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
  ASSERT_TRUE(group::Prepare(owner,fixture.Load(),token,assembly));
  auto output=Sentinel();const auto unchanged=output;
  prepared_snapshot_probe::FailAfterNextDeviceRead();
  EXPECT_EQ(observer.ObservePrepared(owner,token,{},&output).status,Code::DeviceFailure);
  EXPECT_TRUE(prepared_snapshot_probe::CompletedReadBeforeFailure());
  SameOutput(output,unchanged);
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,owner.accepted()));
  EXPECT_EQ(owner.Commit(token).status,Code::DeviceFailure);
  EXPECT_EQ(observer.ObservePrepared(owner,token,{},&output).status,Code::DeviceFailure);
  SameOutput(output,unchanged);
}
TEST_F(Cuda,QuaternionThresholdNeighborsAgreeWithTheExistingCpuSnapshotPredicate) {
  const auto center=std::sqrt(1+1e-12);
  for(const auto q:{std::nextafter(center,0.),center,std::nextafter(center,2.)}) {
    SCOPED_TRACE(q);
    nt::Initial fixture;fixture.n=2;fixture.h=1e-6;
    fe::FENodalState old_owner,new_owner;
    ASSERT_EQ(fixture.Initialize(old_owner).status,Code::Ok);
    ASSERT_EQ(fixture.Initialize(new_owner).status,Code::Ok);
    fe::NodalUniformMotionObserver observer;ASSERT_EQ(observer.Initialize(new_owner).status,Code::Ok);
    fe::NodalTrialToken old_token,new_token;
    fe::NodalAssemblyView old_assembly,new_assembly;fe::NodalPreparedView old_view,new_view;
    ASSERT_TRUE(nt::BeginLoad(old_owner,{},old_token,old_assembly));
    ASSERT_TRUE(nt::Prepare(old_owner,old_token,old_assembly,old_view));
    ASSERT_TRUE(nt::BeginLoad(new_owner,{},new_token,new_assembly));
    ASSERT_TRUE(nt::Prepare(new_owner,new_token,new_assembly,new_view));
    for(const auto& view:{old_view,new_view}) {
      ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.orientation_wxyz)+4,
          &q,sizeof(q),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
    }
    Snapshot fields(fixture.n);auto out=Sentinel();
    const auto old=old_owner.CopyPrepared(old_token,fields.Buffer(),&old_view);
    const auto next=observer.ObservePrepared(new_owner,new_token,{},&out);
    EXPECT_EQ(old.status,next.status);
    if(old.status==Code::Ok)Same(out.motion,Oracle(fixture.x.data(),fields,{},old_view.proposed_time));
  }
}
} // namespace motion_observer_test
