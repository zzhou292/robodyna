#include "PreparedSnapshotFixture.h"
namespace prepared_snapshot_test {
TEST_F(Cuda,PreparedMotionAndNonzeroGroupReactionsMatchTheSoleAcceptedSlabAfterCommit) {
  Fixture fixture;fe::FENodalState owner;ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  const auto allocation=owner.allocations();
  for(unsigned step=0;step<4;++step) {
    nt::Snapshot before,held,accepted;ASSERT_TRUE(nt::Read(owner,before));
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
    ASSERT_TRUE(Prepare(owner,fixture.Load(step),token,assembly));const auto token_bytes=Image(token);
    fe::NodalPreparedView borrowed;ASSERT_EQ(owner.BorrowPrepared(token,&borrowed).status,Code::Ok);
    Output trial;ASSERT_TRUE(Copy(owner,token,trial));
    EXPECT_TRUE(fe::trial_identity::SamePrepared(borrowed,trial.prepared));EXPECT_EQ(Image(token),token_bytes);
    EXPECT_EQ(trial.prepared.kick_dt,step?fixture.input.h:fixture.input.h/2);
    EXPECT_EQ(trial.prepared.rigid_groups.group_count,2u);EXPECT_EQ(trial.prepared.rigid_groups.member_count,8u);
    double force=0,couple=0;
    for(unsigned j=0;j<24;++j) {force+=std::abs(trial.nodes.reaction[j]);couple+=std::abs(trial.nodes.couple[j]);}
    EXPECT_GT(force,1e-6);EXPECT_GT(couple,1e-6);
    ASSERT_TRUE(nt::Read(owner,held));nt::SameState(before,held);
    // Reading before the receipt grants no validation authority. Complete the
    // receipt separately, then exercise the same snapshot in Ready phase.
    ASSERT_EQ(fe::CompleteNodalValidation(owner,token,
      {assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,Qualification,true}).status,Code::Ok);
    Output ready;ASSERT_TRUE(Copy(owner,token,ready));SameOutput(trial,ready);
    ASSERT_EQ(owner.Commit(token).status,Code::Ok);ASSERT_TRUE(nt::Read(owner,accepted));
    SameFields(trial.nodes,accepted,fixture.input.n);
    EXPECT_EQ(accepted.stamp.time,trial.prepared.proposed_time);
    EXPECT_EQ(accepted.stamp.velocity_time,trial.prepared.velocity_time);
    EXPECT_EQ(accepted.stamp.reaction_time,trial.prepared.base_time);
    EXPECT_EQ(accepted.stamp.reaction_base_epoch,trial.prepared.base_kinematics.base_epoch);
    EXPECT_EQ(accepted.stamp.reaction_kick_dt,trial.prepared.kick_dt);
    Output stale;const auto untouched=stale;
    EXPECT_EQ(owner.CopyPrepared(token,stale.nodes.buffer(),&stale.prepared).status,Code::StaleTrial);
    SameOutput(stale,untouched);EXPECT_EQ(Image(token),token_bytes);
    EXPECT_TRUE(fe::trial_identity::SameStamp(owner.accepted(),accepted.stamp));
    EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
  }
}
TEST_F(Cuda,RejectedFirstAndLaterPreparedSnapshotsRetryExactlyWithoutChangingAcceptedState) {
  Fixture fixture;fe::FENodalState owner,control;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);ASSERT_EQ(fixture.Initialize(control).status,Code::Ok);
  for(unsigned step=0;step<3;++step) {
    nt::Snapshot before,held;ASSERT_TRUE(nt::Read(owner,before));
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
    ASSERT_TRUE(Prepare(owner,fixture.Load(step),token,assembly));Output candidate;ASSERT_TRUE(Copy(owner,token,candidate));
    EXPECT_NE(fe::CompleteNodalValidation(owner,token,
      {assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,Qualification,false}).status,Code::Ok);
    owner.Discard();ASSERT_TRUE(nt::Read(owner,held));nt::SameState(before,held);
    ASSERT_TRUE(Prepare(owner,fixture.Load(step),token,assembly));Output retry;ASSERT_TRUE(Copy(owner,token,retry));
    SameFields(candidate.nodes,retry.nodes,fixture.input.n);
    EXPECT_EQ(candidate.prepared.proposed_time,retry.prepared.proposed_time);
    EXPECT_EQ(candidate.prepared.velocity_time,retry.prepared.velocity_time);
    EXPECT_EQ(candidate.prepared.kick_dt,retry.prepared.kick_dt);EXPECT_NE(candidate.prepared.attempt,retry.prepared.attempt);
    ASSERT_TRUE(Accept(owner,token,assembly));ASSERT_TRUE(Step(control,fixture.Load(step)));SameOwners(owner,control);
  }
}
TEST_F(Cuda,LegacyTranslationReadbackSupportsXvOnlyAndRejectsExtendedRequestsWithoutConsumingTrial) {
  nt::Initial initial;fe::FENodalState owner;fe::NodalStateConfig config;
  config.node_count=initial.n;config.fixed_dt=initial.h;
  ASSERT_EQ(owner.Initialize(config,{initial.x.data(),initial.v.data(),nullptr,initial.n},
    initial.inverse.data(),initial.fixed.data()).status,Code::Ok);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Code::Ok);ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  ASSERT_EQ(fe::AdvanceTranslations(owner,token).status,Code::Ok);
  Output output;const auto before=output;
  EXPECT_EQ(owner.CopyPrepared(token,output.nodes.buffer(),&output.prepared).status,Code::UnsupportedRotation);
  SameOutput(output,before);
  ASSERT_EQ(owner.CopyPrepared(token,{output.nodes.x.data(),output.nodes.v.data(),nt::Capacity},&output.prepared).status,Code::Ok);
  EXPECT_EQ(output.nodes.x[0],0);EXPECT_EQ(output.nodes.v[0],0);
  EXPECT_EQ(output.nodes.q,before.nodes.q);EXPECT_EQ(output.nodes.reaction,before.nodes.reaction);
  EXPECT_EQ(output.nodes.couple,before.nodes.couple);EXPECT_EQ(output.nodes.omega,before.nodes.omega);
  ASSERT_EQ(owner.Commit(token).status,Code::Ok);
}
} // namespace prepared_snapshot_test
