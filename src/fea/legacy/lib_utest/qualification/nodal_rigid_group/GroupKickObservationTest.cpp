#include "GroupKickFixture.h"
namespace rigid_observation_test {
TEST(NodalRigidKickObservation,SixtyFourChangingLoadStepsRetainAppliedReactionAndReplacementChannels) {
  KickFixture fixture; bool saw_reaction_work=false,saw_replacement=false;
  // A finite, larger packet duration makes second-order rigid drift and its
  // replacement channel clearly resolvable above arithmetic roundoff.
  fixture.h=1./128; fixture.packet.body.durations={0,fixture.h/2,fixture.h};
  fixture.after_phase=StoredPhase(fixture.h,0);
  for(unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step); ASSERT_TRUE(fixture.Solve()); rigid::GroupKickObservation output;
    const auto report=rigid::ObserveGroupKick(fixture.Input(),output);
    ASSERT_TRUE(report)<<int(report.status)<<" member "<<report.member<<" dof "<<report.dof<<" residual "<<report.residual<<" budget "<<report.roundoff_budget;
    CompareWork(fixture.Input(),output);
    saw_reaction_work|=std::abs(output.reaction.total)>10*output.roundoff_budget;
    saw_replacement|=std::abs(output.replacement_delta)>10*output.roundoff_budget;
    fixture.Carry(step+1);
  }
  EXPECT_TRUE(saw_reaction_work); EXPECT_TRUE(saw_replacement);
}

TEST(NodalRigidKickObservation,CorruptFinalMemberForceOrCoupleCannotCancelInTotalWork) {
  KickFixture fixture; ASSERT_TRUE(fixture.Solve()); rigid::GroupKickObservation output; output.native_delta=345;
  const auto saved=rigid_step_test::Bytes(output);
  for(unsigned dof=0;dof<6;++dof) {
    auto input=fixture.Input(); auto corrupt=fixture.reaction;
    auto& vector=dof<3?corrupt.back().force:corrupt.back().couple;
    double* values[]{&vector.x,&vector.y,&vector.z}; *values[dof%3]+=.01;
    input.reaction=corrupt.data(); const auto report=rigid::ObserveGroupKick(input,output);
    EXPECT_EQ(report.status,Status::KickMismatch); EXPECT_EQ(report.member,Count-1); EXPECT_EQ(report.dof,dof);
    EXPECT_GT(std::abs(report.residual),report.roundoff_budget); EXPECT_EQ(rigid_step_test::Bytes(output),saved);
  }
  ASSERT_TRUE(rigid::ObserveGroupKick(fixture.Input(),output)); CompareWork(fixture.Input(),output);
}

TEST(NodalRigidKickObservation,ReversalWithZeroAverageVelocityStillRequiresCorrectImpulse) {
  KickFixture fixture; ASSERT_TRUE(fixture.Solve());
  fixture.before[3].velocity.x=.5; fixture.after[3].velocity.x=-.5;
  fixture.applied[3].force.x=0; fixture.reaction[3].force.x=0;
  rigid::GroupKickObservation output; const auto report=rigid::ObserveGroupKick(fixture.Input(),output);
  EXPECT_EQ(report.status,Status::KickMismatch); EXPECT_EQ(report.member,3u); EXPECT_EQ(report.dof,0u);
}

TEST(NodalRigidKickObservation,FiniteStoredVelocityRoundingBudgetAdmitsSubUlpKick) {
  KickFixture fixture; ASSERT_TRUE(fixture.Solve());
  fixture.before[3].velocity.x=1e8; fixture.after[3].velocity.x=1e8;
  fixture.applied[3].force.x=1e-8; fixture.reaction[3].force.x=0;
  rigid::GroupKickObservation output;
  ASSERT_TRUE(rigid::ObserveGroupKick(fixture.Input(),output));
  EXPECT_GT(output.roundoff_budget,0); EXPECT_LE(std::abs(output.native_residual),output.roundoff_budget);
  // The admitted loss is representational; the same metric must reject a kick
  // far larger than rounding at this stored velocity scale.
  fixture.applied[3].force.x=1e7;
  EXPECT_EQ(rigid::ObserveGroupKick(fixture.Input(),output).status,Status::KickMismatch);
}

TEST(NodalRigidKickObservation,PhaseMismatchAndOverflowLeaveOutputUnchanged) {
  KickFixture fixture; ASSERT_TRUE(fixture.Solve()); rigid::GroupKickObservation output; output.aggregate_delta=987;
  const auto saved=rigid_step_test::Bytes(output); auto input=fixture.Input(); input.kick_dt*=2;
  EXPECT_EQ(rigid::ObserveGroupKick(input,output).status,Status::UnsupportedPhase);
  EXPECT_EQ(rigid_step_test::Bytes(output),saved);
  input=fixture.Input(); fixture.reaction[3].couple.z=std::numeric_limits<double>::infinity();
  const auto report=rigid::ObserveGroupKick(input,output);
  EXPECT_EQ(report.status,Status::InvalidInput); EXPECT_EQ(report.member,3u);
  EXPECT_EQ(rigid_step_test::Bytes(output),saved);
  fixture.reaction[3].couple.z=fixture.trial.member[3].reaction_couple.z;
  fixture.after[3].omega.z=1e308;
  EXPECT_EQ(rigid::ObserveGroupKick(fixture.Input(),output).status,Status::NonfiniteResult);
  EXPECT_EQ(rigid_step_test::Bytes(output),saved);
}

TEST(NodalRigidKickObservation,SuperposedWorldRotationPreservesAppliedAndReactionCoupleWork) {
  KickFixture fixture;
  for(unsigned step=0;step<8;++step) { ASSERT_TRUE(fixture.Solve()); fixture.Carry(step+1); }
  ASSERT_TRUE(fixture.Solve()); auto input=fixture.Input(); rigid::GroupKickObservation original,rotated;
  ASSERT_TRUE(rigid::ObserveGroupKick(input,original));
  const auto rotation=rigid_step_test::DenseFrame().axes;
  auto transform=[&](Vec3 v) { return rigid::detail::ToWorld(rotation,v); };
  for(auto* state:{&input.before_group,&input.after_group}) {
    state->center=transform(state->center); state->velocity=transform(state->velocity); state->omega=transform(state->omega);
    state->principal_axes=Multiply(rotation,state->principal_axes);
  }
  for(unsigned i=0;i<Count;++i) {
    fixture.before[i]={transform(fixture.before[i].velocity),transform(fixture.before[i].omega)};
    fixture.after[i]={transform(fixture.after[i].velocity),transform(fixture.after[i].omega)};
    fixture.applied[i]={transform(fixture.applied[i].force),transform(fixture.applied[i].couple)};
    fixture.reaction[i]={transform(fixture.reaction[i].force),transform(fixture.reaction[i].couple)};
  }
  ASSERT_TRUE(rigid::ObserveGroupKick(input,rotated)); CompareWork(input,rotated);
  const auto tolerance=3e-12*original.before.aggregate.total;
  EXPECT_NEAR(rotated.native_delta,original.native_delta,tolerance);
  EXPECT_NEAR(rotated.aggregate_delta,original.aggregate_delta,tolerance);
  EXPECT_NEAR(rotated.applied.translation,original.applied.translation,tolerance);
  EXPECT_NEAR(rotated.applied.rotation,original.applied.rotation,tolerance);
  EXPECT_NEAR(rotated.reaction.translation,original.reaction.translation,tolerance);
  EXPECT_NEAR(rotated.reaction.rotation,original.reaction.rotation,tolerance);
}

TEST(NodalRigidKickObservation,PhaseComparisonAccountsForRoundedAbsoluteTimes) {
  KickFixture fixture; constexpr double h=1e-6,base=1e6;
  fixture.packet.body.durations={0,h/2,h};
  fixture.before_phase={rigid::ObservationPhaseKind::PhysicalInitialization,base,base,base};
  fixture.after_phase={rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame,base+h,base+h/2,base};
  ASSERT_TRUE(fixture.Solve()); rigid::GroupKickObservation output;
  ASSERT_TRUE(rigid::ObserveGroupKick(fixture.Input(),output));
  EXPECT_NE((base+h/2)-base,h/2); // A dt-only relative tolerance would reject this.
}

TEST(NodalRigidKickObservation,LargeFinitePhaseScaleCannotCreateInfiniteAdmissionTolerance) {
  KickFixture fixture; ASSERT_TRUE(fixture.Solve()); auto input=fixture.Input();
  input.after_group=input.before_group; input.after_members=input.before_members;
  std::array<rigid::Wrench,Count> zero{}; input.applied=zero.data(); input.reaction=zero.data();
  input.before_phase=StoredPhase(8e307,6e307); input.after_phase=StoredPhase(1e308,8e307);
  input.kick_dt=1e308; // Actual velocity-time change is 2e307.
  rigid::GroupKickObservation output; output.aggregate_delta=901; const auto saved=rigid_step_test::Bytes(output);
  EXPECT_EQ(rigid::ObserveGroupKick(input,output).status,Status::UnsupportedPhase);
  EXPECT_EQ(rigid_step_test::Bytes(output),saved);
}
} // namespace rigid_observation_test
