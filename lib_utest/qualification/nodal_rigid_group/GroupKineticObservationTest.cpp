#include "GroupObservationOracle.h"
namespace rigid_observation_test {
TEST(NodalRigidKineticObservation,UnpartitionedNativeChannelPreservesTotalAndRejectsForgedEvidence) {
  Fixture fixture; rigid::GroupKineticObservation before,after;
  ASSERT_TRUE(rigid::ObserveGroupKinetic(fixture.Input(),before));
  for(auto& member:fixture.source) {
    member.unpartitioned_native_inertia_kg_m2=member.physical_inertia_kg_m2;
    member.physical_inertia_kg_m2=0;
  }
  auto group=*fixture.model.groups();
  group.unpartitioned_native_inertia_sum=group.physical_inertia_sum; group.physical_inertia_sum=0;
  auto input=fixture.Input(); input.metric.group=&group;
  ASSERT_TRUE(rigid::ObserveGroupKinetic(input,after));
  EXPECT_EQ(after.members.total,before.members.total);
  EXPECT_EQ(after.aggregate.total,before.aggregate.total);
  EXPECT_EQ(after.replacement,before.replacement);
  EXPECT_EQ(after.members.unpartitioned_native_rotation,before.members.physical_rotation);
  EXPECT_EQ(after.aggregate.unpartitioned_native_member_rotation,before.aggregate.physical_member_rotation);
  EXPECT_EQ(after.members.physical_rotation,0); EXPECT_EQ(after.aggregate.physical_member_rotation,0);
  EXPECT_NEAR(after.members.inertia_partition_residual,0,64*std::numeric_limits<double>::epsilon()*after.members.native_rotation);
  const auto saved=rigid_step_test::Bytes(after);
  group.unpartitioned_native_inertia_sum*=2;
  EXPECT_EQ(rigid::ObserveGroupKinetic(input,after).status,Status::InvalidMetric);
  EXPECT_EQ(rigid_step_test::Bytes(after),saved);
}
TEST(NodalRigidKineticObservation,DenseWorldTensorMatchesIndependentScalarDecomposition) {
  Fixture fixture; const auto input=fixture.Input(); rigid::GroupKineticObservation output;
  const auto report=rigid::ObserveGroupKinetic(input,output); ASSERT_TRUE(report)<<int(report.status);
  const auto expected=Oracle(input); Compare(output,expected);
  const auto w=L(input.group.omega);
  long double diagonal_only=0; for(unsigned a=0;a<3;++a) diagonal_only+=.5L*w[a]*expected.tensor[3*a+a]*w[a];
  EXPECT_GT(std::abs(expected.tensor[1])+std::abs(expected.tensor[2])+std::abs(expected.tensor[5]),.01L);
  EXPECT_GT(std::abs(expected.rotation-diagonal_only),.001L);
  EXPECT_EQ(output.replacement,output.aggregate.total-output.members.total);
  EXPECT_EQ(output.phase.kind,rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame);
}

TEST(NodalRigidKineticObservation,PrimaryRegularizersAndPrincipalCorrectionAreCountedOnce) {
  for(auto kind:{Fixture::Kind::MeasurablePrimary,Fixture::Kind::CorrectedInertia}) {
    Fixture fixture(kind); const auto input=fixture.Input(); rigid::GroupKineticObservation output;
    ASSERT_TRUE(rigid::ObserveGroupKinetic(input,output)); const auto expected=Oracle(input); Compare(output,expected);
    if(kind==Fixture::Kind::MeasurablePrimary) {
      EXPECT_GT(output.aggregate.primary_translation,.01*output.aggregate.translation);
      EXPECT_GT(output.aggregate.primary_parallel_axis_rotation,0);
      EXPECT_GT(output.aggregate.primary_isotropic_rotation,.1*output.aggregate.rotation);
    } else {
      EXPECT_TRUE(fixture.model.groups()[0].regularization.principal_inertia_changed);
      EXPECT_GT(output.aggregate.principal_correction_rotation,.01*output.aggregate.rotation);
    }
    const auto sum=output.aggregate.member_orbital_rotation+output.aggregate.native_member_rotation+
      output.aggregate.primary_parallel_axis_rotation+output.aggregate.primary_isotropic_rotation+
      output.aggregate.principal_correction_rotation;
    EXPECT_NEAR(sum,output.aggregate.rotation,output.aggregate.decomposition_roundoff_budget);
    EXPECT_GT(output.aggregate.native_member_rotation,0); // Adding it again is observably wrong.
  }
}

TEST(NodalRigidKineticObservation,ConstantWorldRotationPreservesAllScalarChannels) {
  Fixture fixture; const auto input=fixture.Input(); rigid::GroupKineticObservation before,after;
  ASSERT_TRUE(rigid::ObserveGroupKinetic(input,before));
  const auto rotation=rigid_step_test::DenseFrame().axes;
  fixture.state.velocity=rigid::detail::ToWorld(rotation,fixture.state.velocity);
  fixture.state.omega=rigid::detail::ToWorld(rotation,fixture.state.omega);
  fixture.state.principal_axes=Multiply(rotation,fixture.state.principal_axes);
  for(auto& m:fixture.motion) { m.velocity=rigid::detail::ToWorld(rotation,m.velocity); m.omega=rigid::detail::ToWorld(rotation,m.omega); }
  ASSERT_TRUE(rigid::ObserveGroupKinetic(fixture.Input(),after)); Compare(after,Oracle(fixture.Input()));
  Near(after.aggregate.total,before.aggregate.total); Near(after.members.total,before.members.total);
  Near(after.aggregate.member_orbital_rotation,before.aggregate.member_orbital_rotation);
}

TEST(NodalRigidKineticObservation,BadPhaseMetricAndLastMemberOverflowLeaveOutputUnchanged) {
  Fixture fixture; auto input=fixture.Input(); rigid::GroupKineticObservation output; output.replacement=123;
  const auto before=rigid_step_test::Bytes(output);
  input.phase={}; EXPECT_EQ(rigid::ObserveGroupKinetic(input,output).status,Status::UnsupportedPhase);
  EXPECT_EQ(rigid_step_test::Bytes(output),before);
  input=fixture.Input(); input.phase.velocity_time=input.phase.position_time;
  EXPECT_EQ(rigid::ObserveGroupKinetic(input,output).status,Status::UnsupportedPhase);
  auto changed=*fixture.model.groups(); input=fixture.Input(); input.metric.group=&changed;
  changed.native_total_inertia_sum*=2;
  EXPECT_EQ(rigid::ObserveGroupKinetic(input,output).status,Status::InvalidMetric);
  EXPECT_EQ(rigid_step_test::Bytes(output),before);
  input=fixture.Input(); fixture.motion.back().velocity.x=1e308;
  const auto report=rigid::ObserveGroupKinetic(input,output);
  EXPECT_EQ(report.status,Status::NonfiniteResult); EXPECT_EQ(report.member,Count-1);
  EXPECT_EQ(rigid_step_test::Bytes(output),before);
  fixture.SetRigidMotion(); ASSERT_TRUE(rigid::ObserveGroupKinetic(fixture.Input(),output));
}
} // namespace rigid_observation_test
