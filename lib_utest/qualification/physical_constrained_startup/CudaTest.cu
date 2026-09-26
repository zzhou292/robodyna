// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace constrained_startup_test {
TEST(ConstrainedStartupCuda, GenuineCinMovingContributorsAndIndependentFixedShellCommitTogether) {
  Rig fixed,uniform(false);
  ASSERT_NO_FATAL_FAILURE(FixIsland(fixed.fixture));
  ASSERT_TRUE(fixed.Initialize());
  ASSERT_TRUE(uniform.Initialize());
  EXPECT_GT(fixed.fixture.WitnessCount(),0u);
  EXPECT_EQ(MovingCache(fixed),MovingCache(uniform));
  ASSERT_NO_FATAL_FAILURE(FixedState(fixed));
  const auto allocations=fixed.owner.allocations();
  for(unsigned step=0;step<3;++step) {
    fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics d;
    ASSERT_TRUE(fixed.Prepare(token,view,d));
    ASSERT_TRUE(common::Good(fixed.publication.CommitPhysical(fixed.owner,token,d,Receipt(view))));
    ASSERT_NO_FATAL_FAILURE(FixedState(fixed));
  }
  EXPECT_EQ(fixed.owner.allocations().device_bytes,allocations.device_bytes);
  EXPECT_EQ(fixed.owner.allocations().device_allocations,allocations.device_allocations);
  common::Snapshot before,after;
  ASSERT_TRUE(fixed.Read(before));
  fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics d;
  ASSERT_TRUE(fixed.Prepare(token,view,d));
  EXPECT_NE(fixed.publication.CommitPhysical(fixed.owner,token,d,Receipt(view,false)).status,
      fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(fixed.Read(after));
  common::Exact(before,after);
  ASSERT_TRUE(fixed.Prepare(token,view,d));
  ASSERT_TRUE(common::Good(fixed.publication.CommitPhysical(fixed.owner,token,d,Receipt(view))));
  ASSERT_NO_FATAL_FAILURE(FixedState(fixed));
}

TEST(ConstrainedStartupCuda, ActualMaskQueryPreservesCinRigidAndAbsentRotationSemantics) {
  Rig rig;
  ASSERT_NO_FATAL_FAILURE(FixIsland(rig.fixture));
  ASSERT_TRUE(common::Good(InitializeOwner(rig.fixture,rig.owner)));
  const auto cin=rig.fixture.domain.Find(901);
  const auto rigid=rig.fixture.domain.Find(9303);
  const auto absent=rig.fixture.domain.Find(9307);
  ASSERT_EQ(rig.fixture.im[cin],0);
  ASSERT_EQ(rig.fixture.present[absent],0);
  const std::size_t nodes[]{cin,rigid,absent,absent};
  EXPECT_EQ(rig.owner.ValidateFreeTranslationalNodes(nodes,4).status,fe::NodalStatus::Ok);
  EXPECT_NE(rig.owner.ValidateFreeRotationalNodes(&absent,1).status,fe::NodalStatus::Ok);
  const auto wall=rig.fixture.domain.Find(20);
  EXPECT_NE(rig.owner.ValidateFreeTranslationalNodes(&wall,1).status,fe::NodalStatus::Ok);
  EXPECT_NE(rig.owner.ValidateFreeTranslationalNodes(nullptr,1).status,fe::NodalStatus::Ok);
  EXPECT_EQ(rig.owner.ValidateFreeTranslationalNodes(nodes,SIZE_MAX).status,fe::NodalStatus::ResourceLimit);
  auto invalid=rig.fixture.domain.node_count();
  EXPECT_NE(rig.owner.ValidateFreeTranslationalNodes(&invalid,1).status,fe::NodalStatus::Ok);
}

TEST(ConstrainedStartupCuda, WrongProjectedFieldsAndWrongDescriptorRejectBeforePublication) {
  Rig rig;
  ASSERT_NO_FATAL_FAILURE(FixIsland(rig.fixture));
  ASSERT_TRUE(common::Good(InitializeOwner(rig.fixture,rig.owner)));
  auto velocity=rig.fixture.v;
  velocity[3*rig.fixture.domain.Find(20)+1]=.5;
  EXPECT_EQ(rig.owner.ValidateInitialConstrainedTranslation(rig.owner.accepted(),velocity.data(),
      rig.fixture.domain.node_count(),rig.fixture.startup.uniform_velocity).status,fe::NodalStatus::InvalidInput);
  velocity=rig.fixture.v;velocity[3*rig.fixture.domain.Find(12)]=0.;
  EXPECT_EQ(rig.owner.ValidateInitialConstrainedTranslation(rig.owner.accepted(),velocity.data(),
      rig.fixture.domain.node_count(),rig.fixture.startup.uniform_velocity).status,fe::NodalStatus::InvalidInput);
  fe::type25::Batch batch;
  fe::type25::BatchConfig c;c.owner=rig.owner.accepted();c.configuration_id=common::Configuration;
  c.qualification_id=common::Qualification;c.element_count=2;c.startup=rig.fixture.startup;
  c.startup.uniform_velocity.y=.25;
  EXPECT_EQ(batch.InitializeMapped(c,rig.fixture.physical,rig.owner,rig.fixture.WitnessSource(),fe::type25::CapacityProfile::Legacy).status,
      fe::type25::BatchStatus::InvalidInput);
  EXPECT_EQ(batch.allocations().device_bytes,0u);
  c.startup=rig.fixture.startup;
  ASSERT_TRUE(common::Good(batch.InitializeMapped(c,rig.fixture.physical,rig.owner,rig.fixture.WitnessSource(),fe::type25::CapacityProfile::Legacy)));
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
}

TEST(ConstrainedStartupCuda, FixedSolidSupportCannotClaimAUniformVelocityTt0Cache) {
  Rig rig;
  ASSERT_NO_FATAL_FAILURE(FixIsland(rig.fixture));
  const auto node=rig.fixture.domain.Find(9307);
  // A genuine owner with a partial world constraint and correctly projected
  // velocity still lies outside this solid cache profile.
  rig.fixture.fixed[node]=2;
  rig.fixture.v[3*node+1]=0.;
  ASSERT_TRUE(rig.Initialize(true,false));
  const auto report=rig.publication.InitializePhysical(rig.owner,rig.fixture.physical,
      rig.fixture.rigid,rig.fixture.WitnessSource(),rig.Participants(),rig.fixture.Identity());
  EXPECT_NE(report.status,fe::ShellPublicationStatus::Success);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
  Rig valid;
  ASSERT_NO_FATAL_FAILURE(FixIsland(valid.fixture));
  ASSERT_TRUE(valid.Initialize());
}

TEST(ConstrainedStartupCuda, EveryBatchMustRetainTheSameImmutableStartupIdentity) {
  Rig rig;
  ASSERT_NO_FATAL_FAILURE(FixIsland(rig.fixture));
  ASSERT_TRUE(rig.Initialize(true,false));
  auto identity=rig.fixture.Identity();identity.startup.kind=fe::ShellBatchStartupKind::ReferenceUniformTranslation;
  EXPECT_NE(rig.publication.InitializePhysical(rig.owner,rig.fixture.physical,rig.fixture.rigid,
      rig.fixture.WitnessSource(),rig.Participants(),identity).status,fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(rig.Attach());
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
}
} // namespace constrained_startup_test
