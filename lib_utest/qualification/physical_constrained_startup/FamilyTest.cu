// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../beam18_publication/Fixture.h"
#include "../type45_resident/OwnerFixture.h"
namespace constrained_startup_test {
TEST(ConstrainedFamilyCuda, BeamCommonPublicationUsesTheIdenticalConstrainedDescriptor) {
  beam18_publication_test::Rig rig(Startup(),true),uniform(Startup(false),true);
  ASSERT_NO_FATAL_FAILURE(FixIsland(rig.mixed.fixture));
  ASSERT_TRUE(rig.Initialize());
  ASSERT_TRUE(uniform.Initialize());
  std::vector<fe::beam18::Result> first(rig.source.model.parents().size()),second(first.size());
  fe::beam18::BatchDiagnostics a,b;
  ASSERT_TRUE(common::Good(rig.beam.CopyAcceptedResults(rig.mixed.owner.accepted(),
      {first.data(),first.size()},&a)));
  ASSERT_TRUE(common::Good(uniform.beam.CopyAcceptedResults(uniform.mixed.owner.accepted(),
      {second.data(),second.size()},&b)));
  EXPECT_EQ(beam18_publication_test::BeamValues(first),beam18_publication_test::BeamValues(second));
  for(unsigned step=0;step<2;++step) {
    fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics d;
    ASSERT_TRUE(rig.Prepare(token,view,d));
    ASSERT_TRUE(common::Good(rig.mixed.publication.CommitPhysical(rig.mixed.owner,token,d,Receipt(view))));
    ASSERT_NO_FATAL_FAILURE(FixedState(rig.mixed));
  }
}
TEST(ConstrainedFamilyCuda, JointVirginAndAutomaticStiffnessPhasesRemainDistinct) {
  type45_resident_test::Rig rig(Startup(),true);
  ASSERT_NO_FATAL_FAILURE(FixIsland(rig.physical.fixture));
  ASSERT_TRUE(rig.Initialize());
  std::array<fe::type45::Result,3> initial,after;
  fe::type45::BatchDiagnostics before,diagnostics;
  ASSERT_TRUE(rig.CopyAccepted(initial,before));
  EXPECT_FALSE(before.automatic_stiffness_initialized);
  for(const auto& value:initial)EXPECT_FALSE(value.automatic_stiffness_initialized);
  fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics d;
  ASSERT_TRUE(rig.Prepare(token,view,d));
  ASSERT_TRUE(common::Good(rig.physical.publication.CommitPhysical(rig.physical.owner,token,d,Receipt(view))));
  ASSERT_TRUE(rig.CopyAccepted(after,diagnostics));
  EXPECT_TRUE(diagnostics.automatic_stiffness_initialized);
  for(const auto& value:after)EXPECT_TRUE(value.automatic_stiffness_initialized);
  ASSERT_NO_FATAL_FAILURE(FixedState(rig.physical));
}
} // namespace constrained_startup_test
