// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace beam18_publication_test {
TEST(BeamPublicationCuda, LoadedShellBeamOrdinaryPartPlainShareOneAcceptedTransaction) {
  Rig rig; ASSERT_TRUE(rig.Initialize());
  Snapshot initial,prior,after; ASSERT_TRUE(rig.Read(initial)); prior=initial;
  EXPECT_TRUE(initial.mixed.diagnostics.has_beam18);
  EXPECT_FALSE(initial.mixed.diagnostics.beam18.has_completed_interval);
  const auto allocation=rig.beam.allocations();
  for (unsigned step=0;step<6;++step) {
    SCOPED_TRACE(step);
    fe::NodalTrialToken token; fe::NodalPreparedView view; fe::ShellPhysicalDiagnostics candidate;
    ASSERT_TRUE(rig.Prepare(token,view,candidate));
    ASSERT_TRUE(rig.Read(after)); Exact(prior,after);
    EXPECT_TRUE(candidate.has_beam18); EXPECT_TRUE(candidate.beam18.accepted_force_assembled);
    EXPECT_EQ(candidate.beam18.epoch,candidate.qeph.epoch);
    EXPECT_EQ(candidate.beam18.owner_id,view.owner_id);
    ASSERT_TRUE(Good(rig.mixed.publication.CommitPhysical(rig.mixed.owner,token,candidate,
        {view.owner_id,view.kinematics.base_epoch,view.attempt,existing::Qualification,true})));
    ASSERT_TRUE(rig.Read(after));
    EXPECT_EQ(after.mixed.stamp.epoch,step+1);
    EXPECT_EQ(after.mixed.diagnostics.beam18.epoch,step+1);
    EXPECT_EQ(after.mixed.diagnostics.qeph.epoch,step+1);
    EXPECT_EQ(after.mixed.diagnostics.solids.epoch,step+1);
    prior=after;
  }
  EXPECT_NE(initial.beam,after.beam);
  EXPECT_NE(initial.mixed.values,after.mixed.values);
  std::vector<b::Result> beam(3); b::BatchDiagnostics bd;
  ASSERT_TRUE(Good(rig.beam.CopyAcceptedResults(rig.mixed.owner.accepted(),{beam.data(),beam.size()},&bd)));
  double beam_force=0;
  for (const auto& value:beam) for (auto force:value.rhs_force_n)
    beam_force+=std::abs(force.x)+std::abs(force.y)+std::abs(force.z);
  EXPECT_GT(beam_force,1e-12);
  fe::qeph::ForceTrial shell[2]; fe::qeph::BatchDiagnostics qd;
  ASSERT_TRUE(Good(rig.mixed.qeph.CopyAcceptedResults(rig.mixed.owner.accepted(),shell,2,&qd)));
  double shell_force=0;
  for (const auto& value:shell) for (auto force:value.internal_force)
    shell_force+=std::abs(force.x)+std::abs(force.y)+std::abs(force.z);
  EXPECT_GT(shell_force,1e-12);
  EXPECT_EQ(rig.beam.allocations().device_bytes,allocation.device_bytes);
}
} // namespace beam18_publication_test
