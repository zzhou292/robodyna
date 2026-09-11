// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include <limits>

namespace physical_publication_test {
TEST(PhysicalPublicationCuda, LateAttachFailureClaimsNothingThenExactCompleteScope) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize(false,false));
  const auto before = rig.owner.accepted();
  EXPECT_EQ(rig.publication.InitializePhysical(rig.owner,rig.fixture.physical,rig.fixture.rigid,
      rig.fixture.WitnessSource(),rig.Participants(),rig.fixture.Identity()).status,
      fe::ShellPublicationStatus::NotInitialized);
  EXPECT_EQ(rig.publication.allocations().device_bytes,0u);
  ASSERT_TRUE(rig.InitializeSolids());
  ASSERT_TRUE(rig.Attach());
  EXPECT_TRUE(fe::trial_identity::SameStamp(before,rig.owner.accepted()));
  fe::ShellBatchPublication other;
  EXPECT_NE(other.InitializePhysical(rig.owner,rig.fixture.physical,rig.fixture.rigid,
      rig.fixture.WitnessSource(),rig.Participants(),rig.fixture.Identity()).status,
      fe::ShellPublicationStatus::Success);
  Snapshot accepted;
  ASSERT_TRUE(rig.Read(accepted));
  EXPECT_FALSE(accepted.diagnostics.kinetic_available);
  fe::ShellBatchDiagnostics legacy;
  EXPECT_EQ(rig.publication.CopyAcceptedDiagnostics(rig.owner.accepted(),&legacy).status,
      fe::ShellPublicationStatus::NotJoined);
  auto foreign = rig.Shells();
  fe::t3::T3Batch wrong;
  foreign.t3 = &wrong;
  EXPECT_EQ(rig.publication.ValidateAcceptedActivitySources(rig.owner,foreign,
      rig.fixture.source.shells.inventory()).status,fe::ShellPublicationStatus::NotJoined);
  auto* overlap = reinterpret_cast<fe::ShellPhysicalDiagnostics*>(
      const_cast<fe::NodalCoefficientNode*>(rig.fixture.ledger.nodes().data()));
  EXPECT_EQ(rig.publication.CopyAcceptedPhysicalDiagnostics(rig.owner.accepted(),overlap).status,
      fe::ShellPublicationStatus::InvalidInput);
}
TEST(PhysicalPublicationCuda, RejectedCaptureKeepsEveryHistoryActivityAndOwnerThenOneCommit) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  fe::ShellPhysicalDiagnostics candidate;
  ASSERT_TRUE(rig.Prepare(token,view,candidate));
  ASSERT_TRUE(rig.Read(after));
  Exact(before,after);
  EXPECT_NE(rig.publication.CommitPhysical(rig.owner,token,candidate,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,Qualification,false}).status,
      fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(rig.Read(after));
  Exact(before,after);
  const auto rejected_attempt = view.attempt;
  ASSERT_TRUE(rig.Prepare(token,view,candidate));
  EXPECT_GT(view.attempt,rejected_attempt);
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,token,candidate,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,Qualification,true})));
  ASSERT_TRUE(rig.Read(after));
  EXPECT_EQ(after.stamp.epoch,1u);
  EXPECT_NE(before.values,after.values);
  EXPECT_EQ(after.diagnostics.qeph.epoch,1u);
  EXPECT_EQ(after.diagnostics.t3.epoch,1u);
  EXPECT_EQ(after.diagnostics.qbat.epoch,1u);
  EXPECT_EQ(after.diagnostics.type25.epoch,1u);
  EXPECT_EQ(after.diagnostics.type13.epoch,1u);
  EXPECT_EQ(after.diagnostics.solids.epoch,1u);
  ASSERT_TRUE(rig.Prepare(token,view,candidate));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,token,candidate,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,Qualification,true})));
  ASSERT_TRUE(rig.Read(after));
  EXPECT_EQ(after.stamp.epoch,2u);
  EXPECT_EQ(Bits(after.stamp.reaction_kick_dt),Bits(H));
}
TEST(PhysicalPublicationCuda, LastContributorNumericalFailureAndForgedLastDiagnosticDiscardAll) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  Snapshot before,after;
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView view;
  fe::ShellPhysicalDiagnostics candidates;
  ASSERT_TRUE(rig.Prepare(token,view,candidates));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,token,candidates,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,Qualification,true})));
  ASSERT_TRUE(rig.Read(before));
  ASSERT_TRUE(rig.Begin(token,assembly));
  ASSERT_TRUE(rig.Advance(token,assembly,view));
  ASSERT_TRUE(rig.Evaluate(token,view,candidates,false));
  const auto last = rig.fixture.domain.Find(9307);
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.position_xyz)+3*last,&invalid,
      sizeof(invalid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  EXPECT_NE(rig.solids.EvaluateCandidate(rig.owner,token,view,&candidates.solids).status,
      fe::solids::BatchStatus::Success);
  fe::ShellPhysicalDiagnostics output;
  output.valid = true;
  const auto untouched = output;
  EXPECT_NE(rig.publication.PreparePhysical(rig.owner,token,
      {&candidates.qeph,&candidates.t3,&candidates.qbat,&candidates.type25,&candidates.type13,&candidates.solids},&output).status,
      fe::ShellPublicationStatus::Success);
  EXPECT_TRUE(fe::shell_publication_detail::SamePhysicalDiagnostics(output,untouched));
  ASSERT_TRUE(rig.Read(after));
  Exact(before,after);
  ASSERT_TRUE(rig.Prepare(token,view,output));
  auto altered = output;
  altered.solids.physical_hourglass_work_increment_j[2] += 1;
  EXPECT_NE(rig.publication.CommitPhysical(rig.owner,token,altered,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,Qualification,true}).status,
      fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(rig.Read(after));
  Exact(before,after);
  ASSERT_TRUE(rig.Prepare(token,view,output));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,token,output,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,Qualification,true})));
  EXPECT_EQ(rig.owner.accepted().epoch,2u);
}
} // namespace physical_publication_test
