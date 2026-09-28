// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../type45_resident/OwnerFixture.h"
#include "lib_src/elements/publication/physical_activity/BatchAccess.h"
namespace physical_activity_test {
TEST(PhysicalActivityCuda, ExplicitNoJointSourceRejectsInventedModel) {
  Fixture f; ASSERT_TRUE(f.Initialize());
  EXPECT_TRUE(Good(f.snapshot.ValidateType45Source(nullptr)));
  fe::type45::Model unprepared;
  EXPECT_EQ(f.snapshot.ValidateType45Source(&unprepared).status, Status::SourceMismatch);
  ASSERT_TRUE(f.Begin()); fe::PhysicalActivityDeviceView view;
  ASSERT_TRUE(Good(f.snapshot.BorrowAccepted(f.rig.owner, f.token, f.assembly, f.accepted, &view)));
  EXPECT_EQ(view.type45_count, 0u); f.Discard();
}
TEST(PhysicalActivityCuda, MasslessJointSourceRequiresActualModelAndCompleteCount) {
  namespace j = type45_resident_test;
  j::Rig rig, foreign; ASSERT_TRUE(rig.Initialize()); ASSERT_TRUE(foreign.Initialize());
  auto& physical = rig.physical;
  fe::PhysicalActivitySnapshot snapshot;
  ASSERT_TRUE(Good(snapshot.Initialize(physical.owner, physical.publication, physical.fixture.physical,
      rig.Participants(), physical.fixture.Identity())));
  ASSERT_EQ(rig.model.joints().size(), foreign.model.joints().size());
  ASSERT_EQ(rig.model.source_instance_id(), foreign.model.source_instance_id());
  EXPECT_TRUE(Good(snapshot.ValidateType45Source(&rig.model)));
  fe::type45::Model alias(rig.model);
  EXPECT_TRUE(Good(snapshot.ValidateType45Source(&alias)));
  EXPECT_EQ(snapshot.ValidateType45Source(&foreign.model).status, Status::SourceMismatch);
  EXPECT_EQ(snapshot.ValidateType45Source(nullptr).status, Status::SourceMismatch);
  fe::ShellPhysicalDiagnostics diagnostics;
  ASSERT_TRUE(p::Good(physical.publication.CopyAcceptedPhysicalDiagnostics(physical.owner.accepted(), &diagnostics)));
  auto wrong = diagnostics.type45; ++wrong.joint_count;
  EXPECT_EQ(fe::physical_activity::BatchAccess::Type45(&rig.joints, &rig.model, true,
      physical.owner, physical.publication, physical.fixture.physical, true, wrong).status, Status::SourceMismatch);
  wrong = diagnostics.type45; ++wrong.source_instance_id;
  EXPECT_EQ(fe::physical_activity::BatchAccess::Type45(&rig.joints, &rig.model, true,
      physical.owner, physical.publication, physical.fixture.physical, true, wrong).status, Status::SourceMismatch);
  fe::NodalTrialToken token; fe::NodalAssemblyView assembly;
  ASSERT_TRUE(physical.Begin(token, assembly));
  ASSERT_TRUE(p::Good(rig.joints.AssembleAccepted(physical.owner, token, assembly)));
  fe::PhysicalAcceptedActivityReceipt accepted;
  ASSERT_TRUE(Good(snapshot.CaptureAccepted(physical.owner, token, assembly, &accepted)));
  fe::PhysicalActivityDeviceView view;
  ASSERT_TRUE(Good(snapshot.BorrowAccepted(physical.owner, token, assembly, accepted, &view)));
  EXPECT_EQ(view.type45_count, 3u);
  fe::NodalPreparedView prepared; fe::ShellPhysicalDiagnostics material, common;
  ASSERT_TRUE(physical.Advance(token, assembly, prepared)); ASSERT_TRUE(physical.Evaluate(token, prepared, material));
  ASSERT_TRUE(p::Good(rig.joints.EvaluateCandidate(physical.owner, token, prepared, &material.type45)));
  ASSERT_TRUE(p::Good(physical.publication.PreparePhysical(physical.owner, token, j::Candidates(material), &common)));
  fe::PhysicalPreparedActivityReceipt candidate;
  ASSERT_TRUE(Good(snapshot.CapturePrepared(physical.owner, token, common, prepared, accepted, &candidate)));
  ASSERT_TRUE(Good(snapshot.BorrowPrepared(physical.owner, token, common, prepared, candidate, &view)));
  EXPECT_EQ(view.type45_count, 3u);
  EXPECT_TRUE(Good(snapshot.ValidateType45Source(&rig.model)));
  physical.owner.Discard(); physical.publication.DiscardTrial(); snapshot.DiscardTrial();
}
} // namespace physical_activity_test
