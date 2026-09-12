// SPDX-License-Identifier: MIT
#include "OwnerSupport.h"

namespace t3_readback_test {
TEST(T3ActivityReadbackCuda,ActualOwnerUsesOneFreshForceCopyAtAcceptedAndPreparedEndpoints) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  for (unsigned step = 0; step < 6; ++step) {
    Oracle accepted;
    ASSERT_NO_FATAL_FAILURE(ReadOracle(rig, accepted));
    std::uint8_t flag = 19;
    t3::BatchDiagnostics diagnostics;
    Watch(1);
    ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(), &flag, 1, &diagnostics).status,
        t3::BatchStatus::Success);
    CheckTransfers();
    transfers.enabled = false;
    EXPECT_EQ(serial::Activity(accepted, 0, diagnostics.time, diagnostics.epoch).status,
        t3::BatchStatus::Success);
    EXPECT_EQ(flag, accepted.histories.sections[0].one_point()->point.failure.history.point_active);
    fe::NodalTrialToken token;
    fe::NodalPreparedView view;
    fe::ShellPhysicalDiagnostics candidate;
    ASSERT_TRUE(rig.Prepare(token, view, candidate));
    Oracle prepared;
    ASSERT_NO_FATAL_FAILURE(ReadOracle(rig, prepared, &candidate.t3));
    Watch(1);
    ASSERT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner, token, candidate.t3, &flag, 1).status,
        t3::BatchStatus::Success);
    CheckTransfers();
    transfers.enabled = false;
    EXPECT_EQ(serial::Activity(prepared, 1, candidate.t3.time, candidate.t3.epoch).status,
        t3::BatchStatus::Success);
    EXPECT_EQ(flag, prepared.histories.sections[0].one_point()->point.failure.history.point_active);
    if (!step) {
      Discard(rig);
      ASSERT_TRUE(rig.Prepare(token, view, candidate));
    }
    ASSERT_NO_FATAL_FAILURE(Commit(rig, token, view, candidate));
  }
}
TEST(T3ActivityReadbackCuda,FullTypedReadbackStillPerformsItsOwnFreshValidation) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  t3::ForceTrial result;
  t3::BatchDiagnostics diagnostics;
  Watch(1);
  ASSERT_EQ(rig.t3.CopyAcceptedResults(rig.owner.accepted(), &result, 1, &diagnostics).status,
      t3::BatchStatus::Success);
  EXPECT_EQ(transfers.force_copies, 1u);
  EXPECT_EQ(transfers.copies, 1u);
  fe::ShellBatchLayeredSection section;
  Watch(1);
  ASSERT_EQ(rig.t3.CopyAcceptedLayeredSectionHistory(rig.owner.accepted(), &section, 1, &diagnostics).status,
      t3::BatchStatus::Success);
  EXPECT_EQ(transfers.force_copies, 2u); // The unchanged full-history API's original schedule.
  EXPECT_EQ(transfers.copies, 6u);
  EXPECT_EQ(transfers.syncs, 5u);
  transfers.enabled = false;
}
} // namespace t3_readback_test
