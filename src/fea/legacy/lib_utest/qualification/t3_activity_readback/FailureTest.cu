// SPDX-License-Identifier: MIT
#include "OwnerSupport.h"

namespace t3_readback_test {
TEST(T3ActivityReadbackCuda,FrozenFailurePriorityOutputPreservationAndCandidateRetry) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto stamp = rig.owner.accepted();
  for (bool prepared : {false, true}) for (auto fault : {Fault::PointThickness, Fault::ForceNonfinite, Fault::Both}) {
    fe::NodalTrialToken token;
    fe::NodalPreparedView view;
    fe::ShellPhysicalDiagnostics candidate;
    if (prepared) ASSERT_TRUE(rig.Prepare(token, view, candidate));
    Oracle oracle;
    ASSERT_NO_FATAL_FAILURE(ReadOracle(rig, oracle, prepared ? &candidate.t3 : nullptr));
    if (fault == Fault::PointThickness || fault == Fault::Both) {
      auto point = *oracle.histories.sections[0].one_point();
      point.point.reported_thickness_m *= 2;
      oracle.histories.sections[0] = fe::ShellBatchLayeredSection::OnePoint(point);
    }
    if (fault == Fault::ForceNonfinite || fault == Fault::Both)
      oracle.staging[0].diagnostics.native_sound_speed = std::numeric_limits<double>::quiet_NaN();
    const auto expected = serial::Activity(oracle, prepared ? 1 : 0,
        prepared ? candidate.t3.time : stamp.time, prepared ? candidate.t3.epoch : stamp.epoch);
    ASSERT_EQ(expected.status, t3::BatchStatus::NonfiniteResult);
    std::uint8_t flag = 19;
    t3::BatchDiagnostics diagnostics;
    const auto before = Bytes(diagnostics);
    Watch(1, fault);
    const auto actual = prepared ? rig.t3.CopyPreparedParentActivity(rig.owner, token, candidate.t3, &flag, 1) :
        rig.t3.CopyAcceptedParentActivity(stamp, &flag, 1, &diagnostics);
    SameReport(actual, expected);
    CheckTransfers();
    transfers.enabled = false;
    EXPECT_EQ(flag, 19);
    EXPECT_EQ(Bytes(diagnostics), before);
    if (prepared) {
      EXPECT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner, token, candidate.t3, &flag, 1).status,
          t3::BatchStatus::StaleTrial);
      Discard(rig);
      ASSERT_TRUE(rig.Prepare(token, view, candidate));
      ASSERT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner, token, candidate.t3, &flag, 1).status,
          t3::BatchStatus::Success);
      Discard(rig);
    }
    ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(stamp, &flag, 1, &diagnostics).status, t3::BatchStatus::Success);
  }
}
TEST(T3ActivityReadbackCuda,RawPointEncodingRejectsBeforeForceAndFreshQueryRepairsStaging) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  std::uint8_t flag = 19;
  t3::BatchDiagnostics diagnostics;
  Watch(1, Fault::RawPointFlag);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(), &flag, 1, &diagnostics).status,
      t3::BatchStatus::NonfiniteResult);
  EXPECT_EQ(transfers.copies, 1u);
  EXPECT_EQ(transfers.force_copies, 0u);
  EXPECT_EQ(flag, 19);
  Watch(1);
  ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(), &flag, 1, &diagnostics).status,
      t3::BatchStatus::Success);
  CheckTransfers();
  transfers.enabled = false;
}
TEST(T3ActivityReadbackCuda,EveryRemainingCopyFailurePoisonsWithoutPublishing) {
  for (std::size_t failed = 1; failed <= 5; ++failed) {
    Rig rig;
    ASSERT_TRUE(rig.Initialize());
    std::uint8_t flag = 19;
    t3::BatchDiagnostics diagnostics;
    const auto before = Bytes(diagnostics);
    Watch(1);
    transfers.fail_copy = failed;
    EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(), &flag, 1, &diagnostics).status,
        t3::BatchStatus::DeviceFailure);
    EXPECT_EQ(transfers.copies, failed);
    transfers.enabled = false;
    EXPECT_EQ(flag, 19);
    EXPECT_EQ(Bytes(diagnostics), before);
    EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(), &flag, 1, &diagnostics).status,
        t3::BatchStatus::DeviceFailure);
  }
}
} // namespace t3_readback_test
