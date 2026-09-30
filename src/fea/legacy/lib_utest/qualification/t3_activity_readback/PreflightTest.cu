// SPDX-License-Identifier: MIT
#include "OwnerSupport.h"

namespace t3_readback_test {
TEST(T3ActivityReadbackCuda,StaleForeignCapacityNullAndOwnedStagingAliasesFailBeforeRead) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  std::uint8_t flag = 19;
  t3::BatchDiagnostics diagnostics;
  Watch(1);
  ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(), &flag, 1, &diagnostics).status,
      t3::BatchStatus::Success);
  auto* alias = static_cast<std::uint8_t*>(transfers.force_host);
  ASSERT_NE(alias, nullptr);
  const auto original = *alias;
  const auto stamp = rig.owner.accepted();
  auto stale = stamp;
  ++stale.epoch;
  Watch(1);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stamp, alias, 1, &diagnostics).status,
      t3::BatchStatus::InvalidInput);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stale, &flag, 1, &diagnostics).status,
      t3::BatchStatus::StaleTrial);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stamp, nullptr, 1, &diagnostics).status,
      t3::BatchStatus::InvalidInput);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stamp, &flag, 1, nullptr).status,
      t3::BatchStatus::InvalidInput);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stamp, &flag, SIZE_MAX, &diagnostics).status,
      t3::BatchStatus::ResourceLimit);
  EXPECT_EQ(transfers.copies, 0u);
  EXPECT_EQ(*alias, original);
  transfers.enabled = false;
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  fe::ShellPhysicalDiagnostics candidate;
  ASSERT_TRUE(rig.Prepare(token, view, candidate));
  Watch(1);
  fe::FENodalState foreign;
  EXPECT_NE(rig.t3.CopyPreparedParentActivity(foreign, token, candidate.t3, &flag, 1).status,
      t3::BatchStatus::Success);
  EXPECT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner, token, candidate.t3, alias, 1).status,
      t3::BatchStatus::InvalidInput);
  EXPECT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner, token, candidate.t3, &flag, SIZE_MAX).status,
      t3::BatchStatus::ResourceLimit);
  EXPECT_EQ(transfers.copies, 0u);
  transfers.enabled = false;
  Discard(rig);
}
} // namespace t3_readback_test
