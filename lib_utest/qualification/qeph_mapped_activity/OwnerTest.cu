// SPDX-License-Identifier: MIT
#include "OwnerSupport.h"

namespace qeph_activity_test {
TEST(QephMappedActivityCuda,ActualCinOwnerKeepsFrozenAcceptedPreparedAndCompleteReadbacks) {
  for (bool skin : {false,true}) {
    Rig rig(skin);
    ASSERT_TRUE(rig.Initialize());
    const auto allocation = rig.qeph.allocations();
    for (unsigned step = 0; step < 6; ++step) {
      fe::NodalTrialToken token;
      fe::NodalPreparedView view;
      fe::ShellPhysicalDiagnostics candidate;
      ASSERT_TRUE(rig.Prepare(token,view,candidate));
      Oracle accepted, prepared;
      ASSERT_NO_FATAL_FAILURE(ReadOracle(rig,accepted));
      ASSERT_NO_FATAL_FAILURE(ReadOracle(rig,prepared,&candidate.qeph));
      ASSERT_EQ(serial::ValidateMappedSections(accepted,0).status,q::BatchStatus::Success);
      ASSERT_EQ(serial::ValidateMappedSections(prepared,1).status,q::BatchStatus::Success);
      std::vector<std::uint8_t> flags(accepted.staging.size(),19);
      q::BatchDiagnostics diagnostics;
      Watch(flags.size());
      ASSERT_EQ(rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),flags.data(),flags.size(),&diagnostics).status,
          q::BatchStatus::Success);
      CheckTransfer();
      transfers.enabled = false;
      CheckFlags(flags,accepted);
      EXPECT_EQ(diagnostics.epoch,step);
      for (unsigned repeat = 0; repeat < 2; ++repeat) {
        Watch(flags.size());
        ASSERT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),flags.size()).status,
            q::BatchStatus::Success);
        CheckTransfer();
        transfers.enabled = false;
        CheckFlags(flags,prepared);
      }
      Oracle repeated;
      ASSERT_NO_FATAL_FAILURE(ReadOracle(rig,repeated,&candidate.qeph));
      EXPECT_EQ(qt_mapped_test::Values(repeated.staging),qt_mapped_test::Values(prepared.staging));
      if (step == 0) {
        Discard(rig);
        ASSERT_TRUE(rig.Prepare(token,view,candidate));
      }
      ASSERT_TRUE(Commit(rig,token,view,candidate));
      EXPECT_EQ(rig.qeph.allocations().device_bytes,allocation.device_bytes);
      EXPECT_EQ(rig.qeph.allocations().device_allocations,allocation.device_allocations);
    }
  }
}

TEST(QephMappedActivityCuda,FreshCompactBufferAliasesAndStaleInputsNeverWriteOutputs) {
  Rig rig(true);
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;
  fe::NodalPreparedView view;
  fe::ShellPhysicalDiagnostics candidate;
  ASSERT_TRUE(rig.Prepare(token,view,candidate));
  const auto n = rig.fixture.physical.shells()->qeph_count();
  std::vector<std::uint8_t> flags(n,19);
  Watch(n);
  ASSERT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),n).status,
      q::BatchStatus::Success);
  transfers.enabled = false;
  auto* compact = static_cast<std::uint8_t*>(transfers.compact_destination);
  ASSERT_NE(compact,nullptr);
  EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,compact,n).status,
      q::BatchStatus::InvalidInput);
  std::fill(flags.begin(),flags.end(),19);
  const auto before = flags;
  auto stale = candidate.qeph;
  ++stale.attempt;
  EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,stale,flags.data(),n).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,flags.data(),SIZE_MAX).status,
      q::BatchStatus::ResourceLimit);
  fe::FENodalState foreign;
  EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(foreign,token,candidate.qeph,flags.data(),n).status,
      q::BatchStatus::StaleTrial);
  EXPECT_EQ(flags,before);
  auto* source = reinterpret_cast<std::uint8_t*>(const_cast<fe::NodalCoefficientNode*>(rig.fixture.ledger.nodes().data()));
  EXPECT_EQ(rig.qeph.CopyPreparedParentActivity(rig.owner,token,candidate.qeph,source,n).status,
      q::BatchStatus::InvalidInput);
  q::BatchDiagnostics diagnostics;
  const auto untouched = qt_mapped_test::Bytes(diagnostics);
  EXPECT_EQ(rig.qeph.CopyAcceptedParentActivity(rig.owner.accepted(),compact,n,&diagnostics).status,
      q::BatchStatus::InvalidInput);
  EXPECT_EQ(qt_mapped_test::Bytes(diagnostics),untouched);
  Discard(rig);
}
} // namespace qeph_activity_test
