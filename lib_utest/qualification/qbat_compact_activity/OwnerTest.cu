// SPDX-License-Identifier: MIT
#include "OwnerProbe.h"
#include "../qbat_resident/ResidentFixture.h"
#include "../qbat_resident/ResultValues.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace qbat_activity_test {
namespace r=qbat_resident_test;namespace q=tl::fea::qbat;
TEST(QbatCompactActivityOwner, RealPreparedCacheCorruptionRejectsAtomicallyThenFreshRetryUsesTheRightSlab) {
  r::Source source(true);r::Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope()));
  const auto stamp=rig.owner.accepted();
  const auto count=rig.binding->qbat_count();
  std::vector<q::BatchResult> accepted(count),after(count);
  q::BatchDiagnostics initial;
  ASSERT_EQ(rig.qbat.CopyAcceptedResults(stamp,accepted.data(),count,&initial).status,q::BatchStatus::Success);
  r::Prepared candidate;ASSERT_TRUE(rig.Prepare(candidate,1));
  CaptureFullSource(count);
  std::vector<q::BatchResult> trial(count);
  ASSERT_EQ(rig.qbat.CopyPreparedResults(candidate.diagnostics.qbat,trial.data(),count).status,q::BatchStatus::Success);
  ASSERT_NE(FullSource(),nullptr);
  auto* device=static_cast<q::BatchResult*>(const_cast<void*>(FullSource()));
  auto corrupted=trial.back();
  corrupted.point[3].material.equivalent_stress_pa=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpy(device+count-1,&corrupted,sizeof(corrupted),cudaMemcpyHostToDevice),cudaSuccess);
  std::vector<std::uint8_t> flags(count,19);
  const auto rejected=rig.qbat.CopyPreparedParentActivity(rig.owner,candidate.token,candidate.diagnostics.qbat,flags.data(),count);
  EXPECT_EQ(rejected.status,q::BatchStatus::NonfiniteResult);EXPECT_EQ(rejected.element,count-1);
  EXPECT_EQ(flags,std::vector<std::uint8_t>(count,19));
  EXPECT_TRUE(tl::fea::trial_identity::SameStamp(rig.owner.accepted(),stamp));
  ASSERT_EQ(rig.qbat.CopyAcceptedResults(stamp,after.data(),count,&initial).status,q::BatchStatus::Success);
  for(std::size_t p=0;p<count;++p) EXPECT_EQ(r::ResultValues(accepted[p]),r::ResultValues(after[p]));
  EXPECT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner,candidate.token,candidate.diagnostics.qbat,flags.data(),count).status,
      q::BatchStatus::StaleTrial);
  rig.Discard();
  r::Prepared retry;ASSERT_TRUE(rig.Prepare(retry,1));
  CapturePacket(count);
  ASSERT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner,retry.token,retry.diagnostics.qbat,flags.data(),count).status,
      q::BatchStatus::Success);
  EXPECT_EQ(PacketBytes(),sizeof(std::uint32_t)+count);ASSERT_NE(PacketHost(),nullptr);
  for(std::size_t p=0;p<count;++p) EXPECT_EQ(flags[p],retry.qbat[p].history.element_active?1:0);
}
TEST(QbatCompactActivityOwner, CompactPrivateHostBytesCannotBeOutputForActivityOrFullSnapshots) {
  r::Source source;r::Rig rig;ASSERT_TRUE(rig.Initialize(source.Scope()));
  const auto stamp=rig.owner.accepted();const auto count=rig.binding->qbat_count();
  std::vector<std::uint8_t> flags(count,19);q::BatchDiagnostics diagnostics;
  CapturePacket(count);
  ASSERT_EQ(rig.qbat.CopyAcceptedParentActivity(stamp,flags.data(),count,&diagnostics).status,q::BatchStatus::Success);
  ASSERT_NE(PacketHost(),nullptr);
  auto* packet=static_cast<std::uint8_t*>(PacketHost());
  EXPECT_EQ(rig.qbat.CopyAcceptedParentActivity(stamp,packet,count,&diagnostics).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(rig.qbat.CopyAcceptedParentActivity(stamp,flags.data(),count,reinterpret_cast<q::BatchDiagnostics*>(packet)).status,
      q::BatchStatus::InvalidInput);
  EXPECT_EQ(rig.qbat.CopyAcceptedResults(stamp,reinterpret_cast<q::BatchResult*>(packet),count,&diagnostics).status,
      q::BatchStatus::InvalidInput);
  r::Prepared candidate;ASSERT_TRUE(rig.Prepare(candidate,1));
  EXPECT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner,candidate.token,candidate.diagnostics.qbat,packet,count).status,
      q::BatchStatus::InvalidInput);
  ASSERT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner,candidate.token,candidate.diagnostics.qbat,flags.data(),count).status,
      q::BatchStatus::Success);
}
TEST(QbatCompactActivityOwner, EachDeviceOperationFailureKeepsCallerOutputAndAcceptedEndpoint) {
  for(auto phase:{ErrorPhase::Memset,ErrorPhase::Launch,ErrorPhase::Copy,ErrorPhase::Drain}) {
    SCOPED_TRACE(static_cast<unsigned>(phase));
    r::Source source;r::Rig rig;ASSERT_TRUE(rig.Initialize(source.Scope()));
    const auto stamp=rig.owner.accepted();const auto count=rig.binding->qbat_count();
    std::vector<std::uint8_t> flags(count,19);q::BatchDiagnostics diagnostics;
    CapturePacket(count);
    ASSERT_EQ(rig.qbat.CopyAcceptedParentActivity(stamp,flags.data(),count,&diagnostics).status,q::BatchStatus::Success);
    r::Prepared candidate;ASSERT_TRUE(rig.Prepare(candidate,1));
    std::fill(flags.begin(),flags.end(),19);
    ASSERT_TRUE(ArmError(phase));
    EXPECT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner,candidate.token,candidate.diagnostics.qbat,flags.data(),count).status,
        q::BatchStatus::DeviceFailure);
    EXPECT_EQ(ErrorHits(),1u);EXPECT_EQ(flags,std::vector<std::uint8_t>(count,19));
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(rig.owner.accepted(),stamp));
    EXPECT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner,candidate.token,candidate.diagnostics.qbat,flags.data(),count).status,
        q::BatchStatus::DeviceFailure);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  }
}
} // namespace qbat_activity_test
