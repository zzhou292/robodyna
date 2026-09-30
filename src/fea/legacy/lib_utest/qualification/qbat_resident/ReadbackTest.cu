// SPDX-License-Identifier: MIT
#include "ResidentFixture.h"

namespace qbat_resident_test {
TEST(QbatResidentCuda, FullReadbackAliasesCapsLateCorruptionAndRetryAreAtomic) {
  MidlayerSource source;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope()));
  Fields initial;
  std::vector<qb::BatchResult> base;
  fe::ShellBatchDiagnostics initial_diagnostics;
  ASSERT_TRUE(rig.Accepted(initial,base,initial_diagnostics));
  auto* participant_alias=reinterpret_cast<fe::ShellBatchDiagnostics*>(&rig.qbat);
  EXPECT_EQ(rig.publication.CopyAcceptedDiagnostics(rig.owner.accepted(),participant_alias).status,
      fe::ShellPublicationStatus::InvalidInput);
  Prepared prepared;
  ASSERT_TRUE(rig.Prepare(prepared,1));
  qb::BatchResult output;
  const auto before=Bytes(output);
  EXPECT_EQ(rig.qbat.CopyPreparedResults(prepared.diagnostics.qbat,&output,0).status,qb::BatchStatus::ResourceLimit);
  EXPECT_EQ(rig.qbat.CopyPreparedResults(prepared.diagnostics.qbat,&output,SIZE_MAX).status,qb::BatchStatus::ResourceLimit);
  auto* alias=reinterpret_cast<qb::BatchResult*>(&prepared.diagnostics.qbat);
  EXPECT_EQ(rig.qbat.CopyPreparedResults(prepared.diagnostics.qbat,alias,1).status,qb::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(output),before);
  Arm(ReadFault::LateNonfinite,1);
  EXPECT_EQ(rig.qbat.CopyPreparedResults(prepared.diagnostics.qbat,&output,1).status,qb::BatchStatus::NonfiniteResult);
  EXPECT_EQ(Bytes(output),before);
  const auto& d=prepared.diagnostics.qbat;
  EXPECT_NE(rig.publication.Commit(rig.owner,prepared.token,prepared.diagnostics,
      {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true}).status,fe::ShellPublicationStatus::Success);
  Fields still;
  std::vector<qb::BatchResult> accepted;
  fe::ShellBatchDiagnostics diagnostics;
  ASSERT_TRUE(rig.Accepted(still,accepted,diagnostics));
  SameFields(initial,still);
  Exact(base,accepted);
  Prepared retry;
  ASSERT_TRUE(rig.Prepare(retry,1));
  Exact(prepared.qbat,retry.qbat);
  Arm(ReadFault::InvalidFlag,1);
  EXPECT_EQ(rig.qbat.CopyPreparedResults(retry.diagnostics.qbat,&output,1).status,qb::BatchStatus::NonfiniteResult);
  EXPECT_EQ(Bytes(output),before);
  rig.Discard();
  ASSERT_TRUE(rig.Prepare(retry,1));
  ASSERT_TRUE(rig.Commit(retry));
  EXPECT_EQ(rig.qbat.CopyPreparedResults(prepared.diagnostics.qbat,&output,1).status,qb::BatchStatus::StaleTrial);
}
TEST(QbatResidentCuda, CompletedCopyFailurePoisonsParticipantBeforeOwnerPublication) {
  MidlayerSource source(false);
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope()));
  Prepared prepared;
  ASSERT_TRUE(rig.Prepare(prepared,1));
  qb::BatchResult output;
  const auto before=Bytes(output);
  Arm(ReadFault::CopyError,1);
  EXPECT_EQ(rig.qbat.CopyPreparedResults(prepared.diagnostics.qbat,&output,1).status,qb::BatchStatus::DeviceFailure);
  EXPECT_EQ(Bytes(output),before);
  const auto& d=prepared.diagnostics.qbat;
  EXPECT_NE(rig.publication.Commit(rig.owner,prepared.token,prepared.diagnostics,
      {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true}).status,fe::ShellPublicationStatus::Success);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
}
TEST(QbatResidentCuda, StartupHostAndDeviceCapsRejectBeforeAllocationAndAllowRetry) {
  Source source;
  auto config=source.Config();
  qb::Batch participant;
  config.storage_limits.max_host_bytes=1;
  EXPECT_EQ(participant.InitializeFormulations(config,source.Scope()).status,qb::BatchStatus::ResourceLimit);
  EXPECT_EQ(participant.allocations().device_allocations,0u);
  config.storage_limits={};
  config.max_device_bytes=1;
  EXPECT_EQ(participant.InitializeFormulations(config,source.Scope()).status,qb::BatchStatus::ResourceLimit);
  EXPECT_EQ(participant.allocations().device_allocations,0u);
  config.max_device_bytes=1024*1024;
  ASSERT_EQ(participant.InitializeFormulations(config,source.Scope()).status,qb::BatchStatus::Success);
  EXPECT_EQ(participant.allocations().device_allocations,1u);
}
} // namespace qbat_resident_test
