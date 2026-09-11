#include "ResidentFixture.h"
#include "lib_src/elements/one_point/ShellOnePointArenaLayout.h"

namespace t3_one_point_resident_test {
TEST_F(Cuda, LateOnePointCopyCorruptionPreservesPublicOutputsAndAcceptedStateThenRetries) {
  for (const auto fault : {ReadFault::Nonfinite, ReadFault::InvalidFlag, ReadFault::ShellMismatch}) {
    Rig rig;
    fe::ShellBatchPlasticityBinding catalog;
    fe::ShellBatchFailureBinding failure;
    ASSERT_TRUE(Initialize(rig, catalog, failure));
    ASSERT_TRUE(rig.Bind());
    Prepared prepared;
    ASSERT_TRUE(Prepare(rig, 0, prepared));
    Frame expected;
    ASSERT_TRUE(mixed::Evaluate(rig, prepared, expected));
    Frame old;
    ASSERT_TRUE(mixed::ReadFrame(rig, old));
    temporal::Snapshot before;
    ASSERT_TRUE(temporal::Read(rig.owner, before));
    auto output = expected.tsection;
    const auto bytes = Bytes(output);
    Arm(fault);
    const auto report = rig.t3.CopyPreparedLayeredSectionHistory(expected.diagnostics.t3,
        output.data(), output.size());
    EXPECT_EQ(report.status, t3::BatchStatus::NonfiniteResult);
    EXPECT_GT(FaultCopies(), 0u);
    EXPECT_EQ(Bytes(output), bytes);
    EXPECT_EQ(rig.t3.CopyPreparedLayeredSectionHistory(expected.diagnostics.t3,
        output.data(), output.size()).status, t3::BatchStatus::StaleTrial);
    rig.Discard();
    Frame held;
    ASSERT_TRUE(mixed::ReadFrame(rig, held));
    Same(old, held);
    temporal::Snapshot after;
    ASSERT_TRUE(temporal::Read(rig.owner, after));
    temporal::SameState(before, after);
    Prepared retry;
    ASSERT_TRUE(Prepare(rig, 0, retry));
    Frame actual;
    ASSERT_TRUE(mixed::Evaluate(rig, retry, actual));
    Same(actual, expected);
    ASSERT_TRUE(mixed::Commit(rig, retry, actual));
  }
}
TEST_F(Cuda, CompletedOnePointCopyFailurePoisonsOnlyParticipantAndPreservesOwner) {
  Rig rig;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  ASSERT_TRUE(Initialize(rig, catalog, failure));
  ASSERT_TRUE(rig.Bind());
  Prepared prepared;
  ASSERT_TRUE(Prepare(rig, 0, prepared));
  Frame candidate;
  ASSERT_TRUE(mixed::Evaluate(rig, prepared, candidate));
  temporal::Snapshot before;
  ASSERT_TRUE(temporal::Read(rig.owner, before));
  auto output = candidate.tsection;
  const auto bytes = Bytes(output);
  Arm(ReadFault::DeviceError);
  EXPECT_EQ(rig.t3.CopyPreparedLayeredSectionHistory(candidate.diagnostics.t3,
      output.data(), output.size()).status, t3::BatchStatus::DeviceFailure);
  EXPECT_EQ(Bytes(output), bytes);
  rig.Discard();
  t3::BatchDiagnostics diagnostics;
  const auto dbytes = Bytes(diagnostics);
  EXPECT_EQ(rig.t3.CopyAcceptedLayeredSectionHistory(rig.owner.accepted(), output.data(),
      output.size(), &diagnostics).status, t3::BatchStatus::DeviceFailure);
  EXPECT_EQ(Bytes(output), bytes);
  EXPECT_EQ(Bytes(diagnostics), dbytes);
  temporal::Snapshot after;
  ASSERT_TRUE(temporal::Read(rig.owner, after));
  temporal::SameState(before, after);
}
TEST_F(Cuda, ExplicitRoleKeepsLegacyAllocationAndRejectsMissingFailureCapsStaleAndAliases) {
  Rig enabled, legacy;
  fe::ShellBatchPlasticityBinding catalog, old_catalog;
  fe::ShellBatchFailureBinding failure, old_failure;
  ASSERT_TRUE(Initialize(enabled, catalog, failure));
  ASSERT_TRUE(Initialize(legacy, old_catalog, old_failure, false));
  ASSERT_TRUE(enabled.Bind());
  ASSERT_TRUE(legacy.Bind());
  fe::shell_batch_plasticity_detail::OnePointLayout layout;
  ASSERT_TRUE(layout.Initialize(Parents, fe::MaxShellResidentDeviceBytes));
  EXPECT_EQ(enabled.t3.allocations().device_bytes - legacy.t3.allocations().device_bytes, layout.bytes);
  EXPECT_EQ(legacy.t3.allocations().device_allocations, 3u);
  EXPECT_EQ(enabled.t3.allocations().device_allocations, 4u);
  t3::T3BatchConfig config;
  config.owner = enabled.owner.accepted();
  config.configuration_id = mixed::Configuration;
  config.qualification_id = mixed::Qualification;
  config.element_count = Parents;
  config.usage = t3::BatchUsage::PrescribedFields;
  t3::T3Batch rejected;
  EXPECT_EQ(rejected.InitializeJoined(config, enabled.binding, catalog).status, t3::BatchStatus::InvalidInput);
  EXPECT_EQ(rejected.allocations().device_allocations, 0u);
  config.max_device_bytes = enabled.t3.allocations().device_bytes - 1;
  EXPECT_EQ(rejected.InitializeJoined(config, enabled.binding, catalog, failure, {}).status,
      t3::BatchStatus::ResourceLimit);
  EXPECT_EQ(rejected.allocations().device_allocations, 0u);
  config.max_device_bytes = enabled.t3.allocations().device_bytes;
  EXPECT_EQ(rejected.InitializeJoined(config, enabled.binding, catalog, failure, {}).status,
      t3::BatchStatus::Success);
  Prepared prepared;
  ASSERT_TRUE(Prepare(enabled, 0, prepared));
  Frame candidate;
  ASSERT_TRUE(mixed::Evaluate(enabled, prepared, candidate));
  auto output = candidate.tsection;
  const auto bytes = Bytes(output);
  EXPECT_EQ(enabled.t3.CopyPreparedLayeredSectionHistory(candidate.diagnostics.t3,
      output.data(), Parents-1).status, t3::BatchStatus::ResourceLimit);
  EXPECT_EQ(Bytes(output), bytes);
  auto diagnostic = candidate.diagnostics.t3;
  const auto dbytes = Bytes(diagnostic);
  EXPECT_EQ(enabled.t3.CopyPreparedLayeredSectionHistory(diagnostic,
      reinterpret_cast<fe::ShellBatchLayeredSection*>(&diagnostic), Parents).status, t3::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(diagnostic), dbytes);
  auto wrong = enabled.owner.accepted();
  ++wrong.owner_id;
  EXPECT_EQ(enabled.t3.CopyAcceptedLayeredSectionHistory(wrong, output.data(), Parents,
      &diagnostic).status, t3::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(output), bytes);
  std::array<fe::ShellBatchFailureState, Parents> unavailable;
  const auto unavailable_bytes = Bytes(unavailable);
  EXPECT_EQ(enabled.t3.CopyPreparedFailureHistory(candidate.diagnostics.t3,
      unavailable.data(), Parents).status, t3::BatchStatus::InvalidInput);
  EXPECT_EQ(enabled.t3.CopyAcceptedFailureHistory(enabled.owner.accepted(),
      unavailable.data(), Parents, &diagnostic).status, t3::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(unavailable), unavailable_bytes);
  ASSERT_TRUE(mixed::Commit(enabled, prepared, candidate));
  EXPECT_EQ(enabled.t3.CopyPreparedLayeredSectionHistory(candidate.diagnostics.t3,
      output.data(), Parents).status, t3::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(output), bytes);
}
} // namespace t3_one_point_resident_test
