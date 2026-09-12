// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/solvers/NodalCinStorage.h"
#include <gtest/gtest.h>

namespace tl::fea::cin_input_test {
TEST(CinForceInputs, CompleteForceHeaderMatchesSerialWrapperAndIndependentNodeOrder) {
  for (bool groups : {false, true}) {
    for (bool reverse : {false, true}) {
      Packet original(groups, true);
      Seed(original);
      for (unsigned step = 0; step < 3; ++step) {
        SCOPED_TRACE(step);
        Packet expected = original, serial = original, staged = original;
        const auto report = FrozenForce(expected);
        ASSERT_EQ(report.status, cin::StageStatus::Success);
        const auto view = serial.Input();
        SameReport(cin::PrepareForceTrial(view.model, inputs::ForceView(view)), report);
        SameReport(StagedForce(staged, reverse), report);
        SameForce(serial, expected);
        SameForce(staged, expected);
        original.accepted = expected.trial;
        original.Begin(step+2);
        Seed(original);
      }
    }
  }
}
TEST(CinForceInputs, FullCheckOrderAndUntouchedEntryInertiaOnEveryInputRejection) {
  using F = Fault;
  for (auto fault : {F::EarlyMass, F::BoundaryInertia, F::LastPosition,
      F::LastCouple, F::LateStiffness, F::NodeBeforeWitness, F::NodeBeforeNumerical,
      F::NumericalBeforeWitness, F::WitnessBeforeRow, F::WitnessAlias,
      F::LateRow, F::MissingActivity, F::NoRows}) {
    SCOPED_TRACE(int(fault));
    Packet initial(true, true);
    Seed(initial);
    Inject(initial, fault);
    Packet expected = initial, staged = initial;
    const auto report = FrozenForce(expected);
    ASSERT_NE(report.status, cin::StageStatus::Success);
    SameReport(StagedForce(staged, true), report);
    UnchangedBeforeTransfer(expected, initial);
    UnchangedBeforeTransfer(staged, initial);
    if (fault == F::BoundaryInertia) EXPECT_EQ(report.node, 127u);
    if (fault == F::NodeBeforeWitness || fault == F::NodeBeforeNumerical) EXPECT_EQ(report.node, 128u);
    if (fault == F::WitnessBeforeRow || fault == F::WitnessAlias) {
      EXPECT_EQ(report.status, cin::StageStatus::SourceMismatch);
    }
  }
}
TEST(CinForceInputs, OwnerPrefixAndSeparateInputKeyPreserveOrdinaryPhaseSentinel) {
  for (bool stale : {false, true}) {
    Packet packet;
    Seed(packet);
    const auto before = packet;
    Inject(packet, stale ? Fault::StaleBeforeNode : Fault::LimitBeforeNode);
    auto input = packet.Input();
    input.input_failure = &packet.input_failure;
    EXPECT_FALSE(inputs::Begin(input));
    EXPECT_EQ(packet.control.status, stale ? NodalStatus::StaleTrial : NodalStatus::MissingStepAdmission);
    EXPECT_EQ(packet.control.node, UINT32_MAX);
    EXPECT_EQ(packet.input_failure, before.input_failure);
    EXPECT_EQ(packet.failure, before.failure);
  }
  Packet packet;
  Seed(packet);
  auto input = packet.Input();
  input.input_failure = &packet.input_failure;
  ASSERT_TRUE(inputs::Begin(input));
  EXPECT_EQ(packet.input_failure, cin_advance::NoFailure);
  packet.input_failure = 128;
  EXPECT_FALSE(inputs::Complete(input));
  EXPECT_EQ(packet.control.node, 128u);
  EXPECT_EQ(packet.control.status, NodalStatus::InvalidOutput);
  EXPECT_NE(packet.failure, cin_advance::NoFailure);
  for (auto it = packet.work.begin()+2*Nodes; it != packet.work.begin()+3*Nodes; ++it) EXPECT_EQ(*it, -9876.25);
}
TEST(CinForceInputs, AdditionalKeyAndActualHeadersRespectExactCapsAndRetry) {
  NodalCinLimits limits;
  nodal_detail::CinLayout layout;
  ASSERT_TRUE(layout.Initialize(372435, 11165, 13173, limits, sizeof(nodal_detail::CinStorage)));
  EXPECT_EQ(layout.input_failure.offset, layout.failure.offset+sizeof(cin_advance::FailureKey));
  EXPECT_EQ(layout.input_failure.bytes, 8u);
  EXPECT_EQ(layout.input_failure.count, 1u);
  EXPECT_EQ(layout.screen.offset, layout.input_failure.offset+layout.input_failure.bytes);
  EXPECT_EQ(layout.group_reports.count, 0u);
  EXPECT_EQ(layout.prepared_transfers.offset, layout.screen.offset+layout.screen.bytes);
  EXPECT_EQ(layout.prepared_transfers.count, 11165u);
  EXPECT_EQ(layout.prepared_transfers.bytes, 5716480u);
  EXPECT_EQ(layout.prepared_recovery.offset, layout.prepared_transfers.offset+layout.prepared_transfers.bytes);
  EXPECT_EQ(layout.prepared_recovery.bytes, 1161160u);
  EXPECT_EQ(layout.recovery_failure.offset, layout.prepared_recovery.offset+layout.prepared_recovery.bytes);
  EXPECT_EQ(layout.recovery_failure.bytes, 4u);
  EXPECT_EQ(layout.device_bytes, layout.recovery_failure.offset+layout.recovery_failure.bytes);
  EXPECT_EQ(layout.scratch_values, 9u*372435);
  EXPECT_EQ(layout.state_values, 4u*372435+2u*11165+1);
  RecordProperty("full_optional_device_bytes", std::to_string(layout.optional_device_bytes));
  RecordProperty("full_host_bytes", std::to_string(layout.host_bytes));
  RecordProperty("storage_bytes", std::to_string(sizeof(nodal_detail::CinStorage)));
  limits.max_device_bytes = layout.optional_device_bytes;
  limits.max_host_bytes = layout.host_bytes;
  nodal_detail::CinLayout result;
  ASSERT_TRUE(result.Initialize(372435, 11165, 13173, limits, sizeof(nodal_detail::CinStorage)));
  const auto saved = result.device_bytes;
  --limits.max_device_bytes;
  EXPECT_FALSE(result.Initialize(372435, 11165, 13173, limits, sizeof(nodal_detail::CinStorage)));
  EXPECT_EQ(result.device_bytes, saved);
  ++limits.max_device_bytes;
  --limits.max_host_bytes;
  EXPECT_FALSE(result.Initialize(372435, 11165, 13173, limits, sizeof(nodal_detail::CinStorage)));
  EXPECT_EQ(result.device_bytes, saved);
  ++limits.max_host_bytes;
  EXPECT_TRUE(result.Initialize(372435, 11165, 13173, limits, sizeof(nodal_detail::CinStorage)));
}
} // namespace tl::fea::cin_input_test
