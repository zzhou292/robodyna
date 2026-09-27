// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../cin_force_transfers/Fixture.h"
namespace tl::fea::cin_input_test {
TEST(CinPostNodeInputs, IndependentRowsAndWitnessesRetainOriginalPhaseAndIndexPriority) {
  for (unsigned fault = 0; fault < 6; ++fault) {
    SCOPED_TRACE(fault);
    auto input = cin_transfer_test::Population(129, true, true);
    if (fault == 0) { input.first_witness[128] = 127; input.activity[128] = 2; input.rows[0].masters[0] = Nodes; }
    if (fault == 1) { input.rows[127].masters[0] = Nodes; input.activity[1] = 0; }
    if (fault == 2) { input.rows[128].masters[3] = Nodes; input.rows[127].masters[1] = Nodes; }
    if (fault == 3) { input.trial.back() = NAN; input.first_witness[128] = 129; }
    if (fault == 4) { input.work[128] = NAN; input.trial.back() = NAN; }
    auto expected = input, actual = input;
    const auto report = FrozenForce(expected);
    SameReport(StagedForce(actual, true), report);
    SameForce(actual, expected);
    if (fault < 5) UnchangedBeforeTransfer(actual, input);
  }
}
TEST(CinPostNodeInputs, PhaseCompletionLeavesEntryInertiaUntouchedUntilAllChecksPass) {
  Packet packet; Seed(packet);
  auto input = packet.Input(); input.input_failure = &packet.input_failure;
  ASSERT_TRUE(inputs::Begin(input));
  ASSERT_TRUE(inputs::CompleteNodes(input));
  ASSERT_TRUE(inputs::CompleteWitnesses(input));
  ASSERT_TRUE(inputs::CompleteRows(input));
  for (auto it=packet.work.begin()+2*Nodes;it!=packet.work.begin()+3*Nodes;++it) EXPECT_EQ(*it,-9876.25);
  packet.input_failure=0;
  EXPECT_FALSE(inputs::CompleteWitnesses(input));
  EXPECT_EQ(packet.control.status,NodalStatus::InvalidOutput);
  EXPECT_EQ(packet.control.node,UINT32_MAX);
}
}
