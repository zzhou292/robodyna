// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included inside the existing qualification namespace after Fixture,
// AssemblySnapshot and DiagnosticBits. No new owner or controller is added.
TEST(SelfContactForceCuda,
     LastActivityRowCorruptionPreservesAllChannelsReceiptAndSameAttemptRetry) {
  for (const bool empty : {false, true}) {
    SCOPED_TRACE(empty);
    Fixture f;
    ASSERT_TRUE(f.Initialize());
    auto base = f.activity, current = base;
    ASSERT_EQ(base.size(), 2u);
    const c::SelfContactActivityView activity{
        base.data(), current.data(), base.size()};
    auto events = empty ? std::vector<c::SelfContactForceEvent>{} : f.Events();
    if (!empty) ASSERT_FALSE(events.empty());
    for (auto& event : events) {
      c::SelfContactActiveUseReport report;
      if (event.feature.kind == c::FixedTriangleCandidateKind::VertexFace)
        report = f.uses.ClassifyVertexFace(
            event.vertex_use, event.facet_use, event.endpoints[1],
            activity, &event.classification);
      else
        report = f.uses.ClassifyEdgeEdge(
            event.edge_use[0], event.endpoints[0],
            event.edge_use[1], event.endpoints[1],
            event.classification.edge_edge_case, activity, &event.classification);
      ASSERT_EQ(report.status, c::SelfContactActiveUseStatus::Ok);
    }
    const c::SelfContactForceEventView batch{
        events.empty() ? nullptr : events.data(), events.size()};
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(f.rig.Begin(token, assembly));
    c::SelfContactForceAssemblyReceipt receipt;
    ASSERT_TRUE(Good(f.force.AssembleAccepted(
        f.rig.owner, token, assembly, activity, batch, &receipt)));
    const auto held = DiagnosticBits(receipt.diagnostics());
    const auto held_attempt = receipt.diagnostics().attempt;
    ASSERT_TRUE(receipt.prepared());
    f.Discard();

    ASSERT_TRUE(f.rig.Begin(token, assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(f.rig.owner.BorrowCinAssembly(token, &cin)));
    const auto owner_before = f.rig.owner.accepted();
    AssemblySnapshot before(f.rig.fixture.domain.node_count()), after(before.nodes);
    ASSERT_TRUE(before.Read(assembly, cin));
    for (const auto values : {std::array<std::uint8_t, 2>{2, 1},
                              std::array<std::uint8_t, 2>{1, 2},
                              std::array<std::uint8_t, 2>{0, 1}}) {
      base.back() = values[0];
      current.back() = values[1];
      const auto rejected = f.force.AssembleAccepted(
          f.rig.owner, token, assembly, activity, batch, &receipt);
      EXPECT_EQ(rejected.status, c::SelfContactForceStatus::StaleAttempt);
      EXPECT_EQ(rejected.event, SIZE_MAX);
      EXPECT_EQ(rejected.source_order, UINT64_MAX);
      EXPECT_EQ(rejected.node, UINT32_MAX);
      EXPECT_EQ(rejected.pair_status, c::SurfacePenaltyStatus::InvalidInput);
      EXPECT_EQ(rejected.owner_status, fe::NodalStatus::StaleTrial);
      EXPECT_STREQ(rejected.message, "Supplied activity source is stale or invalid");
      ASSERT_TRUE(after.Read(assembly, cin));
      EXPECT_EQ(after.values, before.values);
      EXPECT_TRUE(fe::trial_identity::SameStamp(owner_before, f.rig.owner.accepted()));
      EXPECT_EQ(DiagnosticBits(receipt.diagnostics()), held);
      EXPECT_EQ(receipt.diagnostics().attempt, held_attempt);
      EXPECT_TRUE(receipt.prepared());
      EXPECT_FALSE(f.force.Authenticates(receipt));
    }
    base.back() = current.back() = 1;
    ASSERT_TRUE(Good(f.force.AssembleAccepted(
        f.rig.owner, token, assembly, activity, batch, &receipt)));
    EXPECT_EQ(receipt.diagnostics().attempt, assembly.attempt);
    EXPECT_TRUE(f.force.Authenticates(receipt));
    ASSERT_TRUE(after.Read(assembly, cin));
    if (empty) EXPECT_EQ(after.values, before.values);
    f.Discard();
  }
}
