// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included after the owning CUDA fixture and its bitwise observation helpers.
namespace self_contact_transaction_cuda_test {
namespace {
struct FailingDiagnosticClock {
  std::uint64_t calls = 0;
  static bool Read(void* context, std::uint64_t*) noexcept {
    ++static_cast<FailingDiagnosticClock*>(context)->calls;
    errno = EIO;
    return false;
  }
  sct::DiagnosticClock reader() { return {Read, this}; }
};
void CheckSnapshot(const c::SelfContactAttemptDiagnostics& snapshot,
                   const fe::NodalAssemblyView& assembly, bool failed_clock) {
  EXPECT_TRUE(snapshot.enabled && snapshot.entered && snapshot.finished);
  EXPECT_TRUE(snapshot.succeeded && snapshot.authenticated && snapshot.counts_complete);
  EXPECT_FALSE(snapshot.counter_saturated);
  EXPECT_EQ(snapshot.owner_id, assembly.owner_id);
  EXPECT_EQ(snapshot.base_epoch, assembly.accepted.base_epoch);
  EXPECT_EQ(snapshot.attempt, assembly.attempt);
  EXPECT_EQ(snapshot.clock_failures > 0, failed_clock);
  for (const auto& stage : snapshot.stages) {
    EXPECT_EQ(stage.failures, 0u);
    EXPECT_EQ(stage.valid_samples, failed_clock ? 0u : stage.calls);
    if (failed_clock) EXPECT_EQ(stage.wall_ns, 0u);
  }
}
}

TEST(SelfContactTransactionCuda, DiagnosticsPreserveForcesReceiptsPolicyAndCommittedState) {
  DeterminismObservation reference;
  for (unsigned mode = 0; mode < 3; ++mode) {
    SCOPED_TRACE(mode); // disabled / enabled monotonic / enabled failed clock
    FailingDiagnosticClock clock;
    Fixture fixture(false, false, 2.5, p::ContactConstraintLayout::Legacy, 1);
    fixture.config.enable_diagnostics = mode != 0;
    c::SelfContactTransactionLimits limits;
    limits.max_global_events = 512;
    limits.max_event_identity_census = 512; // Direct ledger: one discovery pass.
    ASSERT_TRUE(fixture.Initialize(limits));
    if (mode != 1)
      sct::QualificationAccess::SetDiagnosticClock(fixture.transaction, clock.reader());
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token, &cin)));
    AssemblyFields fields(fixture.rig.fixture.domain.node_count());
    ASSERT_TRUE(fields.Read(assembly, cin));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    c::SelfContactTransactionReceipt receipt;
    ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &receipt)));
    const auto diagnostics = fixture.transaction.diagnostics();
    if (mode == 0) {
      EXPECT_FALSE(diagnostics.accepted.enabled || diagnostics.candidate.enabled);
      EXPECT_EQ(clock.calls, 0u);
    } else {
      CheckSnapshot(diagnostics.accepted, assembly, mode == 2);
      CheckSnapshot(diagnostics.candidate, assembly, mode == 2);
      EXPECT_GT(diagnostics.accepted.discovery.calls, 0u);
      EXPECT_EQ(diagnostics.accepted.discovery.feature_candidates,
                accepted.discovered_features());
      EXPECT_LE(diagnostics.candidate.native_submitted_pairs,
                receipt.policy_summary().exact_crossing_pairs);
      EXPECT_EQ(diagnostics.candidate.native_work +
                    receipt.policy_summary().linear_policy_coverage_work,
                receipt.policy_summary().exact_crossing_work);
    }
    DeterminismObservation actual;
    actual.fields = FieldBits(fields);
    actual.accepted_receipt = AcceptedReceiptBits(accepted);
    actual.transaction_receipt = TransactionReceiptBits(receipt);
    actual.policy_summary = PolicySummaryBits(fixture.transaction.policy_summary());
    actual.policy_outcomes = PolicyOutcomeBits(
        fixture.transaction.policy_outcomes(), &actual.canonical_event_order);
    ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
    p::Snapshot final;
    ASSERT_TRUE(fixture.rig.Read(final));
    actual.accepted_state = final.values;
    actual.final_stamp = StampBits(final.stamp);
    if (!mode) reference = actual;
    else {
      EXPECT_EQ(actual.fields, reference.fields);
      EXPECT_EQ(actual.accepted_receipt, reference.accepted_receipt);
      EXPECT_EQ(actual.transaction_receipt, reference.transaction_receipt);
      EXPECT_EQ(actual.policy_summary, reference.policy_summary);
      EXPECT_EQ(actual.policy_outcomes, reference.policy_outcomes);
      EXPECT_EQ(actual.canonical_event_order, reference.canonical_event_order);
      EXPECT_EQ(actual.accepted_state, reference.accepted_state);
      EXPECT_EQ(actual.final_stamp, reference.final_stamp);
    }
  }
}

TEST(SelfContactTransactionCuda, DiagnosticFailureSurvivesRollbackAndResetsOnRetry) {
  std::size_t reference_pair = SIZE_MAX;
  for (unsigned mode = 0; mode < 2; ++mode) {
    Fixture fixture(false, true);
    fixture.config.enable_diagnostics = mode != 0;
    c::SelfContactTransactionLimits limits;
    limits.crossing.max_depth = 4;
    limits.crossing.max_work_per_pair = 31;
    limits.crossing.max_total_work = 31 * 64;
    ASSERT_TRUE(fixture.Initialize(limits));
    FailingDiagnosticClock clock;
    sct::QualificationAccess::SetDiagnosticClock(fixture.transaction, clock.reader());
    p::Snapshot before, after;
    ASSERT_TRUE(fixture.rig.Read(before));
    std::uint64_t prior_attempt = 0;
    for (unsigned retry = 0; retry < 2; ++retry) {
      fe::NodalTrialToken token;
      fe::NodalAssemblyView assembly;
      ASSERT_TRUE(fixture.rig.Begin(token, assembly));
      c::SelfContactAcceptedAssemblyReceipt accepted;
      ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
          fixture.rig.owner, token, assembly, &accepted)));
      EXPECT_FALSE(fixture.transaction.diagnostics().candidate.entered);
      fe::NodalPreparedView prepared;
      fe::ShellPhysicalDiagnostics common;
      ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
      c::SelfContactTransactionReceipt unchanged;
      const auto report = fixture.transaction.SealCandidate(
          fixture.rig.owner, token, common, prepared, accepted, &unchanged);
      EXPECT_EQ(report.status, c::SelfContactTransactionStatus::CandidateRejected);
      EXPECT_EQ(report.crossing_reason, c::RepresentedIntervalReason::None);
      EXPECT_FALSE(unchanged.valid());
      if (!mode && !retry) reference_pair = report.pair;
      EXPECT_EQ(report.pair, reference_pair);
      const auto diagnostic = fixture.transaction.diagnostics().candidate;
      if (mode) {
        EXPECT_TRUE(diagnostic.enabled && diagnostic.entered && diagnostic.finished);
        EXPECT_TRUE(diagnostic.authenticated);
        EXPECT_FALSE(diagnostic.succeeded || diagnostic.counts_complete);
        EXPECT_EQ(diagnostic.attempt, assembly.attempt);
        EXPECT_GT(diagnostic.attempt, prior_attempt);
        EXPECT_GT(diagnostic.clock_failures, 0u);
        prior_attempt = diagnostic.attempt;
        fixture.transaction.DiscardTrial();
        const auto retained = fixture.transaction.diagnostics().candidate;
        EXPECT_EQ(retained.attempt, diagnostic.attempt);
        EXPECT_EQ(retained.clock_failures, diagnostic.clock_failures);
        EXPECT_EQ(retained.discovery.calls, diagnostic.discovery.calls);
        EXPECT_TRUE(retained.finished);
      } else EXPECT_EQ(clock.calls, 0u);
      ASSERT_TRUE(fixture.rig.Read(after));
      p::Exact(before, after);
    }
  }
}
}  // namespace self_contact_transaction_cuda_test
