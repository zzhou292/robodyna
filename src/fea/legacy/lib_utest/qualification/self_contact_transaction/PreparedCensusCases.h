// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included after CudaTest.cu's shared owner/publication Fixture. These are
// diagnostic-authority tests; no fixture census is a physical commit receipt.
namespace self_contact_transaction_cuda_test {

struct PreparedCensusOutputs {
  std::size_t nonlinear_count = SIZE_MAX;
  std::size_t linear_count = SIZE_MAX;
  sct::NonlinearCandidateRosterSummary nonlinear;
  sct::LinearCandidateCensusSummary linear;
  sct::PreparedMotionCertificateView motion;
  sct::QualificationPreparedCensusReceipt receipt;

  c::SelfContactTransactionReport Capture(
      Fixture& fixture, const fe::NodalTrialToken& token,
      const fe::ShellPhysicalDiagnostics& common,
      const fe::NodalPreparedView& prepared,
      const c::SelfContactAcceptedAssemblyReceipt& accepted) {
    return sct::QualificationAccess::ClassifyPreparedCandidateCensus(
        fixture.transaction, fixture.rig.owner, token, common,
        prepared, accepted, nullptr, 0, &nonlinear_count, &nonlinear,
        nullptr, 0, &linear_count, &linear, &motion, &receipt);
  }
};

TEST(SelfContactTransactionCuda,
     PreparedCensusUsesActualRemovalAndPreservesAcceptedPolicyUntilDiscard) {
  Fixture fixture(false, false, 1.e-12);
  ASSERT_TRUE(fixture.DriveT3Removal());
  ASSERT_TRUE(fixture.Initialize());
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  const auto owners = sct::QualificationAccess::AcceptedCertificates(
      fixture.transaction);
  ASSERT_TRUE(owners.complete);
  ASSERT_GT(owners.count, 0u);
  const auto feature = owners.data[0].discovery;
  const c::FixedTriangleFeatureView features{&feature, 1, true};
  sct::AcceptedFeaturePolicyEvidence original;
  std::size_t policy_count = 0;
  ASSERT_TRUE(Good(sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
      fixture.transaction, accepted, features, &original, 1, &policy_count)));
  ASSERT_EQ(policy_count, 1u);

  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  PreparedCensusOutputs census;
  ASSERT_TRUE(Good(census.Capture(fixture, token, common, prepared, accepted)));
  ASSERT_TRUE(census.receipt.valid());
  EXPECT_FALSE(accepted.valid());
  EXPECT_EQ(census.nonlinear_count, 0u);
  EXPECT_EQ(census.linear_count, 0u);
  EXPECT_TRUE(census.motion.complete);
  const auto activity = census.receipt.activity_summary();
  EXPECT_TRUE(activity.complete);
  EXPECT_EQ(activity.selected, 2u);
  EXPECT_EQ(activity.accepted_active, 2u);
  EXPECT_EQ(activity.prepared_active, 1u);
  EXPECT_EQ(activity.removing, 1u);
  EXPECT_EQ(activity.inactive, 0u);
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  for (unsigned repetition = 0; repetition < 2; ++repetition) {
    sct::AcceptedFeaturePolicyEvidence replay;
    ASSERT_TRUE(Good(sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
        fixture.transaction, census.receipt, features,
        &replay, 1, &policy_count)));
    EXPECT_EQ(policy_count, 1u);
    EXPECT_EQ(replay.disposition, original.disposition);
    EXPECT_EQ(replay.report_status, original.report_status);
    EXPECT_EQ(replay.pair_status, original.pair_status);
    EXPECT_EQ(replay.admitted, original.admitted);
    EXPECT_EQ(replay.active[0], original.active[0]);
    EXPECT_EQ(replay.active[1], original.active[1]);
    EXPECT_EQ(replay.ledger_key_matches, original.ledger_key_matches);
    EXPECT_EQ(replay.first_ledger_source_order, original.first_ledger_source_order);
    EXPECT_TRUE(census.receipt.valid());
  }
  fixture.Discard();
  EXPECT_FALSE(census.receipt.valid());
  EXPECT_FALSE(census.receipt.activity_summary().complete);
  sct::AcceptedFeaturePolicyEvidence stale;
  EXPECT_EQ(sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
      fixture.transaction, census.receipt, features, &stale, 1,
      &policy_count).status, c::SelfContactTransactionStatus::IdentityMismatch);
  EXPECT_EQ(policy_count, 0u);
  ASSERT_TRUE(fixture.rig.Read(after));
  p::Exact(before, after);
}

TEST(SelfContactTransactionCuda,
     PreparedCensusRejectsForgedCommonDiagnosticsWithoutPublication) {
  Fixture fixture(true);
  ASSERT_TRUE(fixture.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  auto forged = common;
  ++forged.t3.attempt;
  PreparedCensusOutputs census;
  EXPECT_EQ(census.Capture(fixture, token, forged, prepared, accepted).status,
            c::SelfContactTransactionStatus::ActivityFailure);
  EXPECT_FALSE(census.receipt.valid());
  EXPECT_FALSE(census.motion.complete);
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  fixture.Discard();
}

TEST(SelfContactTransactionCuda,
     PreparedCensusAuthenticatesSecondIntervalAfterRealCommit) {
  Fixture fixture(true);
  ASSERT_TRUE(fixture.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  c::SelfContactAcceptedAssemblyReceipt accepted;
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  c::SelfContactTransactionReceipt committed;
  ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, accepted, &committed)));
  ASSERT_TRUE(fixture.Commit(token, prepared, common, committed));
  ASSERT_EQ(fixture.rig.owner.accepted().epoch, 1u);
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  EXPECT_EQ(prepared.kinematics.base_epoch, 1u);
  PreparedCensusOutputs census;
  ASSERT_TRUE(Good(census.Capture(fixture, token, common, prepared, accepted)));
  EXPECT_TRUE(census.receipt.valid());
  const auto activity = census.receipt.activity_summary();
  EXPECT_TRUE(activity.complete);
  EXPECT_EQ(activity.selected, 1u);
  EXPECT_EQ(activity.accepted_active, 1u);
  EXPECT_EQ(activity.prepared_active, 1u);
  EXPECT_EQ(activity.removing, 0u);
  fixture.Discard();
  EXPECT_FALSE(census.receipt.valid());
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);
}

TEST(SelfContactTransactionCuda,
     PreparedCensusRejectsAliasedAndOverflowedOutputsBeforeAnyWrite) {
  Fixture fixture(true);
  ASSERT_TRUE(fixture.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  PreparedCensusOutputs output;
  sct::NonlinearCandidateRosterEntry row;
  const auto capture = [&](sct::NonlinearCandidateRosterEntry* roster,
                           std::size_t capacity, std::size_t* nonlinear_count,
                           std::size_t* linear_count) {
    return sct::QualificationAccess::ClassifyPreparedCandidateCensus(
        fixture.transaction, fixture.rig.owner, token, common,
        prepared, accepted, roster, capacity, nonlinear_count,
        &output.nonlinear, nullptr, 0, linear_count, &output.linear,
        &output.motion, &output.receipt);
  };
  EXPECT_EQ(capture(nullptr, 0, &output.nonlinear_count,
                    &output.nonlinear_count).status,
            c::SelfContactTransactionStatus::InvalidInput);
  EXPECT_EQ(output.nonlinear_count, SIZE_MAX);
  EXPECT_EQ(output.linear_count, SIZE_MAX);
  EXPECT_TRUE(accepted.valid());
  const auto original_attempt = common.t3.attempt;
  EXPECT_EQ(capture(nullptr, 0,
                    reinterpret_cast<std::size_t*>(&common.t3.attempt),
                    &output.linear_count).status,
            c::SelfContactTransactionStatus::InvalidInput);
  EXPECT_EQ(common.t3.attempt, original_attempt);
  EXPECT_EQ(output.linear_count, SIZE_MAX);
  EXPECT_TRUE(accepted.valid());
  EXPECT_EQ(capture(nullptr, 0,
                    reinterpret_cast<std::size_t*>(&output.receipt),
                    &output.linear_count).status,
            c::SelfContactTransactionStatus::InvalidInput);
  EXPECT_FALSE(output.receipt.valid());
  EXPECT_TRUE(accepted.valid());
  EXPECT_EQ(capture(&row, SIZE_MAX / sizeof(row) + 1,
                    &output.nonlinear_count, &output.linear_count).status,
            c::SelfContactTransactionStatus::InvalidInput);
  EXPECT_EQ(output.nonlinear_count, SIZE_MAX);
  EXPECT_EQ(output.linear_count, SIZE_MAX);
  EXPECT_TRUE(accepted.valid());
  ASSERT_TRUE(Good(output.Capture(fixture, token, common, prepared, accepted)));
  EXPECT_TRUE(output.receipt.valid());
  fixture.Discard();
}

TEST(SelfContactTransactionCuda,
     PreparedCensusCannotSealAndOldCopiesStayInvalidOnRetry) {
  Fixture fixture(true);
  ASSERT_TRUE(fixture.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  c::SelfContactAcceptedAssemblyReceipt accepted;
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
  PreparedCensusOutputs census;
  ASSERT_TRUE(Good(census.Capture(fixture, token, common, prepared, accepted)));
  const auto old = census.receipt;
  c::SelfContactTransactionReceipt publication;
  EXPECT_NE(fixture.transaction.SealCandidate(
      fixture.rig.owner, token, common, prepared, accepted,
      &publication).status, c::SelfContactTransactionStatus::Ok);
  EXPECT_FALSE(publication.valid());
  EXPECT_FALSE(old.valid());
  EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
  fixture.Discard();
  ASSERT_TRUE(fixture.rig.Begin(token, assembly));
  ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
      fixture.rig.owner, token, assembly, &accepted)));
  EXPECT_FALSE(old.valid());
  std::size_t count = 1;
  EXPECT_EQ(sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
      fixture.transaction, old, {nullptr, 0, true}, nullptr, 0,
      &count).status, c::SelfContactTransactionStatus::IdentityMismatch);
  EXPECT_EQ(count, 0u);
  const sct::QualificationPreparedCensusReceipt empty;
  EXPECT_EQ(sct::QualificationAccess::ClassifyAcceptedFeaturePolicies(
      fixture.transaction, empty, {nullptr, 0, true}, nullptr, 0,
      &count).status, c::SelfContactTransactionStatus::IdentityMismatch);
  fixture.Discard();
}

}  // namespace self_contact_transaction_cuda_test
