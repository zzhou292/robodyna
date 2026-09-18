// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

namespace self_contact_transaction_cuda_test {

std::vector<std::uint64_t> CompleteBatchPolicyBits(
    const c::SelfContactCandidatePolicySummary& value) {
  auto result = PolicySummaryBits(value);
  result.insert(result.end(), {
      value.motion_certified_nonlinear_separated,
      value.nonlinear_subdivision_pairs, value.nonlinear_subdivision_work,
      value.nonlinear_subdivision_unresolved,
      value.nonlinear_subdivision_work_exhausted,
      value.nonlinear_subdivision_depth_exhausted,
      value.linear_policy_coverage_pairs, value.linear_policy_coverage_work,
      value.linear_policy_certified_separated,
      value.linear_policy_accepted_coverage,
      value.linear_policy_exact_exclusion,
      value.linear_policy_potential_contact,
      value.linear_policy_work_exhausted, value.linear_policy_depth_exhausted,
      value.linear_policy_missing_accepted_owner,
      value.linear_policy_owner_ambiguity,
      value.linear_policy_possible_geometric_crossing,
      value.linear_policy_unresolved});
  return result;
}

TEST(SelfContactTransactionCuda,
     RawBatchSizePreservesDiscoveryForcePolicyAndOwnerAcrossDiscardRetry) {
  // Both source geometries and their native expected behavior are separately
  // qualified by TranslatedLocalCudaCases. Vary only the raw per-call budget;
  // discovery chunk, source/facet ordering, material and physical step stay put.
  for (const bool positive_force : {false, true}) {
    SCOPED_TRACE(positive_force);
    std::vector<std::uint64_t> expected_fields, expected_accepted,
        expected_outcomes, expected_summary, expected_final;
    for (const std::size_t batch_size : {1u, 2u, 64u}) {
      SCOPED_TRACE(batch_size);
      const double apex = positive_force
          ? std::nextafter(.04 + .0005, .04) : .05;
      Fixture fixture(true, false, 2.5, p::ContactConstraintLayout::Legacy,
                      0, 0, apex, {}, positive_force);
      fixture.single_parent = false;
      c::SelfContactTransactionLimits limits;
      limits.crossing.max_total_work =
          batch_size * limits.crossing.max_work_per_pair;
      ASSERT_TRUE(fixture.Initialize(limits));
      const auto forecast = fixture.transaction.forecast();
      EXPECT_EQ(forecast.crossing_batch_pair_capacity, batch_size);
      EXPECT_EQ(forecast.facet_pair_chunk_capacity, limits.max_facet_pair_chunk);
      EXPECT_EQ(forecast.raw_crossing_result_capacity, limits.max_facet_pair_chunk);
      EXPECT_EQ(forecast.raw_crossing_result_bytes,
                sizeof(c::RepresentedIntervalResult) * limits.max_facet_pair_chunk);
      const auto allocations = fixture.transaction.allocations();
      p::Snapshot initial, discarded;
      ASSERT_TRUE(fixture.rig.Read(initial));
      for (unsigned retry = 0; retry < 2; ++retry) {
        fe::NodalTrialToken token;
        fe::NodalAssemblyView assembly;
        ASSERT_TRUE(fixture.rig.Begin(token, assembly));
        fe::NodalCinAssemblyView cin;
        ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token, &cin)));
        AssemblyFields fields(fixture.rig.fixture.domain.node_count());
        c::SelfContactAcceptedAssemblyReceipt accepted;
        ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
            fixture.rig.owner, token, assembly, &accepted)));
        ASSERT_TRUE(fields.Read(assembly, cin));
        const auto accepted_bits = AcceptedReceiptBits(accepted);
        EXPECT_GT(accepted.local_masked_tasks(), 0u);
        if (positive_force) {
          EXPECT_GT(accepted.diagnostics().active_count, 0u);
          EXPECT_GT(accepted.diagnostics().maximum_force_norm_n, 0);
          EXPECT_GT(accepted.diagnostics().maximum_sti_diagonal_n_m, 0);
        }
        fe::NodalPreparedView prepared;
        fe::ShellPhysicalDiagnostics common;
        ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
        c::SelfContactTransactionReceipt receipt;
        ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
            fixture.rig.owner, token, common, prepared, accepted, &receipt)));
        ASSERT_NO_FATAL_FAILURE(CheckTranslatedLocalPolicy(receipt));
        EXPECT_EQ(receipt.policy_summary().exact_crossing_pairs, 2u);
        EXPECT_EQ(receipt.policy_summary().outcomes, 2u);
        std::vector<std::uint64_t> event_order;
        const auto outcomes = PolicyOutcomeBits(
            fixture.transaction.policy_outcomes(), &event_order);
        const auto summary = CompleteBatchPolicyBits(receipt.policy_summary());
        if (!retry) {
          fixture.Discard();
          EXPECT_FALSE(receipt.valid());
          ASSERT_TRUE(fixture.rig.Read(discarded));
          p::Exact(initial, discarded);
          continue;
        }
        ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
        p::Snapshot final;
        ASSERT_TRUE(fixture.rig.Read(final));
        EXPECT_EQ(final.stamp.epoch, 1u);
        EXPECT_EQ(p::Bits(final.stamp.time), p::Bits(p::H));
        if (batch_size == 1) {
          expected_fields = FieldBits(fields);
          expected_accepted = accepted_bits;
          expected_outcomes = outcomes;
          expected_summary = summary;
          expected_final = final.values;
        } else {
          EXPECT_EQ(FieldBits(fields), expected_fields);
          EXPECT_EQ(accepted_bits, expected_accepted);
          EXPECT_EQ(outcomes, expected_outcomes);
          EXPECT_EQ(summary, expected_summary);
          EXPECT_EQ(final.values, expected_final);
        }
      }
      const auto after = fixture.transaction.allocations();
      EXPECT_EQ(after.device.device_bytes, allocations.device.device_bytes);
      EXPECT_EQ(after.device.device_allocations, allocations.device.device_allocations);
      EXPECT_EQ(after.activity.host_bytes, allocations.activity.host_bytes);
    }
  }
}

TEST(SelfContactTransactionCuda,
     RawSubbatchesCannotBypassCompleteStreamWorkCapAndRollback) {
  Fixture fixture(true);
  fixture.single_parent = false;
  c::SelfContactTransactionLimits limits;
  limits.crossing.max_total_work = 1;
  limits.max_stream_crossing_work = 1;
  ASSERT_TRUE(fixture.Initialize(limits));
  EXPECT_EQ(fixture.transaction.forecast().crossing_batch_pair_capacity, 1u);
  p::Snapshot initial, discarded;
  ASSERT_TRUE(fixture.rig.Read(initial));
  for (unsigned retry = 0; retry < 2; ++retry) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    c::SelfContactTransactionReceipt receipt;
    const auto report = fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &receipt);
    EXPECT_EQ(report.status, c::SelfContactTransactionStatus::CrossingFailure);
    EXPECT_EQ(report.crossing_status, c::RepresentedIntervalStatus::ResourceLimit);
    EXPECT_STREQ(report.message, "Complete crossing stream exceeds its hard work cap");
    EXPECT_FALSE(receipt.valid());
    EXPECT_FALSE(accepted.valid());
    EXPECT_FALSE(fixture.transaction.policy_summary().complete);
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 0u);
    fixture.Discard();
    ASSERT_TRUE(fixture.rig.Read(discarded));
    p::Exact(initial, discarded);
  }
}

}  // namespace self_contact_transaction_cuda_test
