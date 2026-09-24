// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FacetFilterCudaProbe.h"
#include "TransactionReportAssertions.h"
// Included after the actual physical owner and complete fieldwise helpers.
namespace self_contact_transaction_cuda_test {
TEST(SelfContactNativeCrossingTransactionCuda, ExactSelectedPoolForecastAndAggregateCaps) {
  for (bool filters : {false, true}) {
    SCOPED_TRACE(filters);
    Fixture fixture;
    ASSERT_TRUE(fixture.InitializeInfrastructure());
    auto& config = fixture.config;
    config.force.owner = fixture.rig.owner.accepted();
    config.force.startup = fixture.rig.fixture.Identity().startup;
    config.force.stiffness_per_area_n_m3 = 2e9;
    config.force.event_capacity = 512;
    config.force.configuration_id = p::Configuration;
    config.force.qualification_id = p::Qualification;
    config.source_id = p::SelfContactSource;
    config.enable_cuda_facet_filters = filters;
    c::SelfContactTransactionLimits limits;
    const auto cpu = c::SelfContactTransaction::Forecast(config, fixture.uses, fixture.rig.fixture.Identity(), limits);
    ASSERT_TRUE(Good(cpu.report));
    const auto cpu_executor = sct::CrossingExecutor::Preflight(config, limits);
    config.enable_cuda_native_crossing = true;
    config.native_crossing_device_workers = 512;
    config.native_crossing_numeric_cohort_pairs = 4096;
    const auto gpu = c::SelfContactTransaction::Forecast(config, fixture.uses, fixture.rig.fixture.Identity(), limits);
    ASSERT_TRUE(Good(gpu.report));
    const auto gpu_executor = sct::CrossingExecutor::Preflight(config, limits);
    EXPECT_EQ(gpu.forecast.owned_host_bytes, cpu.forecast.owned_host_bytes - cpu_executor.owned_host_bytes + gpu_executor.owned_host_bytes);
    EXPECT_EQ(gpu.forecast.startup_host_bytes, cpu.forecast.startup_host_bytes - cpu_executor.startup_host_bytes + gpu_executor.startup_host_bytes);
    EXPECT_EQ(gpu.forecast.device_bytes, cpu.forecast.device_bytes + gpu_executor.device_bytes);
    EXPECT_EQ(gpu.forecast.device_allocations, cpu.forecast.device_allocations + 1);
    auto cap = limits;
    cap.max_host_bytes = gpu.forecast.owned_host_bytes - 1;
    EXPECT_EQ(c::SelfContactTransaction::Forecast(config, fixture.uses, fixture.rig.fixture.Identity(), cap).report.status, c::SelfContactTransactionStatus::ResourceLimit);
    cap = limits; cap.max_startup_host_bytes = gpu.forecast.startup_host_bytes - 1;
    EXPECT_EQ(c::SelfContactTransaction::Forecast(config, fixture.uses, fixture.rig.fixture.Identity(), cap).report.status, c::SelfContactTransactionStatus::ResourceLimit);
    cap = limits; cap.max_device_bytes = gpu.forecast.device_bytes - 1;
    EXPECT_EQ(c::SelfContactTransaction::Forecast(config, fixture.uses, fixture.rig.fixture.Identity(), cap).report.status, c::SelfContactTransactionStatus::ResourceLimit);
    ASSERT_EQ(fixture.rig.owner.BorrowOwnerStream(&fixture.owner_stream).status, fe::NodalStatus::Ok);
    const auto allocations = facet_filter_cuda_probe::Allocations();
    ASSERT_TRUE(Good(fixture.transaction.Initialize(config, fixture.uses, fixture.rig.owner,
        fixture.rig.publication, fixture.physical, fixture.rig.Participants(),
        fixture.rig.fixture.Identity(), fixture.owner_stream, limits)));
    EXPECT_EQ(facet_filter_cuda_probe::Allocations() - allocations, gpu.forecast.device_allocations);
    EXPECT_EQ(fixture.transaction.allocations().device.device_bytes, gpu.forecast.device_bytes);
    EXPECT_EQ(fixture.transaction.allocations().device.device_allocations, gpu.forecast.device_allocations);
  }
}
TEST(SelfContactNativeCrossingTransactionCuda, OptionalExecutorPreservesPhysicalAssemblyDiscardAndCommit) {
  for (bool compact_census : {false, true}) {
    SCOPED_TRACE(compact_census);
    DeterminismObservation reference;
    for (unsigned mode = 0; mode < 3; ++mode) {
      SCOPED_TRACE(mode); // CPU, GPU native only, both optional GPU backends.
      Fixture fixture(false, false, 2.5, p::ContactConstraintLayout::Legacy, 1);
      fixture.config.enable_diagnostics = true;
      fixture.config.enable_cuda_native_crossing = mode != 0;
      fixture.config.native_crossing_numeric_cohort_pairs = 4096;
      fixture.config.enable_cuda_facet_filters = mode == 2;
      c::SelfContactTransactionLimits limits;
      if (!compact_census) limits.max_global_events = limits.max_event_identity_census = 512;
      ASSERT_TRUE(fixture.Initialize(limits));
      p::Snapshot before, after;
      ASSERT_TRUE(fixture.rig.Read(before));
      for (unsigned retry = 0; retry < 2; ++retry) {
        fe::NodalTrialToken token; fe::NodalAssemblyView assembly;
        ASSERT_TRUE(fixture.rig.Begin(token, assembly));
        c::SelfContactAcceptedAssemblyReceipt accepted;
        ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(fixture.rig.owner, token, assembly, &accepted)));
        fe::NodalCinAssemblyView cin;
        ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token, &cin)));
        AssemblyFields fields(fixture.rig.fixture.domain.node_count());
        ASSERT_TRUE(fields.Read(assembly, cin));
        fe::NodalPreparedView prepared; fe::ShellPhysicalDiagnostics common;
        ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
        c::SelfContactTransactionReceipt receipt;
        ASSERT_TRUE(Good(fixture.transaction.SealCandidate(fixture.rig.owner, token, common, prepared, accepted, &receipt)));
        if (!mode) EXPECT_EQ(fixture.transaction.diagnostics().candidate.native_device.calls, 0u);
        if (!retry) {
          fixture.Discard(); ASSERT_TRUE(fixture.rig.Read(after)); p::Exact(before, after);
          EXPECT_FALSE(fixture.transaction.policy_outcomes().complete);
          continue;
        }
        DeterminismObservation actual;
        actual.fields = FieldBits(fields);
        actual.accepted_receipt = AcceptedReceiptBits(accepted);
        actual.transaction_receipt = TransactionReceiptBits(receipt);
        actual.policy_summary = PolicySummaryBits(fixture.transaction.policy_summary());
        actual.policy_outcomes = PolicyOutcomeBits(fixture.transaction.policy_outcomes(), &actual.canonical_event_order);
        ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
        ASSERT_TRUE(fixture.rig.Read(after));
        actual.accepted_state = after.values; actual.final_stamp = StampBits(after.stamp);
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
  }
}
TEST(SelfContactNativeCrossingTransactionCuda, NonlocalRejectionAndRetryKeepNativeFailureAndAcceptedState) {
  c::SelfContactTransactionReport reference;
  for (bool enabled : {false, true}) {
    Fixture fixture(false, true);
    fixture.config.enable_cuda_native_crossing = enabled;
    fixture.config.native_crossing_numeric_cohort_pairs = 4096;
    c::SelfContactTransactionLimits limits;
    limits.crossing.max_depth = 4;
    limits.crossing.max_work_per_pair = 31;
    limits.crossing.max_total_work = 31 * 64;
    ASSERT_TRUE(fixture.Initialize(limits));
    p::Snapshot before, after;
    ASSERT_TRUE(fixture.rig.Read(before));
    for (unsigned retry = 0; retry < 2; ++retry) {
      fe::NodalTrialToken token; fe::NodalAssemblyView assembly;
      ASSERT_TRUE(fixture.rig.Begin(token, assembly));
      c::SelfContactAcceptedAssemblyReceipt accepted;
      ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(fixture.rig.owner, token, assembly, &accepted)));
      fe::NodalPreparedView prepared; fe::ShellPhysicalDiagnostics common;
      ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
      c::SelfContactTransactionReceipt receipt;
      const auto report = fixture.transaction.SealCandidate(fixture.rig.owner, token, common, prepared, accepted, &receipt);
      ASSERT_NE(report.status, c::SelfContactTransactionStatus::Ok);
      EXPECT_FALSE(receipt.valid());
      if (!enabled && !retry) reference = report;
      else sct::test::ExpectTransactionReport(report, reference);
      ASSERT_TRUE(fixture.rig.Read(after)); p::Exact(before, after);
      fixture.Discard();
    }
  }
}
}  // namespace self_contact_transaction_cuda_test
