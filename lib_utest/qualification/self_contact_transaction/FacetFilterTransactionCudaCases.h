// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FacetFilterCudaProbe.h"
#include "TransactionReportAssertions.h"
namespace self_contact_transaction_cuda_test {
TEST(SelfContactFacetFilterTransactionCuda, DisabledHasNoExtraDeviceAllocationAndEnabledForecastIsExact) {
  namespace filters=c::self_contact_filters;
  for(bool enabled:{false,true}) {
    SCOPED_TRACE(enabled);Fixture fixture;ASSERT_TRUE(fixture.InitializeInfrastructure());
    EXPECT_EQ(fixture.transaction.facet_filter_initialization(),c::SelfContactFacetFilterInitialization::NotInitialized);
    auto& config=fixture.config;
    config.force.owner=fixture.rig.owner.accepted();config.force.startup=fixture.rig.fixture.Identity().startup;
    config.force.stiffness_per_area_n_m3=2e9;config.force.event_capacity=512;
    config.force.configuration_id=p::Configuration;config.force.qualification_id=p::Qualification;
    config.source_id=p::SelfContactSource;config.enable_cuda_facet_filters=enabled;
    c::SelfContactTransactionLimits limits;
    const auto full=c::SelfContactTransaction::Forecast(config,fixture.uses,fixture.rig.fixture.Identity(),limits);
    ASSERT_TRUE(Good(full.report));
    config.enable_cuda_facet_filters=false;
    const auto scalar=c::SelfContactTransaction::Forecast(config,fixture.uses,fixture.rig.fixture.Identity(),limits);
    ASSERT_TRUE(Good(scalar.report));config.enable_cuda_facet_filters=enabled;
    const auto extra=sct::FacetFilters::Preflight(fixture.uses.facet_uses().size(),limits.max_facet_pair_chunk,
        limits.max_host_bytes,limits.max_device_bytes);
    ASSERT_EQ(extra.report.status,filters::Status::Ok);
    EXPECT_EQ(full.forecast.device_bytes,scalar.forecast.device_bytes+(enabled?extra.device_bytes:0));
    EXPECT_EQ(full.forecast.owned_host_bytes,scalar.forecast.owned_host_bytes+(enabled?extra.owned_host_bytes:0));
    EXPECT_EQ(full.forecast.startup_host_bytes,scalar.forecast.startup_host_bytes+(enabled?extra.startup_host_bytes:0));
    if(enabled) {
      auto cap=limits;cap.max_host_bytes=full.forecast.owned_host_bytes-1;
      EXPECT_EQ(c::SelfContactTransaction::Forecast(config,fixture.uses,fixture.rig.fixture.Identity(),cap).report.status,c::SelfContactTransactionStatus::ResourceLimit);
      cap=limits;cap.max_device_bytes=full.forecast.device_bytes-1;
      EXPECT_EQ(c::SelfContactTransaction::Forecast(config,fixture.uses,fixture.rig.fixture.Identity(),cap).report.status,c::SelfContactTransactionStatus::ResourceLimit);
      cap=limits;cap.max_startup_host_bytes=full.forecast.startup_host_bytes-1;
      EXPECT_EQ(c::SelfContactTransaction::Forecast(config,fixture.uses,fixture.rig.fixture.Identity(),cap).report.status,c::SelfContactTransactionStatus::ResourceLimit);
    }
    ASSERT_EQ(fixture.rig.owner.BorrowOwnerStream(&fixture.owner_stream).status,fe::NodalStatus::Ok);
    const auto before=facet_filter_cuda_probe::Allocations();
    ASSERT_TRUE(Good(fixture.transaction.Initialize(config,fixture.uses,fixture.rig.owner,fixture.rig.publication,
        fixture.physical,fixture.rig.Participants(),fixture.rig.fixture.Identity(),fixture.owner_stream,limits)));
    EXPECT_EQ(fixture.transaction.facet_filter_initialization(),enabled
        ? c::SelfContactFacetFilterInitialization::Cuda
        : c::SelfContactFacetFilterInitialization::Disabled);
    EXPECT_EQ(facet_filter_cuda_probe::Allocations()-before,enabled?3u:2u);
    EXPECT_EQ(fixture.transaction.allocations().device.device_allocations,enabled?3u:2u);
    EXPECT_EQ(fixture.transaction.allocations().device.device_bytes,full.forecast.device_bytes);
  }
}
TEST(SelfContactFacetFilterTransactionCuda, ScalarAndGpuKeepBothCensusPassesForcesPolicyDiscardAndCommit) {
  for(bool compact_census:{false,true}) {
    SCOPED_TRACE(compact_census);DeterminismObservation reference;
    for(bool enabled:{false,true}) {
      Fixture fixture(false,false,2.5,p::ContactConstraintLayout::Legacy,1);
      fixture.config.enable_cuda_facet_filters=enabled;
      c::SelfContactTransactionLimits limits;
      if(!compact_census){limits.max_global_events=512;limits.max_event_identity_census=512;}
      ASSERT_TRUE(fixture.Initialize(limits));p::Snapshot before,after;ASSERT_TRUE(fixture.rig.Read(before));
      for(unsigned retry=0;retry<2;++retry) {
        fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
        ASSERT_TRUE(fixture.rig.Begin(token,assembly));c::SelfContactAcceptedAssemblyReceipt accepted;
        ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(fixture.rig.owner,token,assembly,&accepted)));
        fe::NodalCinAssemblyView cin;ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token,&cin)));
        AssemblyFields fields(fixture.rig.fixture.domain.node_count());ASSERT_TRUE(fields.Read(assembly,cin));
        fe::NodalPreparedView prepared;fe::ShellPhysicalDiagnostics common;
        ASSERT_TRUE(fixture.Prepare(token,assembly,prepared,common));c::SelfContactTransactionReceipt receipt;
        ASSERT_TRUE(Good(fixture.transaction.SealCandidate(fixture.rig.owner,token,common,prepared,accepted,&receipt)));
        if(!retry) {fixture.Discard();ASSERT_TRUE(fixture.rig.Read(after));p::Exact(before,after);EXPECT_FALSE(fixture.transaction.policy_outcomes().complete);continue;}
        DeterminismObservation actual;actual.fields=FieldBits(fields);
        actual.accepted_receipt=AcceptedReceiptBits(accepted);actual.transaction_receipt=TransactionReceiptBits(receipt);
        actual.policy_summary=PolicySummaryBits(fixture.transaction.policy_summary());
        actual.policy_outcomes=PolicyOutcomeBits(fixture.transaction.policy_outcomes(),&actual.canonical_event_order);
        ASSERT_TRUE(fixture.Commit(token,prepared,common,receipt));ASSERT_TRUE(fixture.rig.Read(after));
        actual.accepted_state=after.values;actual.final_stamp=StampBits(after.stamp);
        if(!enabled)reference=actual;
        else {EXPECT_EQ(actual.fields,reference.fields);EXPECT_EQ(actual.accepted_receipt,reference.accepted_receipt);
          EXPECT_EQ(actual.transaction_receipt,reference.transaction_receipt);EXPECT_EQ(actual.policy_summary,reference.policy_summary);
          EXPECT_EQ(actual.policy_outcomes,reference.policy_outcomes);EXPECT_EQ(actual.canonical_event_order,reference.canonical_event_order);
          EXPECT_EQ(actual.accepted_state,reference.accepted_state);EXPECT_EQ(actual.final_stamp,reference.final_stamp);}
      }
    }
  }
}
TEST(SelfContactFacetFilterTransactionCuda, NonlinearBudgetExhaustionPreservesFirstFailureAndAcceptedState) {
  c::SelfContactTransactionReport reference;
  for(bool enabled:{false,true}) {
    Fixture fixture(false,false,2.5,p::ContactConstraintLayout::MergedPartAndPlain);
    fixture.config.enable_cuda_facet_filters=enabled;
    fixture.rig.external_force_source_node=14;fixture.rig.external_force_z_n=1000;
    c::SelfContactTransactionLimits limits;limits.max_nonlinear_subdivision_work_per_pair=1;limits.max_nonlinear_subdivision_depth=0;
    ASSERT_TRUE(fixture.Initialize(limits));p::Snapshot before,after;ASSERT_TRUE(fixture.rig.Read(before));
    for(unsigned retry=0;retry<2;++retry) {
      fe::NodalTrialToken token;fe::NodalAssemblyView assembly;ASSERT_TRUE(fixture.rig.Begin(token,assembly));
      c::SelfContactAcceptedAssemblyReceipt accepted;ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(fixture.rig.owner,token,assembly,&accepted)));
      fe::NodalPreparedView prepared;fe::ShellPhysicalDiagnostics common;ASSERT_TRUE(fixture.Prepare(token,assembly,prepared,common));
      c::SelfContactTransactionReceipt receipt;
      const auto report=fixture.transaction.SealCandidate(fixture.rig.owner,token,common,prepared,accepted,&receipt);
      ASSERT_EQ(report.status,c::SelfContactTransactionStatus::UnsupportedMotion);EXPECT_FALSE(receipt.valid());
      if(!enabled&&!retry)reference=report;else sct::test::ExpectTransactionReport(report,reference);
      ASSERT_TRUE(fixture.rig.Read(after));p::Exact(before,after);fixture.Discard();
    }
  }
}
}  // namespace self_contact_transaction_cuda_test
