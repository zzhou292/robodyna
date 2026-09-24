#include "../RuntimeBudget.h"
#include "../SelfContactStageError.h"
#include "case/vehicle_dynamics/ScratchReceiptRoster.h"
#include "case/vehicle_dynamics/StepTiming.h"

#include <gtest/gtest.h>

namespace crash::cases::vehicle_self_contact::test {

TEST(VehicleSelfContactRuntimeValues,
     FirstProfileAndTimingAppendAreStable) {
    EXPECT_EQ(FirstProfileStiffnessPerAreaNPerM3, 2e9);
    EXPECT_EQ(static_cast<std::size_t>(
                  vehicle_dynamics::StepStage::AssembleBeam18), 28u);
    EXPECT_EQ(static_cast<std::size_t>(
                  vehicle_dynamics::StepStage::EvaluateBeam18), 29u);
    EXPECT_EQ(static_cast<std::size_t>(
                  vehicle_dynamics::StepStage::AssembleSelfContact), 30u);
    EXPECT_EQ(static_cast<std::size_t>(
                  vehicle_dynamics::StepStage::EvaluateSelfContact), 31u);
    EXPECT_EQ(vehicle_dynamics::StepStageCount, 32u);
    EXPECT_EQ(RuntimeConfig{}.source_id, 0u);
    EXPECT_EQ(RuntimeConfig{}.event_capacity, 0u);
}

TEST(VehicleSelfContactRuntimeValues,
     SharedSourceActivityDeviceAndParticipationAreCountedOnce) {
    vehicle_dynamics::Forecast dynamics;
    dynamics.startup.retained_source_upper_bound = 1000;
    dynamics.startup.retained_host_upper_bound = 2000;
    dynamics.startup.peak_temporary_bytes = 500;
    dynamics.startup.device_bytes = 100;
    dynamics.workspace_bytes = 20;

    SetupForecast setup;
    setup.shared_vehicle_source_reservation_bytes = 1000;
    setup.retained_setup_reservation_bytes = 1300;
    setup.peak_temporary_reservation_bytes = 600;

    tlfea::contact::SelfContactTransactionForecast transaction;
    transaction.activity.arena_bytes = 50;
    transaction.activity.owned_host_bytes = 80;
    transaction.activity.startup_host_bytes = 80;
    transaction.broadphase.retained_source_bytes = 100;
    transaction.broadphase.owned_host_bytes = 140;
    transaction.broadphase.startup_host_bytes = 140;
    transaction.regularity.retained_active_use_bytes = 200;
    transaction.regularity.owned_payload_bytes = 240;
    transaction.regularity.startup_payload_bytes = 240;
    transaction.force.owned_host_bytes = 100;
    transaction.force.retained_active_use_bytes = 200;
    transaction.force.startup_host_bytes = 1000;
    transaction.participation.publication_host_bytes = 40;
    transaction.owned_host_bytes =
        detail::TransactionChargesBroadphaseBacking() ? 1000 : 900;
    transaction.startup_host_bytes = transaction.owned_host_bytes + 700;
    transaction.device_bytes = 300;

    RuntimeLimits limits;
    limits.host_bytes = 5000;
    limits.device_bytes = 500;
    const auto result = detail::ComposeForecast(
        dynamics, setup, transaction, 10, limits);
    EXPECT_EQ(detail::IncrementalSetupHost(setup), 300u);
    EXPECT_EQ(detail::IncrementalTransactionHost(transaction), 700u);
    EXPECT_EQ(detail::TransactionStartupScratch(transaction), 700u);
    EXPECT_EQ(result.retained_host_upper_bound, 3030u);
    EXPECT_EQ(result.peak_host_upper_bound, 3730u);
    EXPECT_EQ(result.device_bytes, 400u);
    EXPECT_EQ(result.transaction.activity.arena_bytes, 50u);
    EXPECT_EQ(
        result.transaction.participation.publication_host_bytes, 40u);

    limits.host_bytes = result.peak_host_upper_bound;
    limits.device_bytes = result.device_bytes;
    EXPECT_EQ(detail::ComposeForecast(
        dynamics, setup, transaction, 10, limits)
        .peak_host_upper_bound, limits.host_bytes);
    --limits.host_bytes;
    EXPECT_THROW(
        detail::ComposeForecast(
            dynamics, setup, transaction, 10, limits),
        std::runtime_error);
    ++limits.host_bytes;
    --limits.device_bytes;
    EXPECT_THROW(
        detail::ComposeForecast(
            dynamics, setup, transaction, 10, limits),
        std::runtime_error);
}

TEST(VehicleSelfContactRuntimeValues,
     TypedAcceptedCapacityFailureExposesCountWithoutTextParsing) {
    tlfea::contact::SelfContactTransactionReport report;
    report.status =
        tlfea::contact::SelfContactTransactionStatus::ResourceLimit;
    report.candidate = 37;
    report.count_kind = tlfea::contact::SelfContactTransactionCountKind::ExactAcceptedEvents;
    report.pair = 91;
    report.message = "opaque";
    const SelfContactStageError error(
        report, SelfContactRuntimeStage::AcceptedAssembly, 1);
    EXPECT_EQ(error.required_events(), 37u);
    EXPECT_EQ(error.report().candidate, 37u);
    EXPECT_EQ(error.report().pair, 91u);
    EXPECT_EQ(error.stage(),
              SelfContactRuntimeStage::AcceptedAssembly);

    const SelfContactStageError candidate(
        report, SelfContactRuntimeStage::CandidateSeal, 1);
    EXPECT_EQ(candidate.required_events(), 0u);
}

TEST(VehicleSelfContactRuntimeCoupon,
     SyntheticSelfOnlyAndWallSelfBudgetsChargeTwoSlotsOnce) {
    SetupForecast setup;
    setup.shared_vehicle_source_reservation_bytes = 1000;
    setup.retained_setup_reservation_bytes = 1300;

    RuntimeForecast self;
    auto& transaction = self.transaction;
    transaction.activity.arena_bytes = 50;
    transaction.activity.owned_host_bytes = 80;
    transaction.activity.startup_host_bytes = 80;
    transaction.broadphase.retained_source_bytes = 100;
    transaction.broadphase.owned_host_bytes = 140;
    transaction.broadphase.startup_host_bytes = 140;
    transaction.regularity.retained_active_use_bytes = 200;
    transaction.regularity.owned_payload_bytes = 240;
    transaction.regularity.startup_payload_bytes = 240;
    transaction.force.owned_host_bytes = 100;
    transaction.force.retained_active_use_bytes = 200;
    transaction.force.startup_host_bytes = 1000;
    transaction.participation.publication_host_bytes = 40;
    transaction.owned_host_bytes =
        detail::TransactionChargesBroadphaseBacking() ? 1000 : 900;
    transaction.startup_host_bytes = transaction.owned_host_bytes + 700;
    transaction.device_bytes = 300;
    self.retained_host_upper_bound = 2500;
    self.peak_host_upper_bound = 2800;

    tl::fea::ShellPhysicalScratchParticipationForecast participation;
    participation.publication_host_bytes = 60;
    participation.configured_issuer_host_bytes =
        2 * sizeof(tl::fea::ShellPhysicalScratchParticipation);
    participation.total_host_bytes =
        participation.publication_host_bytes +
        participation.configured_issuer_host_bytes;
    const auto combined = detail::ComposeCombinedBudget(
        1600, 1800, 100, 40, setup, self, participation,
        10, 5000, 500);
    EXPECT_EQ(combined.retained_host_upper_bound, 2590u);
    EXPECT_EQ(combined.peak_host_upper_bound, 2890u);
    EXPECT_EQ(combined.device_bytes, 400u);
    EXPECT_EQ(participation.publication_host_bytes, 60u);
    EXPECT_EQ(participation.configured_issuer_host_bytes,
        2 * sizeof(tl::fea::ShellPhysicalScratchParticipation));

    EXPECT_EQ(detail::ComposeCombinedBudget(
        1600, 1800, 100, 40, setup, self, participation,
        10, combined.peak_host_upper_bound, 500)
        .peak_host_upper_bound, combined.peak_host_upper_bound);
    EXPECT_THROW(detail::ComposeCombinedBudget(
        1600, 1800, 100, 40, setup, self, participation,
        10, combined.peak_host_upper_bound - 1, 500),
        std::runtime_error);
}

TEST(VehicleSelfContactRuntimeCoupon,
     CommitRosterPreservesMappedWallThenSelfContactSlots) {
    tl::fea::ShellPhysicalScratchParticipationReceipt wall_receipt;
    tl::fea::ShellPhysicalScratchParticipationReceipt self_receipt;
    const tl::fea::ShellPhysicalScratchReceiptRoster wall{
        &wall_receipt, nullptr};
    const tl::fea::ShellPhysicalScratchReceiptRoster self_contact{
        nullptr, &self_receipt};
    const auto combined =
        vehicle_dynamics::detail::ComposeScratchReceipts(
            wall, self_contact);
    EXPECT_EQ(combined.mapped_wall, &wall_receipt);
    EXPECT_EQ(combined.self_contact, &self_receipt);
}

TEST(VehicleSelfContactRuntimeValues, SingleConfigMapperForwardsOptionalFiltersWithoutChangingPhysics) {
    EXPECT_FALSE(RuntimeConfig{}.enable_cuda_facet_filters);
    tl::fea::NodalStamp owner;
    owner.owner_id=17;owner.epoch=3;owner.fixed_dt=2e-7;owner.time=6e-7;
    tl::fea::ShellPhysicalPublicationIdentity identity;
    identity.configuration_id=19;identity.qualification_id=23;
    RuntimeConfig config;
    config.source_id=29;config.event_capacity=31;config.broadphase_axis=2;
    for (const bool enabled:{false,true}) {
        config.enable_cuda_facet_filters=enabled;
        for (const bool diagnostics:{false,true}) {
            config.enable_diagnostics=diagnostics;
            const auto actual=detail::TransactionConfig(config,owner,identity);
            EXPECT_EQ(actual.enable_cuda_facet_filters,enabled);
            EXPECT_EQ(actual.enable_diagnostics,diagnostics);
            EXPECT_EQ(actual.source_id,29u);
            EXPECT_EQ(actual.broadphase_axis,2u);
            EXPECT_EQ(actual.force.event_capacity,31u);
            EXPECT_EQ(actual.force.configuration_id,19u);
            EXPECT_EQ(actual.force.qualification_id,23u);
            EXPECT_EQ(actual.force.stiffness_per_area_n_m3,2e9);
            EXPECT_EQ(actual.force.owner.owner_id,17u);
            EXPECT_EQ(actual.force.owner.epoch,3u);
            EXPECT_EQ(actual.force.owner.fixed_dt,2e-7);
            EXPECT_EQ(actual.force.owner.time,6e-7);
        }
    }
}

TEST(VehicleSelfContactRuntimeValues, TypedInitializationMustMatchTheExactRequestedOrFallbackFootprint) {
    namespace c=tlfea::contact;
    using Mode=c::SelfContactFacetFilterInitialization;
    RuntimeConfig config;
    c::SelfContactTransactionForecast cpu,cuda;
    cpu.activity.arena_bytes=cuda.activity.arena_bytes=101;
    cpu.device_bytes=300;cpu.device_allocations=2;
    cuda.device_bytes=500;cuda.device_allocations=3;
    c::SelfContactTransactionAllocationInfo actual;
    actual.activity.host_bytes=101;actual.device={300,2};
    EXPECT_TRUE(detail::TransactionAllocationsMatch(config,Mode::Disabled,cpu,actual));
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::NotInitialized,cpu,actual));
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::Cuda,cpu,actual));
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::Disabled,cpu,actual,&cpu));
    config.enable_cuda_facet_filters=true;
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::Disabled,cuda,actual));
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::Cuda,cuda,actual));
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::UnsupportedHostArithmetic,cuda,actual));
    EXPECT_TRUE(detail::TransactionAllocationsMatch(config,Mode::UnsupportedHostArithmetic,cuda,actual,&cpu));
    for (unsigned field=0;field<3;++field) {
        auto corrupt=actual;
        if(field==0)++corrupt.activity.host_bytes;
        if(field==1)++corrupt.device.device_bytes;
        if(field==2)++corrupt.device.device_allocations;
        EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::UnsupportedHostArithmetic,cuda,corrupt,&cpu));
    }
    actual.device={500,3};
    EXPECT_TRUE(detail::TransactionAllocationsMatch(config,Mode::Cuda,cuda,actual));
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::Cuda,cuda,actual,&cpu));
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::UnsupportedHostArithmetic,cuda,actual,&cuda));
    auto different_activity=cpu;++different_activity.activity.arena_bytes;
    EXPECT_FALSE(detail::TransactionAllocationsMatch(config,Mode::UnsupportedHostArithmetic,cuda,actual,&different_activity));
    EXPECT_EQ(detail::EffectiveCombinedDeviceBytes(900,500,500),900u);
    EXPECT_EQ(detail::EffectiveCombinedDeviceBytes(900,500,300),700u);
    EXPECT_THROW(detail::EffectiveCombinedDeviceBytes(499,500,300),std::runtime_error);
    EXPECT_THROW(detail::EffectiveCombinedDeviceBytes(900,500,501),std::runtime_error);
}

TEST(VehicleSelfContactRuntimeValues, TotalStartupScratchIncludesOptionalComponentsOutsideOldPartitions) {
    tlfea::contact::SelfContactTransactionForecast transaction;
    transaction.owned_host_bytes=1000;
    transaction.startup_host_bytes=1800;
    transaction.force.owned_host_bytes=100;
    transaction.force.retained_active_use_bytes=200;
    transaction.force.startup_host_bytes=1000;
    EXPECT_EQ(detail::TransactionStartupScratch(transaction),800u);
    transaction.startup_host_bytes=1700;
    EXPECT_EQ(detail::TransactionStartupScratch(transaction),700u);
    transaction.startup_host_bytes=999;
    EXPECT_THROW(detail::TransactionStartupScratch(transaction),std::runtime_error);
}

}  // namespace crash::cases::vehicle_self_contact::test
