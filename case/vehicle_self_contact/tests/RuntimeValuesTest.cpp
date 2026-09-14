#include "../RuntimeBudget.h"
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
    transaction.startup_host_bytes = 1700;
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

}  // namespace crash::cases::vehicle_self_contact::test
