#include "TwoIntervalAcceptance.h"
#include "OriginalFixture.h"
#include "case/vehicle_self_contact/VehicleSelfContactSetup.h"

#include <iostream>

namespace crash::cases::vehicle_run::test {

TEST(VehicleRunWallSelfContact,
     FullV5ForecastAdmitsEventHeadroomBeforeAnyDynamicsOwner) {
    const auto source = Source(PhysicalProfile::VehicleSupportsV5, nullptr,
                               ContactProfile::WallSelfContactV1);
    ASSERT_TRUE(source.self_contact);
    const auto plan = PreparedRun::Prepare(source.setup, source.joints,
        CombinedConfig(), Identity(), source.self_contact);
    const auto& actual = plan.forecast();
    ASSERT_TRUE(actual.contact.self_contact);
    EXPECT_FALSE(actual.caps.expanded);
    EXPECT_LE(actual.complete_host_bytes, 20ull * 1000 * 1000 * 1000);
    EXPECT_LE(actual.complete_archive_bytes, 2ull << 30);

    // Compare against the SAME source/factory with the final M2 gate's exact
    // initial event reservation. This does not allocate either dynamics owner,
    // and the smaller reservation is never used for the production run.
    const auto composition = ContactComposition::Prepare(
        ContactProfile::WallSelfContactV1, source.self_contact);
    auto reference_config = composition.runtime_config();
    auto reference_limits = composition.runtime_limits();
    constexpr std::size_t ReferenceEvents = 32491;
    reference_config.event_capacity = ReferenceEvents;
    auto& transaction = reference_limits.self_contact.transaction;
    transaction.max_global_events = ReferenceEvents;
    transaction.max_event_identity_census = ReferenceEvents;
    transaction.max_event_hash_slots = 2 * ReferenceEvents;
    transaction.force.max_events = ReferenceEvents;
    auto dynamics_config = vehicle_wall::LoadedWallConfig();
    dynamics_config.startup.reserved_step_s = FixedStepS;
    const auto reference = vehicle_self_contact::LoadedWallSelfContact::Preflight(
        source.setup, *source.self_contact, reference_config, reference_limits,
        dynamics_config, &source.joints);
    ASSERT_GE(actual.contact.device_bytes, reference.device_bytes);
    const auto delta = actual.contact.device_bytes - reference.device_bytes;
    constexpr std::uint64_t PriorWholeDeviceGrowth = 6176112640ull;
    constexpr std::uint64_t GuardGrowthCap = 6ull << 30;
    ASSERT_LE(delta, GuardGrowthCap - PriorWholeDeviceGrowth)
        << "Forecast event headroom exceeds the previous gate's remaining GPU growth allowance";
    const auto projected_growth = PriorWholeDeviceGrowth + delta;
    RecordProperty("complete_host_upper_bound", std::to_string(actual.complete_host_bytes));
    RecordProperty("complete_device_bytes", std::to_string(actual.contact.device_bytes));
    RecordProperty("reference_device_bytes", std::to_string(reference.device_bytes));
    RecordProperty("event_capacity_device_delta", std::to_string(delta));
    RecordProperty("projected_whole_device_growth", std::to_string(projected_growth));
    RecordProperty("projected_guard_margin", std::to_string(GuardGrowthCap - projected_growth));
    RecordProperty("scope", "source-authenticated forecast; no dynamics owner or guarantee of future GPU usage");
    std::cout << "V5_WALL_SELF_FORECAST complete_device_bytes=" << actual.contact.device_bytes
              << " reference_device_bytes=" << reference.device_bytes
              << " event_capacity_device_delta=" << delta
              << " projected_guard_margin=" << GuardGrowthCap - projected_growth << std::endl;
}

TEST(VehicleRunWallSelfContact,
     TwoCommittedV5IntervalsPreserveBothContactProfilesAndReplay) {
    CheckTwoCommittedV5Intervals([](const PreparedRun& plan,
                                   const std::filesystem::path& destination,
                                   const Control& control) {
        return plan.Execute(destination, control);
    });
}

} // namespace crash::cases::vehicle_run::test
