#include "TwoIntervalAcceptance.h"
#include "retained_archive/TwoIntervalReplay.h"
#include "OriginalFixture.h"
#include "../SelfContactSummary.h"
#include "../contact_diagnostics/Document.h"
#include <cstdlib>
#include "case/vehicle_self_contact/VehicleSelfContactSetup.h"
#include "output/physical_run/IntervalIO.h"
#include "output/physical_run/Replay.h"

#include <array>
#include <iomanip>
#include <iostream>

namespace crash::cases::vehicle_run::test {

Config CombinedConfig() {
    Config result;
    result.physical_profile = PhysicalProfile::VehicleSupportsV5;
    result.contact_profile = ContactProfile::WallSelfContactV1;
    result.fixed_dt_s = FixedStepS;
    if(const auto* enabled=std::getenv("ROBO_SELF_CONTACT_DIAGNOSTICS")) {
        output::Require((enabled[0]=='0' || enabled[0]=='1') && enabled[1]=='\0',
            "ROBO_SELF_CONTACT_DIAGNOSTICS must be 0 or 1");
        result.self_contact_diagnostics=enabled[0]=='1';
    }
    return result;
}

namespace {
namespace archive = output::physical_run;
void PrintProgress(const Progress& progress) {
    std::cout << std::setprecision(17)
        << "V5_WALL_SELF_CONTROLLER accepted=" << progress.accepted.epoch
        << " time_s=" << progress.accepted.time_s
        << " elapsed_s=" << progress.elapsed_s
        << " events=" << progress.self_contact.last_event_count
        << " policy_outcomes=" << progress.self_contact.last_policy_outcomes
        << " policy_digest=" << progress.self_contact.last_policy_digest;
    detail::WriteSelfContactWorkProgress(std::cout, progress.self_contact);
    for (const auto stage : {
             vehicle_dynamics::StepStage::AssembleSelfContact,
             vehicle_dynamics::StepStage::EvaluateSelfContact,
             vehicle_dynamics::StepStage::PrepareStep,
             vehicle_dynamics::StepStage::Commit}) {
        const auto i = static_cast<std::size_t>(stage);
        std::cout << ' ' << vehicle_dynamics::StepStageNames[i] << "_last_s="
                  << progress.mechanics_timing.last_step[i].wall_ns * 1e-9;
    }
    std::cout << '\n';
}


} // namespace

void CheckTwoCommittedV5Intervals(const TwoIntervalExecution& execute) {
    std::cout << std::unitbuf;
    const auto source = Source(PhysicalProfile::VehicleSupportsV5, nullptr,
                               ContactProfile::WallSelfContactV1);
    ASSERT_TRUE(source.self_contact);
    ASSERT_EQ(source.setup.execution().physical().domain()->node_count(), 376930u);
    ASSERT_EQ(source.self_contact->active_uses().parents().size(), 337092u);
    EXPECT_EQ(&source.self_contact->execution().physical(),
              &source.setup.execution().physical());
    const auto config = CombinedConfig();
    const auto plan = PreparedRun::Prepare(
        source.setup, source.joints, config, Identity(), source.self_contact);
    const auto& forecast = plan.forecast();
    ASSERT_TRUE(forecast.contact.self_contact);
    EXPECT_FALSE(forecast.caps.expanded);
    EXPECT_LE(forecast.complete_host_bytes, 20ull * 1000 * 1000 * 1000);
    EXPECT_LE(forecast.complete_archive_bytes, 2ull << 30);
    EXPECT_GT(forecast.contact.device_bytes, forecast.wall.device_bytes);
    const auto event_capacity =
        forecast.contact.self_contact->transaction.accepted_event_capacity;
    EXPECT_GT(event_capacity, 32491u);
    ::testing::Test::RecordProperty("complete_host_upper_bound",
                   std::to_string(forecast.complete_host_bytes));
    ::testing::Test::RecordProperty("complete_device_bytes",
                   std::to_string(forecast.contact.device_bytes));
    ::testing::Test::RecordProperty("event_capacity", std::to_string(event_capacity));

    records::test::Directory temporary;
    const auto* requested = std::getenv("ROBO_VEHICLE_RUN_OUTPUT");
    const auto destination = requested && *requested
        ? std::filesystem::path(requested) : temporary.path;
    Control control;
    control.maximum_accepted_intervals = 2;
    control.progress_period_s = 1;
    control.progress = PrintProgress;
    const auto result = execute(plan, destination, control);
    PrintProgress(result.loop.progress);
    if(result.last_contact_attempt.enabled) {
        std::cout<<"V5_CONTACT_LAST_ATTEMPT";
        contact_diagnostics::WriteProgress(std::cout,result.last_contact_attempt);
        std::cout<<'\n';
    }
    ASSERT_TRUE(result.session_initialized) << result.loop.reason;
    ASSERT_EQ(result.loop.kind, StopKind::IntervalLimit) << result.loop.reason;
    ASSERT_TRUE(result.loop.valid_manifest) << result.loop.reason;
    ASSERT_TRUE(result.archive_manifest);
    ASSERT_TRUE(result.viewer_input) << result.viewer_input_error;
    ASSERT_TRUE(result.summary) << result.summary_error;
    EXPECT_FALSE(result.rejected_self_contact);
    EXPECT_FALSE(result.rejected_contact_status);
    EXPECT_FALSE(result.rejected_step_limit_s);
    EXPECT_TRUE(result.viewer_input_error.empty());
    EXPECT_TRUE(result.summary_error.empty());
    const auto& progress = result.loop.progress;
    EXPECT_EQ(progress.accepted.epoch, 2u);
    EXPECT_EQ(output::Bits(progress.accepted.time_s), output::Bits(2 * FixedStepS));
    ASSERT_TRUE(progress.contact.available);
    ASSERT_TRUE(progress.self_contact.available);
    EXPECT_EQ(progress.contact.intervals, 2u);
    EXPECT_EQ(progress.self_contact.intervals, 2u);
    EXPECT_GT(progress.contact.peak_observed_force_n, 0);
    EXPECT_GT(progress.contact.peak_observed_penetration_m, 0);
    EXPECT_EQ(progress.mechanics.intervals, 2u);

    static_assert(SummaryByteCap == archive::MetadataCap);
    ASSERT_NO_FATAL_FAILURE(retained::CheckTwoIntervalArchive(
        destination, *result.viewer_input, *result.summary,
        {forecast.contact.self_contact->identity.source_id,
         source.self_contact->active_uses().parents().size(), event_capacity,
         progress.self_contact.last_event_count, progress.self_contact.last_policy_digest,
         progress.self_contact.last_accepted_base_potential_j, result.loop.reason}));

}

} // namespace crash::cases::vehicle_run::test
