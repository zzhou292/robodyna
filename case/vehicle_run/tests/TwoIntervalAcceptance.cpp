#include "TwoIntervalAcceptance.h"
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

void CheckContactRow(const archive::Values& row,
                     const records::Context& context,
                     std::uint64_t epoch,
                     std::uint64_t source_id,
                     std::size_t selected_parents,
                     std::size_t event_capacity) {
    EXPECT_EQ(row.owner, context.identity().owner);
    EXPECT_EQ(row.stamp.epoch, epoch);
    EXPECT_EQ(row.stamp.base_epoch, epoch - 1);
    EXPECT_EQ(output::Bits(row.stamp.time), output::Bits(epoch * FixedStepS));
    EXPECT_EQ(output::Bits(row.stamp.base_time),
              output::Bits((epoch - 1) * FixedStepS));
    EXPECT_GT(row.stamp.attempt, 0u);
    ASSERT_TRUE(row.structural_limit_s);
    EXPECT_GE(*row.structural_limit_s, FixedStepS);
    ASSERT_TRUE(row.self_contact);
    const auto& contact = *row.self_contact;
    EXPECT_NO_THROW(archive::CheckSelfContactValues(contact));
    EXPECT_EQ(contact.source_id, source_id);
    EXPECT_EQ(contact.selected_parents, selected_parents);
    EXPECT_GT(contact.events, 0u);
    EXPECT_LE(contact.events, event_capacity);
    EXPECT_GT(contact.active_events, 0u);
    EXPECT_EQ(contact.events, contact.vertex_face_events + contact.edge_edge_events);
    EXPECT_EQ(contact.policy_outcomes, contact.candidate_facet_pairs);
    EXPECT_EQ(contact.policy_outcomes,
        contact.certified_separated + contact.same_rigid_exclusions +
        contact.local_intersections + contact.represented_vf + contact.represented_ee);
    EXPECT_EQ(contact.selected_parents,
        contact.active_parents + contact.removing_parents + contact.skipped_parents);
    EXPECT_GT(contact.maximum_force_n, 0);
    EXPECT_GT(contact.maximum_sti_n_m, 0);
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

    const auto descriptor = archive::ReadViewerInput(destination, *result.viewer_input);
    const auto archive_path = archive::ViewerArchivePath(destination, descriptor);
    const auto replay = archive::Replay::Open(
        archive_path, descriptor.manifest, descriptor.source, descriptor.mapping_sha256);
    const auto& configuration = replay.configuration();
    const auto& index = replay.index();
    EXPECT_TRUE(configuration.wall);
    EXPECT_TRUE(configuration.profile.self_contact);
    EXPECT_TRUE(configuration.profile.type45);
    EXPECT_TRUE(configuration.profile.beam18);
    EXPECT_TRUE(configuration.profile.structural_limit);
    ASSERT_TRUE(replay.wall());
    ASSERT_TRUE(replay.wall_composition());
    EXPECT_EQ(replay.wall_composition()->profile,
              archive::CompositionProfile::VehicleSupportsV5);
    EXPECT_EQ(replay.wall_composition()->physical_nodes, 376930u);
    EXPECT_EQ(replay.context().parents().size(), 349645u);
    EXPECT_EQ(index.accepted_intervals, 2u);
    EXPECT_FALSE(index.horizon_complete);
    EXPECT_EQ(index.stop_reason, result.loop.reason);

    std::array<archive::Values, 2> rows;
    std::size_t count = 0;
    const auto sequence = archive::ReadIntervals(
        archive_path, replay.context(), configuration.profile,
        index.planned_intervals, index.accepted_intervals, index.segments,
        configuration.request.file_byte_cap, 16u << 20,
        [&](const archive::Values& row) {
            EXPECT_LT(count, rows.size());
            if (count < rows.size()) rows[count++] = row;
        });
    ASSERT_EQ(count, 2u);
    for (std::size_t i = 0; i < rows.size(); ++i)
        ASSERT_NO_FATAL_FAILURE(CheckContactRow(rows[i], replay.context(), i + 1,
            forecast.contact.self_contact->identity.source_id,
            source.self_contact->active_uses().parents().size(), event_capacity));
    EXPECT_LT(rows[0].stamp.attempt, rows[1].stamp.attempt);
    EXPECT_EQ(output::Bits(rows[0].self_contact->base_velocity_time), output::Bits(0.0));
    EXPECT_EQ(output::Bits(rows[1].self_contact->base_velocity_time),
              output::Bits(rows[0].stamp.velocity_time));
    EXPECT_EQ(rows[1].self_contact->events, progress.self_contact.last_event_count);
    EXPECT_EQ(rows[1].self_contact->policy_digest, progress.self_contact.last_policy_digest);
    EXPECT_EQ(output::Bits(rows[1].self_contact->potential_j),
              output::Bits(progress.self_contact.last_accepted_base_potential_j));
    EXPECT_TRUE(records::SameStamp(sequence.last, index.final));
    EXPECT_EQ(sequence.self_source_id, rows[1].self_contact->source_id);

    ASSERT_EQ(index.frames.size(), 2u);
    const auto saved = replay.ReadSample(1);
    EXPECT_TRUE(records::SameStamp(saved.frame.stamp, rows[1].stamp));
    EXPECT_TRUE(records::SameStamp(saved.activity.stamp(), rows[1].stamp));
    const auto summary_bytes = archive::ReadFile(destination, *result.summary, SummaryByteCap);
    output::Document summary;
    summary.Parse(summary_bytes.c_str());
    ASSERT_FALSE(summary.HasParseError());
    ASSERT_TRUE(summary.HasMember("accepted_self_contact"));
    EXPECT_STREQ(summary["contact_profile"].GetString(), "wall-self-contact-v1");
    EXPECT_EQ(summary["accepted_self_contact"]["accepted_intervals"].GetUint64(), 2u);
    EXPECT_EQ(summary["accepted_self_contact"]["last_policy_digest"].GetUint64(),
              rows[1].self_contact->policy_digest);

    ::testing::Test::RecordProperty("accepted_intervals", "2");
    ::testing::Test::RecordProperty("completed_seconds", "0.0000004");
    ::testing::Test::RecordProperty("archive_manifest_sha256", result.archive_manifest->sha256);
    ::testing::Test::RecordProperty("viewer_input_sha256", result.viewer_input->sha256);
    ::testing::Test::RecordProperty("scope", "two committed 200 ns V5 wall+self intervals; no longer-crash claim");
}

} // namespace crash::cases::vehicle_run::test
