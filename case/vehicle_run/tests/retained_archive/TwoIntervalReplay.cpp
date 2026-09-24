#include "TwoIntervalReplay.h"
#include "output/physical_run/IntervalIO.h"
#include "output/BoundedArrayJson.h"
#include <gtest/gtest.h>
#include <array>

namespace crash::cases::vehicle_run::test::retained {
namespace archive = output::physical_run;
namespace records = output::full_shell;
namespace {
void CheckContactRow(const archive::Values& row,
                     const records::Context& context,
                     std::uint64_t epoch,
                     std::uint64_t source_id,
                     std::size_t selected_parents,
                     std::size_t event_capacity) {
    EXPECT_EQ(row.owner, context.identity().owner);
    EXPECT_EQ(row.stamp.epoch, epoch);
    EXPECT_EQ(row.stamp.base_epoch, epoch - 1);
    EXPECT_EQ(output::Bits(row.stamp.time), output::Bits(epoch * StepS));
    EXPECT_EQ(output::Bits(row.stamp.base_time),
              output::Bits((epoch - 1) * StepS));
    EXPECT_GT(row.stamp.attempt, 0u);
    ASSERT_TRUE(row.structural_limit_s);
    EXPECT_GE(*row.structural_limit_s, StepS);
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

void CheckTwoIntervalArchive(const std::filesystem::path& destination,
    const records::RecordFile& viewer_input, const records::RecordFile& summary_file,
    const Expected& expected) {
    ASSERT_GT(expected.source_id, 0u);
    ASSERT_GT(expected.selected_parents, 0u);
    ASSERT_GT(expected.event_capacity, 0u);
    const auto descriptor = archive::ReadViewerInput(destination, viewer_input);
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
    EXPECT_EQ(index.stop_reason, expected.stop_reason);

    const auto staging_bytes = archive::IntervalReadStagingBytes(
        configuration.profile, index.planned_intervals, configuration.request.file_byte_cap);
    EXPECT_LE(staging_bytes, 256u << 20);
    ::testing::Test::RecordProperty("interval_read_staging_bytes", std::to_string(staging_bytes));
    ::testing::Test::RecordProperty("replay_peak_host_bytes", std::to_string(replay.peak_host_bytes()));
    std::array<archive::Values, 2> rows;
    std::size_t count = 0;
    const auto sequence = archive::ReadIntervals(
        archive_path, replay.context(), configuration.profile,
        index.planned_intervals, index.accepted_intervals, index.segments,
        configuration.request.file_byte_cap, staging_bytes,
        [&](const archive::Values& row) {
            EXPECT_LT(count, rows.size());
            if (count < rows.size()) rows[count++] = row;
        });
    ASSERT_EQ(count, 2u);
    for (std::size_t i = 0; i < rows.size(); ++i)
        ASSERT_NO_FATAL_FAILURE(CheckContactRow(rows[i], replay.context(), i + 1,
            expected.source_id,
            expected.selected_parents, expected.event_capacity));
    EXPECT_LT(rows[0].stamp.attempt, rows[1].stamp.attempt);
    EXPECT_EQ(output::Bits(rows[0].self_contact->base_velocity_time), output::Bits(0.0));
    EXPECT_EQ(output::Bits(rows[1].self_contact->base_velocity_time),
              output::Bits(rows[0].stamp.velocity_time));
    EXPECT_EQ(rows[1].self_contact->events, expected.last_event_count);
    EXPECT_EQ(rows[1].self_contact->policy_digest, expected.last_policy_digest);
    EXPECT_EQ(output::Bits(rows[1].self_contact->potential_j),
              output::Bits(expected.last_potential_j));
    EXPECT_TRUE(records::SameStamp(sequence.last, index.final));
    EXPECT_EQ(sequence.self_source_id, rows[1].self_contact->source_id);

    ASSERT_EQ(index.frames.size(), 2u);
    EXPECT_EQ(index.frames.front().stamp.epoch, 0u);
    EXPECT_EQ(index.frames.back().stamp.epoch, 2u);
    EXPECT_EQ(output::Bits(configuration.request.fixed_dt), output::Bits(StepS));
    EXPECT_EQ(output::Bits(index.final.time), output::Bits(2 * StepS));
    const auto saved = replay.ReadSample(1);
    EXPECT_TRUE(records::SameStamp(saved.frame.stamp, rows[1].stamp));
    EXPECT_TRUE(records::SameStamp(saved.activity.stamp(), rows[1].stamp));
    const auto summary_bytes = archive::ReadFile(destination, summary_file, archive::MetadataCap);
    const auto summary = output::array_json::Parse(summary_bytes, archive::MetadataCap);
    for (const auto* field : {"contact_profile", "accepted_self_contact",
            "archive_manifest_file", "archive_manifest_sha256"})
        ASSERT_TRUE(summary.HasMember(field)) << field;
    EXPECT_EQ(output::array_json::Text(summary["archive_manifest_sha256"]), descriptor.manifest.sha256);
    EXPECT_EQ(output::array_json::Text(summary["archive_manifest_file"]),
              descriptor.archive_directory + "/" + descriptor.manifest.file);
    EXPECT_STREQ(summary["contact_profile"].GetString(), "wall-self-contact-v1");
    EXPECT_EQ(summary["accepted_self_contact"]["accepted_intervals"].GetUint64(), 2u);
    EXPECT_EQ(summary["accepted_self_contact"]["last_policy_digest"].GetUint64(),
              rows[1].self_contact->policy_digest);

    ::testing::Test::RecordProperty("accepted_intervals", "2");
    ::testing::Test::RecordProperty("completed_seconds", "0.0000004");
    ::testing::Test::RecordProperty("archive_manifest_sha256", descriptor.manifest.sha256);
    ::testing::Test::RecordProperty("viewer_input_sha256", viewer_input.sha256);
    ::testing::Test::RecordProperty("summary_sha256", summary_file.sha256);
    ::testing::Test::RecordProperty("scope", "read-only replay of two committed 200 ns V5 wall+self intervals; no new dynamics, restart or longer-crash claim");
}
} // namespace crash::cases::vehicle_run::test::retained
