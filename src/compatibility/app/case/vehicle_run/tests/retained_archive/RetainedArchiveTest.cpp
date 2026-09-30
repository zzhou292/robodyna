#include "TwoIntervalReplay.h"
#include "output/BoundedArrayJson.h"
#include <gtest/gtest.h>
#include <cstdlib>

namespace crash::cases::vehicle_run::test::retained {
namespace {
namespace archive = output::physical_run;
namespace records = output::full_shell;
const char* Environment(const char* name) {
    const auto* value = std::getenv(name);
    output::Require(value && *value, "Retained replay requires explicit directory and SHA-256 pins");
    return value;
}
records::RecordFile Pinned(const std::filesystem::path& directory,
                          const char* name, const char* hash_name, std::size_t cap) {
    const auto path = directory / name;
    output::Require(std::filesystem::symlink_status(path).type() ==
                        std::filesystem::file_type::regular,
                    "Retained replay input must be an existing regular nonsymlink file");
    const auto size = std::filesystem::file_size(path);
    output::Require(size > 0 && size <= cap, "Retained replay input exceeds its existing metadata cap");
    return {name, Environment(hash_name), static_cast<std::size_t>(size)};
}
}

TEST(VehicleRunRetainedArchive, TwoCommittedV5IntervalsMatchPinnedSummaryAndSource) {
    const auto directory = std::filesystem::path(Environment("ROBO_RETAINED_V5_RUN"));
    ASSERT_EQ(std::filesystem::symlink_status(directory).type(), std::filesystem::file_type::directory);
    const auto viewer = Pinned(directory, "viewer-input.json", "ROBO_RETAINED_V5_VIEWER_SHA256",
                               archive::ViewerInputByteCap);
    const auto summary_file = Pinned(directory, "run-summary.json", "ROBO_RETAINED_V5_SUMMARY_SHA256",
                                     archive::MetadataCap);
    const auto summary = output::array_json::Parse(
        archive::ReadFile(directory, summary_file, archive::MetadataCap), archive::MetadataCap);
    using namespace output::array_json;
    for (const auto* field : {"schema", "status", "physical_profile", "contact_profile",
            "session_initialized", "valid_archive_manifest", "accepted_intervals",
            "fixed_dt_s", "actual_completed_time_s", "viewer_input_file",
            "viewer_input_sha256", "accepted_self_contact", "reason"})
        ASSERT_TRUE(summary.HasMember(field)) << field;
    ASSERT_EQ(Text(summary["schema"]), "robo_dyna.vehicle_run_summary.v2");
    ASSERT_EQ(Text(summary["status"]), "diagnostic_interval_limit");
    ASSERT_EQ(Text(summary["physical_profile"]), "vehicle-supports-v5");
    ASSERT_EQ(Text(summary["contact_profile"]), "wall-self-contact-v1");
    ASSERT_TRUE(summary["session_initialized"].IsBool() && summary["session_initialized"].GetBool());
    ASSERT_TRUE(summary["valid_archive_manifest"].IsBool() && summary["valid_archive_manifest"].GetBool());
    ASSERT_EQ(UInt(summary["accepted_intervals"]), 2u);
    ASSERT_EQ(output::Bits(Real(summary["fixed_dt_s"])), output::Bits(StepS));
    ASSERT_EQ(output::Bits(Real(summary["actual_completed_time_s"])), output::Bits(2 * StepS));
    ASSERT_EQ(Text(summary["viewer_input_file"]), viewer.file);
    ASSERT_EQ(Text(summary["viewer_input_sha256"]), viewer.sha256);
    ASSERT_TRUE(summary.HasMember("accepted_self_contact"));
    const auto& contact = summary["accepted_self_contact"];
    ASSERT_TRUE(contact.IsObject());
    for (const auto* field : {"available", "accepted_intervals", "last_event_count",
            "last_policy_digest", "last_accepted_base_potential_j"})
        ASSERT_TRUE(contact.HasMember(field)) << field;
    ASSERT_TRUE(contact["available"].IsBool() && contact["available"].GetBool());
    ASSERT_EQ(UInt(contact["accepted_intervals"]), 2u);
    // Explicit selected V5 qualification contract from the preserved producer:
    // RDSELFV1, 337092 selected parents, 65536 reserved events. These constants
    // constrain this test; no source is loaded and no runtime owner is created.
    const Expected expected{
        0x524453454c465631ull, 337092, 65536,
        UInt(contact["last_event_count"]), UInt(contact["last_policy_digest"]),
        Real(contact["last_accepted_base_potential_j"]), Text(summary["reason"])};
    ASSERT_NO_FATAL_FAILURE(CheckTwoIntervalArchive(directory, viewer, summary_file, expected));
}
} // namespace crash::cases::vehicle_run::test::retained
