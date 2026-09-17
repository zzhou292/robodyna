#include "../RunState.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <fstream>
namespace crash::cases::vehicle_run::test {
namespace {
output::Document Read(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::string bytes((std::istreambuf_iterator<char>(input)),{});
    output::Document result;
    result.Parse(bytes.c_str());
    if(result.HasParseError()) throw std::runtime_error("Summary JSON failed to parse");
    return result;
}
}
TEST(VehicleRunSummary, StartupFailureDoesNotInventAcceptedTimeOrContactObservations) {
    records::test::Directory directory;
    Result result;
    result.loop.kind=StopKind::StartupFailure;
    result.loop.reason="Actual startup rejection";
    Config config;
    const auto record=detail::WriteSummary(directory.path,config,Plan(config),{},result);
    const auto document=Read(directory.path/record.file);
    EXPECT_FALSE(document["session_initialized"].GetBool());
    EXPECT_STREQ(document["physical_profile"].GetString(),"retained-shell-v1");
    EXPECT_FALSE(document.HasMember("actual_completed_time_s"));
    EXPECT_FALSE(document.HasMember("contact_observations_available"));
    EXPECT_FALSE(document.HasMember("archive_manifest_file"));
    EXPECT_LE(record.bytes,SummaryByteCap);
    EXPECT_THROW(detail::WriteSummary(directory.path,config,Plan(config),{},result),std::exception);
}
TEST(VehicleRunSummary, AcceptedPrefixPreservesLimitContactAndViewerAuthority) {
    records::test::Directory directory;
    Result result;
    result.session_initialized=true;
    result.loop.kind=StopKind::PhysicsRejected;
    result.loop.valid_manifest=true;
    result.loop.reason="Actual timestep rejection";
    result.loop.progress.accepted={2,6e-7};
    result.loop.progress.contact.available=true;
    result.loop.progress.contact.peak_observed_force_n=2;
    result.loop.progress.contact.reported_drift_work_sum_j=-.125;
    auto& sampled = result.loop.progress.sampled_shell_plasticity;
    sampled.available = sampled.native_fields_available = sampled.positive_sample_observed = true;
    sampled.saved_samples = 2;
    sampled.native_points = 8;
    sampled.last_epoch = sampled.first_positive_saved_epoch = 2;
    sampled.last_attempt = 3;
    sampled.last_time_s = sampled.first_positive_saved_time_s = 6e-7;
    sampled.last_positive_points = 1;
    sampled.last_max_native_equivalent_plastic_strain = sampled.peak_saved_native_equivalent_plastic_strain = .25;
    result.rejected_step_limit_s=2e-7;
    result.rejected_node=17;
    records::RecordFile manifest;
    manifest.file="manifest.json";
    manifest.sha256=std::string(64,'a');
    result.archive_manifest=manifest;
    records::RecordFile viewer;
    viewer.file="viewer-input.json";
    viewer.sha256=std::string(64,'b');
    result.viewer_input=viewer;
    Config config;
    config.physical_profile=PhysicalProfile::ExtendedSolidsV4;
    Forecast forecast;
    forecast.joint_count=40;
    const auto file=detail::WriteSummary(directory.path,config,Plan(config),forecast,result);
    const auto document=Read(directory.path/file.file);
    EXPECT_EQ(document["accepted_intervals"].GetUint64(),2u);
    EXPECT_STREQ(document["physical_profile"].GetString(),"extended-solids-v4");
    EXPECT_EQ(document["selected_joints"].GetUint64(),40u);
    EXPECT_EQ(document["actual_completed_time_s"].GetDouble(),6e-7);
    EXPECT_EQ(document["requested_duration_s"].GetDouble(),.005);
    EXPECT_EQ(document["rejected_step_limit_s"].GetDouble(),2e-7);
    EXPECT_EQ(document["rejected_physical_node"].GetUint64(),17u);
    EXPECT_EQ(document["reported_signed_drift_work_sum_j"].GetDouble(),-.125);
    EXPECT_STREQ(document["archive_manifest_file"].GetString(),"archive/manifest.json");
    EXPECT_STREQ(document["viewer_input_file"].GetString(),"viewer-input.json");
    EXPECT_FALSE(document.HasMember("total_energy_j"));
    const auto& saved_shell = document["sampled_shell_plasticity"];
    EXPECT_EQ(saved_shell["last_saved_epoch"].GetUint64(), 2u);
    EXPECT_EQ(saved_shell["first_positive_saved_time_s"].GetDouble(), 6e-7);
    EXPECT_EQ(saved_shell["last_saved_positive_points"].GetUint64(), 1u);
    EXPECT_LT(file.bytes, SummaryByteCap);
}
TEST(VehicleRunSummary, SelfContactProfilePreservesTypedRejectionAndQualifiedStepBoundary) {
    records::test::Directory directory;
    Config config;
    config.physical_profile=PhysicalProfile::VehicleSupportsV5;
    config.contact_profile=ContactProfile::WallSelfContactV1;
    config.fixed_dt_s=2e-7;
    Result result;
    result.loop.kind=StopKind::PhysicsRejected;
    result.rejected_step_limit_s=1e-7;
    Forecast forecast;
    forecast.contact.device_bytes=1234;
    const auto file=detail::WriteSummary(directory.path,config,Plan(config),forecast,result);
    const auto document=Read(directory.path/file.file);
    EXPECT_STREQ(document["schema"].GetString(),"robo_dyna.vehicle_run_summary.v2");
    EXPECT_STREQ(document["contact_profile"].GetString(),"wall-self-contact-v1");
    EXPECT_EQ(document["complete_device_bytes"].GetUint64(),1234u);
    EXPECT_EQ(document["fixed_dt_s"].GetDouble(),2e-7);
    EXPECT_STREQ(document["recovery"].GetString(),
        "stop and qualify a revised contact/timestep profile before starting a new run");
    EXPECT_FALSE(document.HasMember("accepted_self_contact"));
}
} // namespace crash::cases::vehicle_run::test
