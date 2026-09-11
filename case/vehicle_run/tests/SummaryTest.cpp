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
    const auto file=detail::WriteSummary(directory.path,config,Plan(config),{},result);
    const auto document=Read(directory.path/file.file);
    EXPECT_EQ(document["accepted_intervals"].GetUint64(),2u);
    EXPECT_EQ(document["actual_completed_time_s"].GetDouble(),6e-7);
    EXPECT_EQ(document["requested_duration_s"].GetDouble(),.005);
    EXPECT_EQ(document["rejected_step_limit_s"].GetDouble(),2e-7);
    EXPECT_EQ(document["rejected_physical_node"].GetUint64(),17u);
    EXPECT_EQ(document["reported_signed_drift_work_sum_j"].GetDouble(),-.125);
    EXPECT_STREQ(document["archive_manifest_file"].GetString(),"archive/manifest.json");
    EXPECT_STREQ(document["viewer_input_file"].GetString(),"viewer-input.json");
    EXPECT_FALSE(document.HasMember("total_energy_j"));
}
} // namespace crash::cases::vehicle_run::test
