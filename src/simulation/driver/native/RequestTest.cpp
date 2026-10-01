#include "Request.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <fstream>

namespace robodyna::driver {
namespace {
const char* ValidRequest = R"json({
  "schema":"robodyna.native_vehicle_request.v1", "profile":"yaris.native_v6.wall_self",
  "source_paths":{"canonical":"/source/canonical","scope":"/source/scope","member":"/source/member",
    "declarations":"/source/declarations","glass_resolution":"/source/glass","type13":"/source/type13",
    "auxiliary_member":"/source/aux","original_wall_member":"/source/wall","wall_manifest":"/source/wall.json",
    "self_contact_combine_member":"/source/combine"},
  "solid_packets":{"path":"/source/packets","bytes":123,"sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"},
  "run":{"duration_s":0.1,"fixed_dt_s":0.00000015,"samples":301,"contact_activity":"shell_removal",
    "stage_timing":false,"capture_qeph_rejection":true,"verify_initial_retry":false},
  "resources":{"schema":"robodyna.resources.v1","cpu_threads":4,"rss_bytes":19327352832,
    "minimum_available_ram_bytes":34359738368,"gpu_index":0,"minimum_gpu_free_bytes":8589934592,
    "maximum_gpu_growth_bytes":9663676416,"timeout_s":68400,"cooperative_maximum_elapsed_s":64800,
    "stop_grace_s":30,"archive_bytes":6442450944,"artifact_file_bytes":25165824,
    "solid_worker_blocks":32,"workstation_lock":"/reports/workstation.lock"},
  "output":"/runs/example/accepted","report":"/runs/example/native-report.json","stop_file":"/runs/example/stop.requested"
})json";

class NativeRequestTest : public ::testing::Test {
  protected:
    void SetUp() override {
        static std::atomic<unsigned> sequence{0};
        const auto clock = std::chrono::steady_clock::now().time_since_epoch().count();
        directory_ = std::filesystem::temp_directory_path() /
            ("robodyna-request-" + std::to_string(clock) + "-" + std::to_string(sequence++));
        ASSERT_TRUE(std::filesystem::create_directory(directory_));
    }
    void TearDown() override { std::filesystem::remove_all(directory_); }
    Request Parse(const std::string& text, bool correct_hash = true) {
        const auto path = directory_ / "request.json";
        std::ofstream(path, std::ios::binary | std::ios::trunc) << text;
        return ReadRequest(path, correct_hash ? crash::output::Sha256(text) : std::string(64, '0'));
    }
    std::string Replace(const std::string& before, const std::string& after) {
        std::string result(ValidRequest);
        const auto position = result.find(before);
        EXPECT_NE(position, std::string::npos);
        result.replace(position, before.size(), after);
        return result;
    }
    std::filesystem::path directory_;
};

TEST_F(NativeRequestTest, ReadsOnlyConfigurationWithoutTouchingSourcesOrDestination) {
    const auto request = Parse(ValidRequest);
    EXPECT_EQ(request.run.samples, 301u);
    EXPECT_EQ(request.run.fixed_dt_s, 1.5e-7);
    EXPECT_EQ(request.resources.solid_worker_blocks, 32u);
    EXPECT_EQ(request.output, "/runs/example/accepted");
    EXPECT_EQ(std::distance(std::filesystem::directory_iterator(directory_), std::filesystem::directory_iterator()), 1);
}
TEST_F(NativeRequestTest, RejectsWrongRequestIdentity) {
    EXPECT_THROW(Parse(ValidRequest, false), std::exception);
}
TEST_F(NativeRequestTest, RejectsDuplicateBooleanAndUnknownProfileInputs) {
    EXPECT_THROW(Parse(Replace("\"samples\":301", "\"samples\":301,\"samples\":3")), std::exception);
    EXPECT_THROW(Parse(Replace("\"samples\":301", "\"samples\":true")), std::exception);
    EXPECT_THROW(Parse(Replace("yaris.native_v6.wall_self", "invented.profile")), std::exception);
}
TEST_F(NativeRequestTest, RejectsResourceUnderflowAndUnsupportedWorkerCount) {
    EXPECT_THROW(Parse(Replace("\"rss_bytes\":19327352832", "\"rss_bytes\":1")), std::exception);
    EXPECT_THROW(Parse(Replace("\"solid_worker_blocks\":32", "\"solid_worker_blocks\":33")), std::exception);
    EXPECT_THROW(Parse(Replace("\"cooperative_maximum_elapsed_s\":64800", "\"cooperative_maximum_elapsed_s\":68400")), std::exception);
}
TEST_F(NativeRequestTest, RejectsUnrelatedOutputDestinationsAndRelativeSourcePaths) {
    EXPECT_THROW(Parse(Replace("/runs/example/native-report.json", "/elsewhere/native-report.json")), std::exception);
    EXPECT_THROW(Parse(Replace("/source/member", "relative/member")), std::exception);
}
}  // namespace
}  // namespace robodyna::driver
