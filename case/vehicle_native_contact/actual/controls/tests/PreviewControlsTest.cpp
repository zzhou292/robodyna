#include "../PreviewControls.h"
#include "case/vehicle_run/Loop.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <vector>
namespace crash::cases::vehicle_native_contact::test {
namespace {
namespace run = vehicle_run;
struct PrefixOperations final : run::detail::Operations {
    run::Endpoint accepted;
    double elapsed_s = 0;
    std::uint64_t appended = 0, captured = 0;
    std::vector<std::uint64_t> saved;
    unsigned finished = 0, discarded = 0;
    bool complete = true;
    std::string reason;
    std::function<void()> after_append;
    run::Endpoint Accepted() const noexcept override { return accepted; }
    void Prepare() override { elapsed_s += 2; }
    void Commit() override { ++accepted.epoch; accepted.time_s += .001; }
    void Discard() noexcept override { ++discarded; }
    void Append() override { appended = accepted.epoch; if (after_append) after_append(); }
    void Capture() override { captured = accepted.epoch; }
    void SaveSample() override {
        EXPECT_EQ(captured, accepted.epoch);
        EXPECT_EQ(appended, accepted.epoch);
        saved.push_back(accepted.epoch);
    }
    void Finish(bool horizon_complete, const std::string& stop_reason) override {
        ASSERT_FALSE(saved.empty());
        EXPECT_EQ(saved.back(), accepted.epoch);
        ++finished;
        complete = horizon_complete;
        reason = stop_reason;
    }
};
run::LoopResult Execute(PrefixOperations& operations, const PreviewControls& controls) {
    const run::Horizon horizon{5, .005, .001, .005L};
    return run::detail::RunLoop(operations, horizon, {0, 2, 5}, MakePreviewControl(controls),
        [&] { return operations.elapsed_s; });
}
class NativePreviewControls : public ::testing::Test {
  protected:
    void SetUp() override {
        auto pattern = (std::filesystem::temp_directory_path() / "robo-native-preview-controls-XXXXXX").string();
        const auto* created = ::mkdtemp(pattern.data());
        ASSERT_NE(created, nullptr);
        root = created;
    }
    void TearDown() override {
        if (!root.empty()) { std::error_code ignored; std::filesystem::remove_all(root, ignored); }
    }
    std::filesystem::path root;
};
TEST_F(NativePreviewControls, DefaultsAndExplicitZeroLeaveTheExistingLoopUnbounded) {
    for (const auto* limit : {static_cast<const char*>(nullptr), "0"}) {
        const auto options = ParsePreviewControls(limit, nullptr);
        const auto control = MakePreviewControl(options);
        EXPECT_EQ(control.maximum_elapsed_s, 0);
        EXPECT_EQ(control.maximum_accepted_intervals, 0u);
        EXPECT_EQ(control.progress_period_s, 5);
        EXPECT_FALSE(control.stop_requested);
        PrefixOperations operations;
        const auto result = Execute(operations, options);
        EXPECT_EQ(result.kind, run::StopKind::Completed);
        EXPECT_TRUE(result.valid_manifest);
        EXPECT_TRUE(operations.complete);
        EXPECT_EQ(operations.saved, (std::vector<std::uint64_t>{0, 2, 5}));
    }
}
TEST_F(NativePreviewControls, ExplicitArtifactLimitIsStrictAndDoesNotChangeRunControls) {
    EXPECT_FALSE(ParsePreviewControls(nullptr, nullptr).artifact_file_bytes);
    const auto selected = ParsePreviewControls(nullptr, nullptr, nullptr, "25165824");
    ASSERT_TRUE(selected.artifact_file_bytes);
    EXPECT_EQ(*selected.artifact_file_bytes, 24u << 20);
    const auto controls = MakePreviewControl(selected);
    EXPECT_EQ(controls.maximum_elapsed_s, 0);
    EXPECT_EQ(controls.maximum_accepted_intervals, 0u);
    EXPECT_FALSE(controls.stop_requested);
    EXPECT_FALSE(selected.stage_timing);
    for (const auto* text : {"", "0", "-1", "+1", "25165824 ", " 25165824",
                            "24MiB", "2.5", "18446744073709551616"})
        EXPECT_THROW(ParsePreviewControls(nullptr, nullptr, nullptr, text), std::exception) << text;
}
TEST_F(NativePreviewControls, MalformedLimitsAndEmptyOrOversizePathsRejectBeforeRunning) {
    for (const auto* value : {"", "-1", "nan", "inf", "1.2seconds", "3 ", "1e9999"})
        EXPECT_THROW(ParsePreviewControls(value, nullptr), std::exception) << value;
    const std::string too_long(4097, '1');
    EXPECT_THROW(ParsePreviewControls(too_long.c_str(), nullptr), std::invalid_argument);
    EXPECT_THROW(ParsePreviewControls(nullptr, ""), std::invalid_argument);
    EXPECT_THROW(ParsePreviewControls(nullptr, too_long.c_str()), std::invalid_argument);
    EXPECT_DOUBLE_EQ(ParsePreviewControls("1.25e2", nullptr).maximum_elapsed_s, 125);
}
TEST_F(NativePreviewControls, TimeLimitSealsTheLastAcceptedOffCadenceSample) {
    PrefixOperations operations;
    const auto result = Execute(operations, ParsePreviewControls("5", nullptr));
    EXPECT_EQ(result.kind, run::StopKind::TimeLimit);
    EXPECT_TRUE(result.valid_manifest);
    EXPECT_EQ(result.progress.accepted.epoch, 3u);
    EXPECT_EQ(operations.saved, (std::vector<std::uint64_t>{0, 2, 3}));
    EXPECT_EQ(operations.finished, 1u);
    EXPECT_EQ(operations.discarded, 0u);
    EXPECT_FALSE(operations.complete);
    EXPECT_FALSE(operations.reason.empty());
}
TEST_F(NativePreviewControls, StopFileIsObservedAtTheAcceptedBoundaryAndNeverConsumed) {
    const auto path = root / "stop";
    const auto options = ParsePreviewControls(nullptr, path.c_str());
    auto control = MakePreviewControl(options);
    ASSERT_TRUE(control.stop_requested);
    EXPECT_FALSE(control.stop_requested());
    PrefixOperations operations;
    operations.after_append = [&] {
        if (operations.accepted.epoch == 3) { std::ofstream signal(path); signal << "stop\n"; }
    };
    const auto result = Execute(operations, options);
    EXPECT_EQ(result.kind, run::StopKind::Requested);
    EXPECT_TRUE(result.valid_manifest);
    EXPECT_EQ(operations.saved, (std::vector<std::uint64_t>{0, 2, 3}));
    EXPECT_EQ(operations.finished, 1u);
    EXPECT_FALSE(operations.complete);
    EXPECT_TRUE(std::filesystem::exists(path));
    EXPECT_TRUE(control.stop_requested());
}
TEST_F(NativePreviewControls, ExistingStopFileSealsInitialStateWithoutAttemptingAnInterval) {
    const auto path = root / "stop";
    { std::ofstream signal(path); signal << "already requested\n"; }
    PrefixOperations operations;
    const auto result = Execute(operations, ParsePreviewControls(nullptr, path.c_str()));
    EXPECT_EQ(result.kind, run::StopKind::Requested);
    EXPECT_TRUE(result.valid_manifest);
    EXPECT_EQ(result.progress.accepted.epoch, 0u);
    EXPECT_EQ(operations.saved, (std::vector<std::uint64_t>{0}));
    EXPECT_EQ(operations.elapsed_s, 0);
    EXPECT_EQ(operations.finished, 1u);
    EXPECT_FALSE(operations.complete);
}
} // namespace
} // namespace crash::cases::vehicle_native_contact::test
