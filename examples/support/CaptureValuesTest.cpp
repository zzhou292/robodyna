#include "CaptureSession.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>

namespace robodyna::examples {
namespace {
CaptureOptions Options() {
    CaptureOptions value;
    value.output = "/uncreated-example-output";
    value.chrono_data = "/declared-assets";
    value.steps = 6000;
    value.capture_every = 40;
    value.time_step_s = .001;
    value.source_demo = "unit-fixture";
    value.source_demo_sha256 = std::string(64, 'a');
    return value;
}
TEST(DemoCaptureValues, InitialAndFinalFramesAndForecastAreExplicit) {
    auto options = Options();
    EXPECT_EQ(CaptureFrameCount(options), 151u);
    EXPECT_NO_THROW(ValidateCaptureOptions(options));
    options.steps = 3000; options.capture_every = 20; options.time_step_s = .002;
    EXPECT_EQ(CaptureFrameCount(options), 151u);
    options.steps = 2000; options.capture_every = 20; options.time_step_s = .003;
    EXPECT_EQ(CaptureFrameCount(options), 101u);
    options.steps = 2001;
    EXPECT_THROW(ValidateCaptureOptions(options), std::exception);
    options.steps = 191; options.capture_every = 1;
    EXPECT_THROW(ValidateCaptureOptions(options), std::exception);
    options.steps = 190;
    EXPECT_NO_THROW(ValidateCaptureOptions(options));
}
TEST(DemoCaptureValues, NativeClockAllowsAccumulationRoundingButRejectsDrift) {
    double accumulated = 0;
    for (std::uint64_t i = 0; i <= 6000; ++i) {
        EXPECT_TRUE(CaptureTimeMatches(i, accumulated, .001));
        accumulated += .001;
    }
    EXPECT_FALSE(CaptureTimeMatches(6000, 6.001, .001));
    EXPECT_FALSE(CaptureTimeMatches(1, std::numeric_limits<double>::quiet_NaN(), .001));
    EXPECT_FALSE(CaptureTimeMatches(UINT64_MAX, 1, .001));
}
class DemoHeadlessTest : public ::testing::Test {
  protected:
    void SetUp() override {
        static std::atomic<unsigned> next{0};
        root = std::filesystem::temp_directory_path() /
            ("robodyna-demo-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
             "-" + std::to_string(next++));
        ASSERT_TRUE(std::filesystem::create_directory(root));
        ASSERT_TRUE(std::filesystem::create_directory(root / "assets"));
        options = Options(); options.output = root / "run"; options.chrono_data = root / "assets";
        options.steps = 2; options.capture_every = 1; options.headless = true;
        system.SetGravitationalAcceleration(chrono::VNULL);
        system.AddBody(chrono_types::make_shared<chrono::ChBody>());
    }
    void TearDown() override { std::filesystem::remove_all(root); }
    std::filesystem::path root;
    CaptureOptions options;
    chrono::ChSystemNSC system;
};
// No VSG construction or initialization: exercise the actual system attachment
// contract through the existing CPU-only visual base.
class AttachmentOnlyVisual : public chrono::ChVisualSystem {
  public:
    AttachmentOnlyVisual() = default;
    bool setup_called = false;
  protected:
    void OnSetup(chrono::ChSystem*) override { setup_called = true; }
};
TEST_F(DemoHeadlessTest, RejectsAttachedVisualBeforeFirstDynamicsStep) {
    CaptureSession capture(options);
    {
        AttachmentOnlyVisual visual;
        visual.AttachSystem(&system);
        ASSERT_EQ(system.GetVisualSystem(), &visual);
        ASSERT_FALSE(visual.IsInitialized());
        EXPECT_THROW(capture.ConfigureVisual(visual), std::exception);
        EXPECT_THROW(capture.State(0, system, nullptr), std::exception);
        EXPECT_EQ(system.GetNumSteps(), 0u);
        EXPECT_DOUBLE_EQ(system.GetChTime(), 0);
        EXPECT_FALSE(visual.setup_called);
        EXPECT_FALSE(visual.IsInitialized());
    }
    ASSERT_EQ(system.GetVisualSystem(), nullptr);
    EXPECT_NO_THROW(capture.State(0, system, nullptr));
}
TEST_F(DemoHeadlessTest, RecordsRealStepsWithoutRendererAndCannotOverwrite) {
    CaptureSession capture(options);
    EXPECT_THROW(CaptureSession duplicate(options), std::exception);
    capture.State(0, system, nullptr);
    EXPECT_THROW(capture.State(0, system, nullptr), std::exception);
    crash::output::Document metrics;
    metrics.SetObject();
    EXPECT_THROW(capture.Finish(system, metrics), std::exception);
    for (unsigned step = 1; step <= 2; ++step) {
        ASSERT_NE(system.DoStepDynamics(options.time_step_s), 0);
        capture.State(step, system, nullptr);
    }
    capture.Finish(system, metrics);
    EXPECT_TRUE(std::filesystem::is_regular_file(options.output / "run-summary.json"));
    EXPECT_FALSE(std::filesystem::exists(options.output / "manifest.json"));
    EXPECT_FALSE(std::filesystem::exists(options.output / "frames.csv"));
    EXPECT_THROW(capture.Finish(system, metrics), std::exception);
}
}  // namespace
}  // namespace robodyna::examples
