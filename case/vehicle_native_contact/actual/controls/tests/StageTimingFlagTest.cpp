#include "../PreviewControls.h"
#include <gtest/gtest.h>
#include <stdexcept>
namespace crash::cases::vehicle_native_contact::test {
TEST(NativePreviewStageTiming, DefaultAndStrictBinaryFlagDoNotChangeCooperativeControls) {
    EXPECT_FALSE(ParsePreviewControls(nullptr, nullptr).stage_timing);
    EXPECT_FALSE(ParsePreviewControls(nullptr, nullptr, "0").stage_timing);
    EXPECT_TRUE(ParsePreviewControls(nullptr, nullptr, "1").stage_timing);
    const auto options = ParsePreviewControls("12.5", "/tmp/robo-preview-stop", "1");
    const auto control = MakePreviewControl(options);
    EXPECT_TRUE(options.stage_timing);
    EXPECT_EQ(control.maximum_elapsed_s, 12.5);
    EXPECT_EQ(options.stop_file, "/tmp/robo-preview-stop");
    EXPECT_TRUE(control.stop_requested);
    EXPECT_EQ(control.maximum_accepted_intervals, 0u);
    EXPECT_EQ(control.progress_period_s, 5);
}
TEST(NativePreviewStageTiming, NonBinaryOrEmptyExplicitValuesReject) {
    for (const auto* value : {"", "true", "false", "01", "00", "1.0", "-1", "2", " 1", "1 ", "1\n"})
        EXPECT_THROW(ParsePreviewControls(nullptr, nullptr, value), std::invalid_argument) << value;
}
} // namespace crash::cases::vehicle_native_contact::test
