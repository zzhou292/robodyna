#include "../Horizon.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
namespace crash::cases::vehicle_native_contact::test {
TEST(NativeOutputHorizon, ExistingHorizonsRetainTheOriginalExactStepCount) {
    constexpr double step = 1.5e-7;
    for (const double duration : {2 * step, .0005, .002, .005, .01, .03, .05}) {
        SCOPED_TRACE(duration);
        std::uint64_t original = 0, admitted = 0;
        ASSERT_TRUE(output::full_shell::PlanFixedStepHorizon(step, duration, original));
        ASSERT_TRUE(run_detail::PlanOutputHorizon(step, duration, admitted));
        EXPECT_EQ(admitted, original);
    }
}
TEST(NativeOutputHorizon, HundredMillisecondsUsesTheFirstEndpointAtOrAfterTheRequest) {
    constexpr double step = 1.5e-7;
    std::uint64_t intervals = 0;
    ASSERT_TRUE(run_detail::PlanOutputHorizon(step, .1, intervals));
    EXPECT_EQ(intervals, 666667u);
    EXPECT_TRUE(output::full_shell::MatchesFixedStepHorizon(intervals, step, .1));
    EXPECT_FALSE(output::full_shell::MatchesFixedStepHorizon(intervals - 1, step, .1));
    EXPECT_EQ(run_detail::MaximumOutputDurationSeconds, .1);
}
TEST(NativeOutputHorizon, OverLimitAndInvalidDurationsLeaveTheOutputUntouched) {
    constexpr double invalid = std::numeric_limits<double>::quiet_NaN();
    constexpr double infinity = std::numeric_limits<double>::infinity();
    for (const double duration : {0., -.1, invalid, infinity, -infinity,
                                 std::nextafter(.1, infinity), .1001, 1.}) {
        SCOPED_TRACE(duration);
        std::uint64_t intervals = 77;
        EXPECT_FALSE(run_detail::PlanOutputHorizon(1.5e-7, duration, intervals));
        EXPECT_EQ(intervals, 77u);
    }
}
TEST(NativeOutputHorizon, InvalidTimestepsLeaveTheOutputUntouched) {
    constexpr double invalid = std::numeric_limits<double>::quiet_NaN();
    constexpr double infinity = std::numeric_limits<double>::infinity();
    for (const double step : {0., -1.5e-7, invalid, infinity, -infinity,
                              std::numeric_limits<double>::denorm_min()}) {
        SCOPED_TRACE(step);
        std::uint64_t intervals = 77;
        EXPECT_FALSE(run_detail::PlanOutputHorizon(step, .1, intervals));
        EXPECT_EQ(intervals, 77u);
    }
}
} // namespace crash::cases::vehicle_native_contact::test
