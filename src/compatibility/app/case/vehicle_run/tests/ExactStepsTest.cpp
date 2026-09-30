#include "../Config.h"
#include "output/full_shell/FixedStepHorizon.h"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
namespace crash::cases::vehicle_run::test {
namespace horizon = output::full_shell;
TEST(VehicleRunExactSteps, SixThousandKeepsExactCountAndArchiveHorizonAtBothSteps) {
    for (double dt : {2e-7, 2.25e-7}) {
        Config config;
        config.exact_steps = 6000;
        config.samples = 51;
        config.fixed_dt_s = dt;
        const auto plan = Plan(config);
        EXPECT_EQ(plan.intervals, 6000u);
        EXPECT_EQ(plan.fixed_dt_s, dt);
        EXPECT_TRUE(horizon::MatchesFixedStepHorizon(6000, dt, plan.requested_duration_s));
        EXPECT_FALSE(horizon::MatchesFixedStepHorizon(5999, dt, plan.requested_duration_s));
        EXPECT_FALSE(horizon::MatchesFixedStepHorizon(6001, dt, plan.requested_duration_s));
        std::uint64_t roundtrip = 0;
        ASSERT_TRUE(horizon::PlanFixedStepHorizon(dt, plan.requested_duration_s, roundtrip));
        EXPECT_EQ(roundtrip, 6000u);
        EXPECT_EQ(plan.nominal_endpoint_s, static_cast<long double>(dt) * 6000);
    }
}
TEST(VehicleRunExactSteps, InvalidOrUnrepresentableHorizonPreservesOutput) {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double dt : {0., -1., inf, nan, std::numeric_limits<double>::max()}) {
        double duration = 123.;
        EXPECT_FALSE(horizon::PlanExactStepHorizon(dt, 6000, duration));
        EXPECT_EQ(duration, 123.);
    }
    for (std::uint64_t count : {std::uint64_t{0}, UINT64_MAX, UINT64_MAX / 2 + 1,
                                (std::uint64_t{1} << 53) + 1}) {
        double duration = 123.;
        EXPECT_FALSE(horizon::PlanExactStepHorizon(1., count, duration));
        EXPECT_EQ(duration, 123.);
    }
    // (2^53, 2^53+1] contains no binary64 value; exact count must not round.
    double duration = 0;
    ASSERT_TRUE(horizon::PlanExactStepHorizon(1., std::uint64_t{1} << 53, duration));
    EXPECT_EQ(duration, 9007199254740992.);
}
TEST(VehicleRunExactSteps, SamplesAndWallEnvelopeRemainBounded) {
    Config config;
    config.exact_steps = 1;
    config.samples = 2;
    EXPECT_EQ(Plan(config).intervals, 1u);
    config.samples = 3;
    EXPECT_THROW(Plan(config), std::invalid_argument);
    config.samples = 2;
    config.exact_steps = 6000;
    config.fixed_dt_s = 1.;
    EXPECT_THROW(Plan(config), std::invalid_argument);
    config.fixed_dt_s = 0.;
    EXPECT_THROW(Plan(config), std::invalid_argument);
    config = Config{};
    EXPECT_EQ(Plan(config).intervals, 16667u); // Zero remains the duration-mode sentinel.
    config.duration_s = .006;
    EXPECT_THROW(Plan(config), std::invalid_argument);
}
} // namespace crash::cases::vehicle_run::test
