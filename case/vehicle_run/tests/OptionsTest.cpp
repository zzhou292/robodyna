#include "../cli/Options.h"
#include <gtest/gtest.h>
#include <vector>
namespace crash::cases::vehicle_run::test {
namespace {
std::vector<std::string> Arguments() {
    return {"run","--canonical","canonical","--scope","scope.json","--member","main.key",
        "--declarations","decl.json","--glass-resolution","glass.json","--type13","type13.json",
        "--aux-member","aux.key","--original-wall-member","wall.key","--wall-manifest","wall.json",
        "--output","run-output","--run-id","71"};
}
cli::Options Parse(const std::vector<std::string>& strings) {
    std::vector<const char*> arguments;
    for(const auto& text:strings) arguments.push_back(text.c_str());
    return cli::Parse(static_cast<int>(arguments.size()),arguments.data());
}
}
TEST(VehicleRunOptions, ExplicitCompleteSourceNormalDefaultsAndDiagnosticPrefix) {
    auto args=Arguments();
    const auto defaults=Parse(args);
    EXPECT_EQ(defaults.config.duration_s,.005);
    EXPECT_EQ(defaults.config.fixed_dt_s,3e-7);
    EXPECT_EQ(defaults.gap_m,.02);
    EXPECT_FALSE(defaults.wall_stiffness_n_m3);
    EXPECT_FALSE(defaults.penetration_limit_m);
    EXPECT_EQ(defaults.run_id,71u);
    EXPECT_EQ(defaults.diagnostic_intervals,0u);
    EXPECT_EQ(defaults.config.resources,ResourceProfile::Normal);
    EXPECT_EQ(defaults.config.physical_profile,PhysicalProfile::RetainedShellAssembliesV1);
    args.insert(args.end(),{"--duration-ms","50","--diagnostic-intervals","2","--gap-m","0.000001"});
    const auto short_run=Parse(args);
    EXPECT_EQ(short_run.config.duration_s,.05);
    EXPECT_EQ(short_run.diagnostic_intervals,2u);
    EXPECT_EQ(short_run.gap_m,1e-6);
}
TEST(VehicleRunOptions, ExplicitPhysicalProfileIsIndependentOfResourceAllowance) {
    auto args=Arguments();
    args.insert(args.end(),{"--physical-profile","extended-solids-v4"});
    const auto options=Parse(args);
    EXPECT_EQ(options.config.physical_profile,PhysicalProfile::ExtendedSolidsV4);
    EXPECT_EQ(options.config.resources,ResourceProfile::Normal);
    EXPECT_EQ(options.config.duration_s,.005);
    for(const auto* value:{"", "v4", "full-vehicle", "retained-shell-v2"}) {
        auto invalid=Arguments();
        invalid.insert(invalid.end(),{"--physical-profile",value});
        EXPECT_THROW(Parse(invalid),std::invalid_argument);
    }
    args.insert(args.end(),{"--physical-profile","retained-shell-v1"});
    EXPECT_THROW(Parse(args),std::invalid_argument);
    Config bad;
    bad.physical_profile=static_cast<PhysicalProfile>(999);
    EXPECT_THROW(Plan(bad),std::invalid_argument);
}
TEST(VehicleRunOptions, MissingDuplicateOverflowAndInvalidNumericsRejectBeforeAnySourceRead) {
    for(const auto& tail:std::vector<std::vector<std::string>>{{"--run-id","72"},{"--samples","18446744073709551616"},
        {"--fixed-dt-s","nan"},{"--duration-ms","6"},{"--maximum-elapsed-s","-1"},{"--unknown","1"},{"--gap-m"}}) {
        auto args=Arguments();
        args.insert(args.end(),tail.begin(),tail.end());
        EXPECT_THROW(Parse(args),std::exception);
    }
    EXPECT_THROW(Parse({"run","--forecast-only"}),std::exception);
}
TEST(VehicleRunOptions, ExplicitWallControlsKeepUnitsAndRejectInvalidValuesBeforeSourceRead) {
    auto args=Arguments();
    args.insert(args.end(),{"--wall-stiffness-n-m3","1e9","--penetration-limit-m","0.002"});
    const auto options=Parse(args);
    ASSERT_TRUE(options.wall_stiffness_n_m3);
    ASSERT_TRUE(options.penetration_limit_m);
    EXPECT_EQ(*options.wall_stiffness_n_m3,1e9);
    EXPECT_EQ(*options.penetration_limit_m,.002);
    for(const auto* name:{"--wall-stiffness-n-m3","--penetration-limit-m"}) {
        for(const auto* value:{"0","-1","nan","inf","1e999"}) {
            auto invalid=Arguments();
            invalid.insert(invalid.end(),{name,value});
            EXPECT_THROW(Parse(invalid),std::exception)<<name<<' '<<value;
        }
        auto duplicate=args;
        duplicate.insert(duplicate.end(),{name,"1"});
        EXPECT_THROW(Parse(duplicate),std::exception);
    }
}
TEST(VehicleRunOptions, VehicleSupportsV5IsExplicitAndKeepsNormalResourceDefaults) {
    auto args=Arguments();
    args.insert(args.end(),{"--physical-profile","vehicle-supports-v5"});
    const auto options=Parse(args);
    EXPECT_EQ(options.config.physical_profile,PhysicalProfile::VehicleSupportsV5);
    EXPECT_STREQ(PhysicalProfileName(options.config.physical_profile),"vehicle-supports-v5");
    EXPECT_EQ(options.config.resources,ResourceProfile::Normal);
    EXPECT_EQ(options.config.samples,101u);
    EXPECT_EQ(options.config.fixed_dt_s,3e-7);
    EXPECT_NE(std::string(cli::Usage()).find("vehicle-supports-v5"),std::string::npos);
    args.insert(args.end(),{"--physical-profile","extended-solids-v4"});
    EXPECT_THROW(Parse(args),std::invalid_argument);
}
} // namespace crash::cases::vehicle_run::test
