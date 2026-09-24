#include "../cli/Options.h"
#include <gtest/gtest.h>
#include <algorithm>
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
TEST(VehicleRunOptions, HalfMillisecondPreviewSelectsFullHorizonWithUnchangedSamplingAndStep) {
    auto args=Arguments();
    args.insert(args.end(),{"--physical-profile","vehicle-supports-v5","--duration-ms","0.5",
        "--fixed-dt-s","2e-7"});
    const auto preview=Parse(args);
    EXPECT_EQ(preview.config.duration_s,.0005);
    EXPECT_EQ(preview.config.fixed_dt_s,2e-7);
    EXPECT_EQ(preview.config.samples,101u);
    EXPECT_EQ(preview.diagnostic_intervals,0u);
    EXPECT_EQ(preview.config.resources,ResourceProfile::Normal);
    EXPECT_EQ(Plan(preview.config).intervals,2501u);
    EXPECT_NE(std::string(cli::Usage()).find("--duration-ms 0.5|5|20|50"),std::string::npos);
    for(const auto* value:{"0.499","0.501","1","nan","inf"}) {
        auto invalid=Arguments();
        invalid.insert(invalid.end(),{"--duration-ms",value});
        EXPECT_THROW(Parse(invalid),std::exception)<<value;
    }
}
TEST(VehicleRunOptions, SelfContactRequiresExplicitSourceAndQualifiedPhysicalStep) {
    auto args=Arguments();
    args.insert(args.end(),{"--contact-profile","wall-self-contact-v1","--self-contact-member","combine.key",
        "--physical-profile","vehicle-supports-v5","--fixed-dt-s","2e-7"});
    const auto options=Parse(args);
    EXPECT_EQ(options.config.contact_profile,ContactProfile::WallSelfContactV1);
    EXPECT_EQ(options.source.self_contact_combine_member,"combine.key");
    EXPECT_EQ(options.config.fixed_dt_s,2e-7);
    EXPECT_EQ(options.config.resources,ResourceProfile::Normal);
    for(const auto* omitted:{"--self-contact-member","--physical-profile","--fixed-dt-s"}) {
        auto invalid=args;
        const auto found=std::find(invalid.begin(),invalid.end(),omitted);
        ASSERT_NE(found,invalid.end());
        invalid.erase(found,found+2);
        EXPECT_THROW(Parse(invalid),std::invalid_argument)<<omitted;
    }
    auto ignored=Arguments();
    ignored.insert(ignored.end(),{"--self-contact-member","combine.key"});
    EXPECT_THROW(Parse(ignored),std::invalid_argument);
    auto unknown=Arguments();
    unknown.insert(unknown.end(),{"--contact-profile","self"});
    EXPECT_THROW(Parse(unknown),std::invalid_argument);
    Config bad=options.config;
    bad.fixed_dt_s=3e-7;
    EXPECT_THROW(Plan(bad),std::invalid_argument);
    bad=options.config;
    bad.contact_profile=static_cast<ContactProfile>(999);
    EXPECT_THROW(Plan(bad),std::invalid_argument);
}
TEST(VehicleRunOptions, OptionalFailureDiagnosticsRequiresSelfContactAndExplicitRunDestination) {
    auto args=Arguments();
    EXPECT_TRUE(Parse(args).failure_output.empty());
    args.insert(args.end(),{"--self-contact-failure-output","new-failure"});
    EXPECT_THROW(Parse(args),std::invalid_argument);
    args.insert(args.end(),{"--contact-profile","wall-self-contact-v1","--self-contact-member","combine.key",
        "--physical-profile","vehicle-supports-v5","--fixed-dt-s","2e-7"});
    EXPECT_EQ(Parse(args).failure_output,"new-failure");
    args.push_back("--forecast-only");
    EXPECT_EQ(Parse(args).failure_output,"new-failure");
    auto no_output=args;
    const auto output=std::find(no_output.begin(),no_output.end(),"--output");
    no_output.erase(output,output+2);
    EXPECT_THROW(Parse(no_output),std::invalid_argument);
    auto empty=args;
    *(std::find(empty.begin(),empty.end(),"--self-contact-failure-output")+1)="";
    EXPECT_THROW(Parse(empty),std::invalid_argument);
    args.insert(args.end(),{"--self-contact-failure-output","duplicate"});
    EXPECT_THROW(Parse(args),std::invalid_argument);
}
TEST(VehicleRunOptions, SelfContactDiagnosticsIsExplicitAndNeverIgnoredByWallOnly) {
    auto args=Arguments();
    EXPECT_FALSE(Parse(args).config.self_contact_diagnostics);
    args.push_back("--self-contact-diagnostics");
    EXPECT_THROW(Parse(args),std::invalid_argument);
    args.insert(args.end(),{"--contact-profile","wall-self-contact-v1","--self-contact-member","combine.key",
        "--physical-profile","vehicle-supports-v5","--fixed-dt-s","2e-7"});
    const auto observed=Parse(args);
    EXPECT_TRUE(observed.config.self_contact_diagnostics);
    EXPECT_EQ(observed.config.fixed_dt_s,2e-7);
    EXPECT_EQ(observed.config.resources,ResourceProfile::Normal);
    args.push_back("--self-contact-diagnostics");
    EXPECT_THROW(Parse(args),std::invalid_argument);
}
TEST(VehicleRunOptions, CudaFacetFiltersAreExplicitAndForecastUsesTheSameOption) {
    auto args=Arguments();
    EXPECT_FALSE(Parse(args).config.self_contact_cuda_facet_filters);
    args.push_back("--self-contact-cuda-facet-filters");
    EXPECT_THROW(Parse(args),std::invalid_argument);
    args.insert(args.end(),{"--contact-profile","wall-self-contact-v1","--self-contact-member","combine.key",
        "--physical-profile","vehicle-supports-v5","--fixed-dt-s","2e-7",
        "--self-contact-diagnostics","--self-contact-failure-output","new-failure"});
    const auto runtime=Parse(args);
    EXPECT_TRUE(runtime.config.self_contact_cuda_facet_filters);
    EXPECT_TRUE(runtime.config.self_contact_diagnostics);
    EXPECT_EQ(runtime.config.resources,ResourceProfile::Normal);
    EXPECT_EQ(runtime.config.fixed_dt_s,2e-7);
    EXPECT_EQ(runtime.failure_output,"new-failure");
    args.push_back("--forecast-only");
    const auto forecast=Parse(args);
    EXPECT_TRUE(forecast.forecast_only);
    EXPECT_EQ(forecast.config.self_contact_cuda_facet_filters,runtime.config.self_contact_cuda_facet_filters);
    EXPECT_EQ(Plan(forecast.config).intervals,Plan(runtime.config).intervals);
    EXPECT_NE(std::string(cli::Usage()).find("--self-contact-cuda-facet-filters"),std::string::npos);
    args.push_back("--self-contact-cuda-facet-filters");
    EXPECT_THROW(Parse(args),std::invalid_argument);
}

} // namespace crash::cases::vehicle_run::test
