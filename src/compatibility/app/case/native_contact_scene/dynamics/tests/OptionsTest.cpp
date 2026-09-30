#include "../Options.h"
#include <gtest/gtest.h>
#include <string>
#include <vector>
namespace crash::cases::native_scene::cli {
namespace {
std::vector<std::string> Required() {
    return {"robo_dyna_native_scene_run","--source","input/declared-scene.json",
        "--source-sha256",std::string(64,'a'),"--source-output","new-static",
        "--output","new-run","--run-id","71"};
}
Options ParseArguments(const std::vector<std::string>& args) {
    std::vector<const char*> pointers;for(const auto& arg:args)pointers.push_back(arg.c_str());
    return Parse(int(pointers.size()),pointers.data());
}
Options Extra(std::initializer_list<std::string> extra) {
    auto args=Required();args.insert(args.end(),extra.begin(),extra.end());return ParseArguments(args);
}
}
TEST(NativeSceneCli, ExplicitSourceAndDefaultNumericalProfileAreRetained) {
    const auto options=ParseArguments(Required());
    EXPECT_EQ(options.source,"input/declared-scene.json");EXPECT_EQ(options.source_sha256,std::string(64,'a'));
    EXPECT_EQ(options.source_output,"new-static");EXPECT_EQ(options.output,"new-run");
    EXPECT_EQ(options.config.run_id,71u);EXPECT_EQ(options.config.steps,1000u);EXPECT_EQ(options.config.samples,31u);
    EXPECT_EQ(options.config.dynamics.fixed_dt,3e-7);EXPECT_NE(options.config.dynamics.configuration,0u);
    EXPECT_NE(options.config.dynamics.qualification,0u);EXPECT_FALSE(options.forecast_only);
    EXPECT_EQ(options.control.maximum_accepted_intervals,0u);EXPECT_EQ(options.control.maximum_elapsed_s,0);
}
TEST(NativeSceneCli, DiagnosticControlsDoNotChangePhysicalHorizonOrSource) {
    const auto options=Extra({"--forecast-only","--steps","1000","--samples","101","--fixed-dt-s","3e-7",
        "--diagnostic-intervals","20","--maximum-elapsed-s","12.5","--stop-file","stop.request"});
    EXPECT_TRUE(options.forecast_only);EXPECT_EQ(options.config.steps,1000u);EXPECT_EQ(options.config.samples,101u);
    EXPECT_EQ(options.config.dynamics.fixed_dt,3e-7);EXPECT_EQ(options.control.maximum_accepted_intervals,20u);
    EXPECT_EQ(options.control.maximum_elapsed_s,12.5);EXPECT_EQ(options.stop_file,"stop.request");
    // Parse only records the path. The CLI binds the actual existence callback.
    EXPECT_FALSE(bool(options.control.stop_requested));
    const auto short_run=Extra({"--steps","3","--samples","4"});EXPECT_EQ(short_run.config.steps,3u);
}
TEST(NativeSceneCli, MissingAuthenticationAndConflictingDestinationsAreRejected) {
    auto args=Required();args[4]="bad";EXPECT_THROW(ParseArguments(args),std::exception);
    args=Required();args[4]=std::string(64,'g');EXPECT_THROW(ParseArguments(args),std::exception);
    args=Required();args[2]="";EXPECT_THROW(ParseArguments(args),std::exception);
    args=Required();args[8]="./new-static";EXPECT_THROW(ParseArguments(args),std::exception);
    args=Required();args[10]="0";EXPECT_THROW(ParseArguments(args),std::exception);
    args=Required();args.erase(args.begin()+3,args.begin()+5);EXPECT_THROW(ParseArguments(args),std::exception);
}
TEST(NativeSceneCli, DuplicateUnknownMissingAndOversizedArgumentsAreRejected) {
    EXPECT_THROW(Extra({"--steps","2","--steps","3"}),std::exception);
    EXPECT_THROW(Extra({"--forecast-only","--forecast-only"}),std::exception);
    EXPECT_THROW(Extra({"--not-a-profile","1"}),std::exception);
    EXPECT_THROW(Extra({"--steps"}),std::exception);
    EXPECT_THROW(Extra({"--stop-file",std::string(4097,'x')}),std::exception);
    EXPECT_THROW(Extra({std::string(4097,'x'),"1"}),std::exception);
    EXPECT_THROW(Parse(0,nullptr),std::exception);
    EXPECT_THROW(Parse(49,nullptr),std::exception);
}
TEST(NativeSceneCli, NumericGarbageOverflowAndNonfiniteValuesAreRejected) {
    for(const auto* value:{"-1","1.5","12junk","18446744073709551616",""}) {
        SCOPED_TRACE(value);EXPECT_THROW(Extra({"--steps",value}),std::exception);
    }
    for(const auto* value:{"nan","inf","-inf","1e999","3e-7junk",""}) {
        SCOPED_TRACE(value);EXPECT_THROW(Extra({"--fixed-dt-s",value}),std::exception);
    }
}
TEST(NativeSceneCli, BasicDomainsRejectBeforeStaticSourceWritingOrForecastMode) {
    for(const auto* value:{"0","-3e-7"})EXPECT_THROW(Extra({"--fixed-dt-s",value}),std::exception);
    for(const auto* value:{"0","1000001"})EXPECT_THROW(Extra({"--steps",value}),std::exception);
    for(const auto* value:{"0","1","1001"})EXPECT_THROW(Extra({"--samples",value}),std::exception);
    EXPECT_THROW(Extra({"--steps","3","--samples","5"}),std::exception);
    EXPECT_THROW(Extra({"--maximum-elapsed-s","-1","--forecast-only"}),std::exception);
}
} // namespace crash::cases::native_scene::cli
