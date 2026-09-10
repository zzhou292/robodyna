#include "case/source_assembly_wall/CliOptions.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace crash::cases::source_assembly_wall {
namespace {
CliOptions Parse(std::initializer_list<const char*> extra={}) {
    std::vector<const char*> args{"program","inventory.json","wall.json","128","1","new-archive"};
    args.insert(args.end(),extra);return ParseOptions(static_cast<int>(args.size()),args.data());
}
TEST(PilotOptions, DefaultsAndExistingRefinementRetainExactBinaryStep) {
    const auto defaults=Parse();EXPECT_EQ(PilotFixedStep(defaults.pilot),std::ldexp(1.,-26));
    EXPECT_EQ(defaults.steps,128u);EXPECT_EQ(defaults.frame_every,1u);EXPECT_EQ(defaults.archive,"new-archive");
    EXPECT_EQ(defaults.pilot.refinement,1u);EXPECT_EQ(defaults.pilot.step_multiple,1u);
    EXPECT_FALSE(defaults.pilot.timing.enabled);EXPECT_TRUE(defaults.timing_path.empty());
    EXPECT_EQ(PilotFixedStep(Parse({"2"}).pilot),std::ldexp(1.,-27));
    EXPECT_EQ(PilotFixedStep(Parse({"4"}).pilot),std::ldexp(1.,-28));
}
TEST(PilotOptions, AllTwelveAllowedPairsGiveExactRequestedPowerOfTwoStep) {
    for(unsigned r:{1u,2u,4u})for(unsigned m:{1u,2u,4u,8u}) {
        PilotOptions o;o.refinement=r;o.step_multiple=m;
        const auto expected=std::ldexp(1.,-26+int(std::log2(m))-int(std::log2(r)));
        EXPECT_EQ(PilotFixedStep(o),expected);o.timing.enabled=true;EXPECT_EQ(PilotFixedStep(o),expected);
    }
}
TEST(PilotOptions, RejectsUnadmittedRefinementAndMultipliersThroughDirectApi) {
    for(unsigned r:{0u,3u,5u,std::numeric_limits<unsigned>::max()}) {
        PilotOptions o;o.refinement=r;EXPECT_THROW(PilotFixedStep(o),std::invalid_argument);
    }
    for(unsigned m:{0u,3u,5u,6u,7u,16u,std::numeric_limits<unsigned>::max()}) {
        PilotOptions o;o.step_multiple=m;EXPECT_THROW(PilotFixedStep(o),std::invalid_argument);
    }
}
TEST(PilotOptions, NamedOptionsInEitherOrderKeepTypedTimingAndMultiple) {
    const auto a=Parse({"4","--stage-timing","timing.json","--step-multiple","8"});
    const auto b=Parse({"4","--step-multiple","8","--stage-timing","timing.json"});
    EXPECT_EQ(PilotFixedStep(a.pilot),std::ldexp(1.,-25));EXPECT_EQ(PilotFixedStep(b.pilot),PilotFixedStep(a.pilot));
    EXPECT_EQ(a.timing_path,"timing.json");EXPECT_EQ(a.timing_path,b.timing_path);
    EXPECT_TRUE(a.pilot.timing.enabled);EXPECT_TRUE(b.pilot.timing.enabled);
    EXPECT_EQ(Parse({"--step-multiple","2"}).pilot.step_multiple,2u);
    EXPECT_TRUE(Parse({"--stage-timing","timing.json"}).pilot.timing.enabled);
}
TEST(PilotOptions, RejectsMissingRepeatedUnknownOrMalformedNamedOptions) {
    EXPECT_THROW(Parse({"--stage-timing"}),std::invalid_argument);
    EXPECT_THROW(Parse({"--step-multiple"}),std::invalid_argument);
    EXPECT_THROW(Parse({"--stage-timing","--step-multiple"}),std::invalid_argument);
    EXPECT_THROW(Parse({"--stage-timing",""}),std::invalid_argument);
    EXPECT_THROW(Parse({"--stage-timing","a","--stage-timing","b"}),std::invalid_argument);
    EXPECT_THROW(Parse({"--step-multiple","1","--step-multiple","2"}),std::invalid_argument);
    EXPECT_THROW(Parse({"--wrong","8"}),std::invalid_argument);
    EXPECT_THROW(Parse({"--step-multiple=8"}),std::invalid_argument);
    for(const char* value:{"0","3","16","-1","8x","999999999999999999999999"})
        EXPECT_THROW(Parse({"--step-multiple",value}),std::invalid_argument);
    EXPECT_THROW(Parse({"3"}),std::invalid_argument);
    EXPECT_THROW(Parse({"--step-multiple","8","2"}),std::invalid_argument);
}
TEST(PilotOptions, InvalidCountsNullArgumentsAndPartialInputFailBeforePublication) {
    EXPECT_THROW(ParseOptions(6,nullptr),std::invalid_argument);
    const char* args[]{"program","inventory","wall","0","1","archive"};
    EXPECT_THROW(ParseOptions(6,args),std::invalid_argument);
    args[3]="129";args[4]="130";EXPECT_THROW(ParseOptions(6,args),std::invalid_argument);
    args[3]="1048577";args[4]="1";EXPECT_THROW(ParseOptions(6,args),std::invalid_argument);
    args[3]="128";args[2]=nullptr;EXPECT_THROW(ParseOptions(6,args),std::invalid_argument);
    EXPECT_THROW(ParseOptions(5,args),std::invalid_argument);
}
}
}
