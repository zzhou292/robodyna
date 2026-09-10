#include "case/source_assembly_dynamics/NativeGuardAttribution.h"
#include <gtest/gtest.h>
#include <array>
#include <limits>

namespace crash::cases::source_assembly_dynamics::test {
TEST(NativeGuardAttribution, AggregatePriorityThenFirstFailingSourceRetainsMeasuredUnitsAndLimits) {
    std::array<NativeGuardParent,3> parents{{{101,{.9,.4,.5}},{202,{.4,.3,.3}},{303,{.3,.2,.2}}}};
    const NativeGuardLimits limits{.5,.5,.2,.5};std::size_t reads=0;
    const auto at=[&](std::size_t i) {++reads;return parents[i];};
    auto detail=AttributeNativeGuard({.3,.2,.2},limits,parents.size(),at);
    EXPECT_EQ(detail.source,202u);EXPECT_EQ(detail.measured,.4);EXPECT_EQ(detail.limit,.5);EXPECT_EQ(reads,2u);
    detail=AttributeNativeGuard({.9,.2,.2},limits,parents.size(),at);
    EXPECT_EQ(detail.source,101u);EXPECT_EQ(detail.measured,.4);EXPECT_EQ(detail.limit,.5);
    detail=AttributeNativeGuard({.9,.9,.2},limits,parents.size(),at);
    EXPECT_EQ(detail.source,202u);EXPECT_EQ(detail.measured,.2);EXPECT_EQ(detail.limit,.3*.5);
    EXPECT_STREQ(detail.message,"Requested step exceeds native parent DTEL fraction");
}
TEST(NativeGuardAttribution, InvalidDtAndUnmatchedAggregateNeverInventSourceAttribution) {
    std::array<NativeGuardParent,2> parents{{{101,{1,1,1}},{202,{1,1,0}}}};
    const NativeGuardLimits limits{.5,.5,.2,.5};const auto at=[&](std::size_t i) {return parents[i];};
    auto detail=AttributeNativeGuard({1,1,0},limits,parents.size(),at);
    EXPECT_EQ(detail.source,202u);EXPECT_EQ(detail.measured,0);EXPECT_EQ(detail.limit,0);
    EXPECT_STREQ(detail.message,"Native parent DTEL must be finite and positive");
    detail=AttributeNativeGuard({std::numeric_limits<double>::quiet_NaN(),1,1},limits,parents.size(),at);
    EXPECT_EQ(detail.source,0u);EXPECT_TRUE(std::isnan(detail.measured));EXPECT_EQ(detail.limit,.5);
}
}
