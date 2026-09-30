#include "chrono/ReplayParentScalarColors.h"
#include "ColorChecks.h"
#include <limits>

namespace crash::visual::full_shell::test {
using A = output::ReplayScalarApplicability;
TEST(ReplayScalarApplicability,LegacyValuesKeepExactRampAndDefaultMeaning) {
    ReplayParentScalarColors colors;
    std::vector<output::ReplayParentScalar> fields{{1,0},{2,.25},{3,.5},{4,.75},{5,1}};
    std::vector<chrono::ChColor> actual;
    ASSERT_TRUE(colors.Initialize({1,2,3,4,5},fields,1,actual));
    const chrono::ChColor blue(.12f,.64f,.94f),yellow(.98f,.84f,.16f),red(.90f,.12f,.10f);
    for(std::size_t i=0;i<fields.size();++i) {
        EXPECT_EQ(fields[i].applicability,A::NativeValue);
        const auto expected=i<=2?chrono::ChColor::Interp(blue,yellow,.5*i):chrono::ChColor::Interp(yellow,red,.5*(i-2));
        EXPECT_EQ(actual[i].R,expected.R);EXPECT_EQ(actual[i].G,expected.G);EXPECT_EQ(actual[i].B,expected.B);
    }
    EXPECT_EQ(colors.legend().native,5u);EXPECT_EQ(colors.legend().unavailable,0u);
}
TEST(ReplayScalarApplicability,MissingFieldsHaveDistinctColorsAndImmutableTags) {
    ReplayParentScalarColors colors;
    std::vector<output::ReplayParentScalar> fields{{71,.2,A::NativeValue},{72,0,A::NotApplicable},{73,0,A::Unavailable}};
    std::vector<chrono::ChColor> actual;
    ASSERT_TRUE(colors.Initialize({71,71,72,73},fields,1,actual));
    EXPECT_EQ(colors.legend().native,1u);EXPECT_EQ(colors.legend().not_applicable,1u);EXPECT_EQ(colors.legend().unavailable,1u);
    ExpectColor(actual[0],actual[1]);ExpectColor(actual[2],ReplayMissingScalarColor(A::NotApplicable));
    ExpectColor(actual[3],ReplayMissingScalarColor(A::Unavailable));EXPECT_FALSE(SameColor(actual[2],ReplayScalarColor(0)));
    EXPECT_FALSE(SameColor(actual[2],actual[3]));
    const auto before=actual;
    fields.back().applicability=A::NativeValue;
    EXPECT_FALSE(colors.Stage(fields,actual));ExpectColors(actual,before);
    fields.back().applicability=A::Unavailable;fields.back().value=.1;
    EXPECT_FALSE(colors.Stage(fields,actual));ExpectColors(actual,before);
    fields.back().value=0;fields.front().value=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(colors.Stage(fields,actual));ExpectColors(actual,before);
    fields.front().value=.4;ASSERT_TRUE(colors.Stage(fields,actual));
    ExpectColor(actual[0],ReplayScalarColor(.4));
}
TEST(ReplayScalarApplicability,NoNativePointsRequireNoScalarScaleAndUnknownTagsReject) {
    ReplayParentScalarColors colors;
    std::vector<chrono::ChColor> actual;
    EXPECT_FALSE(colors.Stage({},actual));
    std::vector<output::ReplayParentScalar> fields{{1,0,A::NotApplicable},{2,0,A::Unavailable}};
    EXPECT_FALSE(colors.Initialize({1,2},fields,1,actual));
    ASSERT_TRUE(colors.Initialize({1,2},fields,0,actual));
    EXPECT_EQ(colors.legend().native,0u);EXPECT_EQ(colors.legend().maximum,0);
    const auto before=actual;
    fields.back().applicability=static_cast<A>(99);
    EXPECT_FALSE(colors.Stage(fields,actual));ExpectColors(actual,before);
    EXPECT_FALSE(ValidReplayScalar(fields.back()));EXPECT_EQ(ReplayApplicabilityName(fields.back().applicability),nullptr);
}
} // namespace crash::visual::full_shell::test
