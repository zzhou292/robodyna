#include "modelio/type13/ConvertProperty.h"
#include "lib_utest/qualification/type13/Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace crash::modelio::type13 {
TEST(Type13Conversion,OriginalPropertyMatchesQualifiedNativeLiteralFixture) {
    SourceProperty input{7.8e-9,50000,.3,300,5000,5,5,0,0,2e20};ConvertedProperty converted;
    ASSERT_EQ(ConvertProperty(input,converted),native::Status::Success);
    const type13_test::Fixture fixture;const auto expected=fixture.Input(),actual=converted.input();
    EXPECT_EQ(actual.mass_per_length,expected.mass_per_length);EXPECT_EQ(actual.inertia_per_length,expected.inertia_per_length);
    for(unsigned c=0;c<6;++c) {
        EXPECT_EQ(actual.channels[c].curve_index,expected.channels[c].curve_index);
        EXPECT_EQ(actual.channels[c].stiffness,expected.channels[c].stiffness);
    }
    for(unsigned c=0;c<4;++c)for(unsigned k=0;k<5;++k) {
        EXPECT_EQ(actual.curves[c].points[k].x,expected.curves[c].points[k].x);
        EXPECT_EQ(actual.curves[c].points[k].y,expected.curves[c].points[k].y);
    }
    const auto copy=converted;converted.curves[3].points[4].y=123;
    EXPECT_EQ(copy.input().curves[3].points[4].y,expected.curves[3].points[4].y);
}
TEST(Type13Conversion,LateInvalidConversionPreservesOutputAndRetry) {
    SourceProperty input{7.8e-9,50000,.3,300,5000,5,5,0,0,2e20};ConvertedProperty result;
    ASSERT_EQ(ConvertProperty(input,result),native::Status::Success);
    unsigned char before[sizeof(result)];std::memcpy(before,&result,sizeof(result));
    auto invalid=input;invalid.outer1=1e308;
    EXPECT_NE(ConvertProperty(invalid,result),native::Status::Success);
    EXPECT_EQ(std::memcmp(before,&result,sizeof(result)),0);
    invalid=input;invalid.failure_deformation=std::numeric_limits<double>::infinity();
    EXPECT_NE(ConvertProperty(invalid,result),native::Status::Success);
    EXPECT_EQ(std::memcmp(before,&result,sizeof(result)),0);
    EXPECT_EQ(ConvertProperty(input,result),native::Status::Success);
}
}
