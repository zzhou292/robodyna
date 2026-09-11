// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace type13_test {
TEST(Type13Property,OriginalResolvedPropertyOwnsFourCurvesForSixChannels) {
  Fixture fixture;t::Property property;
  ASSERT_EQ(t::InitializeProperty(fixture.Input(),property),t::Status::Success);
  EXPECT_TRUE(property.initialized());
  EXPECT_DOUBLE_EQ(property.mass_per_length(),1.5315264186251252e-7);
  EXPECT_DOUBLE_EQ(property.inertia_per_length(),4.786020058203517e-7);
  EXPECT_EQ(property.added_inertia_per_length(),0);
  for(unsigned c=0;c<6;++c) {
    EXPECT_GE(property.channel(c).native_stiffness,fixture.Input().channels[c].stiffness);
    EXPECT_EQ(property.channel(c).stiffness_si,
      property.channel(c).native_stiffness*(c<3?1.0:1e-6));
  }
  EXPECT_EQ(property.channel(1).declaration.curve_index,property.channel(2).declaration.curve_index);
  EXPECT_EQ(property.channel(4).declaration.curve_index,property.channel(5).declaration.curve_index);
  const auto copy=property;
  fixture.points[3][4].y=123;
  EXPECT_EQ(copy.curve(3).points[4].y,property.curve(3).points[4].y);
  EXPECT_NE(copy.curve(3).points[4].y,fixture.points[3][4].y);
}

TEST(Type13Property,LastSegmentCanSetStiffnessForBothSharedChannels) {
  Fixture fixture;auto input=fixture.Input();
  fixture.points[3][4].y=1e10;
  t::Property result;
  ASSERT_EQ(t::InitializeProperty(input,result),t::Status::Success);
  const double slope=(fixture.points[3][4].y-fixture.points[3][3].y)/
                     (fixture.points[3][4].x-fixture.points[3][3].x);
  EXPECT_EQ(result.channel(4).native_stiffness,slope);
  EXPECT_EQ(result.channel(5).native_stiffness,slope);
  EXPECT_GT(slope,input.channels[5].stiffness*100);
}

TEST(Type13Property,LateInvalidCurveOrChannelPreservesOwnedOutputAndRetry) {
  Fixture fixture;auto input=fixture.Input();t::Property result;
  ASSERT_EQ(t::InitializeProperty(input,result),t::Status::Success);
  unsigned char before[sizeof(result)];std::memcpy(before,&result,sizeof(result));
  const auto last=fixture.points[3][4];
  fixture.points[3][4].y=std::numeric_limits<double>::infinity();
  EXPECT_EQ(t::InitializeProperty(input,result),t::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(before,&result,sizeof(result)),0);
  fixture.points[3][4]=last;input.channels[5].curve_index=4;
  EXPECT_EQ(t::InitializeProperty(input,result),t::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(before,&result,sizeof(result)),0);
  input.channels[5].curve_index=3;
  ASSERT_EQ(t::InitializeProperty(input,result),t::Status::Success);
  EXPECT_EQ(result.curve(3).points[4].y,last.y);
}

TEST(Type13Property,CountsScopeAndIntermediateOverflowRejectWithoutPublication) {
  Fixture fixture;auto input=fixture.Input();t::Property output;
  input.curves[0].points=nullptr;input.curves[3].count=std::numeric_limits<std::size_t>::max();
  EXPECT_EQ(t::InitializeProperty(input,output),t::Status::InvalidInput);
  EXPECT_FALSE(output.initialized());
  input=fixture.Input();input.controls.length_normalized=0;
  EXPECT_EQ(t::InitializeProperty(input,output),t::Status::UnsupportedScope);
  input=fixture.Input();input.channels[5].hysteresis=3;
  EXPECT_EQ(t::InitializeProperty(input,output),t::Status::UnsupportedScope);
  input=fixture.Input();fixture.points[3][0].x=-1e308;fixture.points[3][1].x=1e308;
  fixture.points[3][2].x=1.1e308;fixture.points[3][3].x=1.2e308;fixture.points[3][4].x=1.3e308;
  EXPECT_EQ(t::InitializeProperty(input,output),t::Status::NonfiniteResult);
  EXPECT_FALSE(output.initialized());
}

TEST(Type13Property,NativeInertiaFloorIsSeparateFromSuppliedPropertyContribution) {
  Fixture fixture;auto input=fixture.Input();input.inertia_per_length=0;
  t::Property property;
  ASSERT_EQ(t::InitializeProperty(input,property),t::Status::Success);
  EXPECT_EQ(property.inertia_per_length(),1e-20);
  EXPECT_EQ(property.added_inertia_per_length(),1e-20);
  t::Startup output;
  ASSERT_EQ(t::InitializeElement(property,DenseReference(),output),t::Status::Success);
  EXPECT_EQ(output.endpoint.isotropic_inertia_kg_m2,output.endpoint.added_inertia_kg_m2);
  EXPECT_GT(output.endpoint.mass_kg,0);
}
} // namespace type13_test
