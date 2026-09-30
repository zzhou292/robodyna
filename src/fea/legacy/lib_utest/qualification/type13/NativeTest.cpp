// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace type13_test {
TEST(Type13Native,OriginalAndRaisedLastSlopeMatchRetainedRkini3) {
  Fixture fixture;
  for(unsigned pass=0;pass<2;++pass) {
    if(pass)fixture.points[3][4].y=1e10;
    auto in=fixture.Input();t::Property property;
    ASSERT_EQ(t::InitializeProperty(in,property),t::Status::Success);
    for(unsigned c=0;c<6;++c)EXPECT_EQ(property.channel(c).native_stiffness,
      NativeSlope(fixture.points[in.channels[c].curve_index],in.channels[c].stiffness));
  }
}

TEST(Type13Native,ExplicitThirdNodeAndSkewFallbackFramesMatchR4buf3) {
  t::ReferenceInput cases[4]={DenseReference(),{},{},{}};
  cases[1].position[1]={2,0,0};cases[1].position[2]={4,0,0};
  cases[2].position[1]={0,2,0};cases[2].position[2]={0,4,0};
  cases[3].position[1]={2,0,0};cases[3].position[2]={0,100000,100000};
  for(const auto& input:cases) {
    t::Reference actual;ASSERT_EQ(t::InitializeReference({1000,.001,1},input,actual),t::Status::Success);
    const auto native=NativeReference(input);
    EXPECT_EQ(actual.length_native,native.length);
    EXPECT_EQ(static_cast<int>(actual.branch),native.branch);
    for(unsigned i=0;i<3;++i)EXPECT_EQ(actual.axes.v[3*i+1],native.y[i]);
  }
}

TEST(Type13Native,RmassAndStarterFloorWithIndependentDimensionalOracle) {
  Fixture fixture;
  for(double source_j:{fixture.Input().inertia_per_length,0.0,1e-21}) {
    auto in=fixture.Input();in.inertia_per_length=source_j;t::Property property;
    ASSERT_EQ(t::InitializeProperty(in,property),t::Status::Success);
    t::Startup actual;ASSERT_EQ(t::InitializeElement(property,DenseReference(),actual),t::Status::Success);
    const auto native=NativeMass(in.mass_per_length,source_j,actual.reference.length_native);
    EXPECT_EQ(property.inertia_per_length(),native[2]);
    EXPECT_EQ(actual.endpoint.mass_kg,native[0]*1000);
    EXPECT_EQ(actual.endpoint.isotropic_inertia_kg_m2,native[1]*.001);
    const long double x=1.25L,y=2,z=2.5L;
    const long double length=std::sqrt(x*x+y*y+z*z);
    const long double oracle_mass=.5L*in.mass_per_length*length*1000;
    const long double oracle_j=.5L*std::max(source_j,1e-20)*length*.001L;
    EXPECT_NEAR(actual.endpoint.mass_kg,static_cast<double>(oracle_mass),
                8*std::numeric_limits<double>::epsilon()*actual.endpoint.mass_kg);
    EXPECT_NEAR(actual.endpoint.isotropic_inertia_kg_m2,static_cast<double>(oracle_j),
                8*std::numeric_limits<double>::epsilon()*actual.endpoint.isotropic_inertia_kg_m2);
  }
}
} // namespace type13_test
