// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace type13_test {
TEST(Type13Reference,ExplicitN3AndBothNativeFallbacks) {
  t::ReferenceInput in;in.position[1]={2,0,0};in.position[2]={0,1,1};t::Reference result;
  ASSERT_EQ(t::InitializeReference({1000,.001,1},in,result),t::Status::Success);
  EXPECT_EQ(result.branch,t::FrameBranch::ThirdNode);
  EXPECT_EQ(result.length_native,2);EXPECT_EQ(result.length_m,.002);
  EXPECT_GT(result.axes.v[7],0);
  in.position[2]={4,0,0};
  ASSERT_EQ(t::InitializeReference({1000,.001,1},in,result),t::Status::Success);
  EXPECT_EQ(result.branch,t::FrameBranch::SkewY);
  in.position[1]={0,2,0};in.position[2]={0,4,0};
  ASSERT_EQ(t::InitializeReference({1000,.001,1},in,result),t::Status::Success);
  EXPECT_EQ(result.branch,t::FrameBranch::SkewX);
  EXPECT_EQ(result.axes.v[1],1);
}

TEST(Type13Reference,NativeN3ThresholdUsesUnnormalizedWorkingUnitSeed) {
  t::ReferenceInput in;in.position[1]={2,0,0};in.position[2]={0,100000,100000};
  t::Reference large,small;
  ASSERT_EQ(t::InitializeReference({1000,.001,1},in,large),t::Status::Success);
  in.position[2]={0,1,1};
  ASSERT_EQ(t::InitializeReference({1000,.001,1},in,small),t::Status::Success);
  EXPECT_EQ(large.branch,t::FrameBranch::SkewY);
  EXPECT_EQ(small.branch,t::FrameBranch::ThirdNode);
  EXPECT_LT(large.third_node_alignment,1e-5);
  EXPECT_GT(small.third_node_alignment,1e-5);
}

TEST(Type13Reference,LateCoefficientFailureNoiseAndReleaseRejectionPreserveOutput) {
  Fixture fixture;t::Property property;ASSERT_EQ(t::InitializeProperty(fixture.Input(),property),t::Status::Success);
  auto in=DenseReference();t::Startup result;
  ASSERT_EQ(t::InitializeElement(property,in,result),t::Status::Success);
  unsigned char before[sizeof(result)];std::memcpy(before,&result,sizeof(result));
  in.endpoint_release[3]=1;
  EXPECT_EQ(t::InitializeElement(property,in,result),t::Status::UnsupportedScope);
  EXPECT_EQ(std::memcmp(before,&result,sizeof(result)),0);
  in=DenseReference();in.coordinate_noise=2;
  EXPECT_EQ(t::InitializeElement(property,in,result),t::Status::DegenerateGeometry);
  in=DenseReference();in.position[2]=in.position[0];
  EXPECT_EQ(t::InitializeElement(property,in,result),t::Status::DegenerateGeometry);
  auto declaration=fixture.Input();declaration.mass_per_length=1e308;
  ASSERT_EQ(t::InitializeProperty(declaration,property),t::Status::Success);
  EXPECT_EQ(t::InitializeElement(property,DenseReference(),result),t::Status::NonfiniteResult);
  EXPECT_EQ(std::memcmp(before,&result,sizeof(result)),0);
  ASSERT_EQ(t::InitializeProperty(fixture.Input(),property),t::Status::Success);
  EXPECT_EQ(t::InitializeElement(property,DenseReference(),result),t::Status::Success);
}
} // namespace type13_test
