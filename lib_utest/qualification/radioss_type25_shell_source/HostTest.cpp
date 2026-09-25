// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25ShellSource.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
using namespace tlfea::contact::radioss_type25;
namespace s=source_shells;
#include "ObservedScene.h"
namespace {
s::Profile Profile() {
  s::Profile p;p.population=s::Population::OrdinaryShellsOnly;p.property_type=1;
  p.input_thickness_mode=0;p.level=1;p.gap_mode=1;p.free_edge_gap=0;p.contact_thickness_update=0;
  p.stiffness_scale=1;p.gap_scale=1;p.maximum_secondary_gap=1e30;p.maximum_main_gap=1e30;return p;
}
struct Fixture {
  std::vector<s::PhysicalShell> shells;
  std::vector<std::uint32_t> mains;
  std::vector<s::Secondary> secondary;
  std::vector<s::NodeFields> nodes;
  std::vector<double> main_values;
  std::vector<s::SecondaryFields> secondary_values;
  std::vector<std::max_align_t> scratch;
  s::Input in;
  void Refresh() {
    in.shells=shells.empty()?nullptr:shells.data();in.shell_count=shells.size();
    in.primary_shells=mains.empty()?nullptr:mains.data();in.primary_count=mains.size();
    in.secondary=secondary.empty()?nullptr:secondary.data();in.secondary_count=secondary.size();
  }
  s::Report Build() {
    Refresh();s::Forecast f;
    auto r=s::Preflight(in,{},f);if(r.status!=s::Status::Ok)return r;
    scratch.resize((f.scratch_bytes+sizeof(std::max_align_t)-1)/sizeof(std::max_align_t));
    nodes.resize(in.node_count);main_values.resize(mains.size());secondary_values.resize(secondary.size());
    return s::Build(in,{},scratch.data(),scratch.size()*sizeof(std::max_align_t),Output());
  }
  s::Output Output() {
    return {nodes.data(),nodes.size(),main_values.empty()?nullptr:main_values.data(),main_values.size(),
      secondary_values.empty()?nullptr:secondary_values.data(),secondary_values.size()};
  }
};
Fixture Mixed() {
  Fixture f;f.in.profile=Profile();f.in.node_count=10;
  f.shells={{1,ShellLayout::Quad4,{0,1,2,3},100,2,2,1,8},
            {2,ShellLayout::Quad4,{4,5,6,7},20,3,0,6,0},
            {3,ShellLayout::Triangle3,{0,8,9,9},300,1,1,2,0}};
  f.mains={2};for(unsigned i=0;i<10;++i)f.secondary.push_back({i,1});return f;
}
TEST(ShellContactSource, ActualStarterWholeModelAndEngineSecondaryGaps) {
  Fixture f;f.in.profile=Profile();f.in.node_count=observed::Nodes;
  f.shells.assign(std::begin(observed::Shells),std::end(observed::Shells));
  f.mains.assign(std::begin(observed::Primary),std::end(observed::Primary));
  f.secondary.assign(std::begin(observed::Secondary),std::end(observed::Secondary));
  ASSERT_EQ(f.Build().status,s::Status::Ok);
  for(std::size_t i=0;i<f.nodes.size();++i) {
    EXPECT_DOUBLE_EQ(f.nodes[i].young_thickness_sum,observed::ETNOD[i]);
    EXPECT_EQ(f.nodes[i].shell_incidence_count,observed::NSHNOD[i]);
    EXPECT_DOUBLE_EQ(f.nodes[i].stiffness,observed::STIFINT[i]);
  }
  for(std::size_t i=0;i<f.main_values.size();++i)EXPECT_DOUBLE_EQ(f.main_values[i],observed::PrimaryK[i]);
  for(std::size_t i=0;i<f.secondary.size();++i) {
    EXPECT_DOUBLE_EQ(f.secondary_values[i].stiffness,observed::SecondaryK[i]);
    EXPECT_DOUBLE_EQ(f.secondary_values[i].gap,observed::SecondaryGap[i]);
  }
}
TEST(ShellContactSource, NoncontactShellsContributePhysicalKAndSharedMainThickness) {
  auto f=Mixed();f.in.profile.maximum_main_gap=3;f.in.profile.maximum_secondary_gap=.75;
  ASSERT_EQ(f.Build().status,s::Status::Ok);
  EXPECT_DOUBLE_EQ(f.nodes[0].young_thickness_sum,500);EXPECT_EQ(f.nodes[0].shell_incidence_count,2);
  EXPECT_DOUBLE_EQ(f.nodes[0].stiffness,250);EXPECT_DOUBLE_EQ(f.nodes[0].unscaled_half_gap,4);
  EXPECT_DOUBLE_EQ(f.main_values[0],300); // Main E*selected thickness, not nodal average.
  EXPECT_DOUBLE_EQ(f.secondary_values[1].stiffness,200);EXPECT_DOUBLE_EQ(f.secondary_values[1].gap,0);
  EXPECT_DOUBLE_EQ(f.secondary_values[4].stiffness,60);EXPECT_DOUBLE_EQ(f.secondary_values[4].gap,0);
  EXPECT_DOUBLE_EQ(f.secondary_values[0].gap,.75);
  const std::uint32_t corners[]{0,8,9,9};s::MainGapFields main;
  ASSERT_EQ(s::MainGaps(f.nodes.data(),f.nodes.size(),corners,&main).status,s::Status::Ok);
  EXPECT_DOUBLE_EQ(main.corner[0],3);EXPECT_DOUBLE_EQ(main.corner[1],.5);
  EXPECT_DOUBLE_EQ(main.corner[2],main.corner[3]);EXPECT_DOUBLE_EQ(main.maximum,3);
}
TEST(ShellContactSource, InputThicknessModeIgnoresContactOverridesButNotPhysicalK) {
  auto f=Mixed();f.in.profile.input_thickness_mode=1;f.in.profile.gap_scale=2;
  ASSERT_EQ(f.Build().status,s::Status::Ok);
  EXPECT_DOUBLE_EQ(f.nodes[0].stiffness,250);EXPECT_DOUBLE_EQ(f.main_values[0],600);
  EXPECT_DOUBLE_EQ(f.nodes[0].main_gap,2);EXPECT_DOUBLE_EQ(f.secondary_values[8].gap,2);
}
TEST(ShellContactSource, ExactRemovalMaskAndEmptyContactSelection) {
  auto f=Mixed();f.secondary[0].existing_coefficient=-0.;f.in.profile.stiffness_scale=2;
  ASSERT_EQ(f.Build().status,s::Status::Ok);
  EXPECT_EQ(f.secondary_values[0].stiffness,0);EXPECT_TRUE(std::signbit(f.secondary_values[0].stiffness));
  EXPECT_EQ(f.nodes[0].stiffness,250);EXPECT_EQ(f.secondary_values[8].stiffness,600);
  f.mains.clear();f.secondary.clear();ASSERT_EQ(f.Build().status,s::Status::Ok);
  EXPECT_EQ(f.nodes[0].stiffness,250);EXPECT_FALSE(f.nodes[0].on_main_surface);
}
TEST(ShellContactSource, DuplicateMainAndLateArithmeticFailurePreserveAllOutputs) {
  auto f=Mixed();ASSERT_EQ(f.Build().status,s::Status::Ok);
  f.nodes[0].stiffness=123;f.main_values[0]=456;f.secondary_values[0].gap=789;
  f.shells.back().young=std::numeric_limits<double>::max();f.shells.back().structural_thickness=2;
  const auto bad=s::Build(f.in,{},f.scratch.data(),f.scratch.size()*sizeof(std::max_align_t),f.Output());
  EXPECT_EQ(bad.status,s::Status::NonfiniteResult);EXPECT_EQ(bad.physical_shell,2);
  EXPECT_EQ(f.nodes[0].stiffness,123);EXPECT_EQ(f.main_values[0],456);EXPECT_EQ(f.secondary_values[0].gap,789);
  f=Mixed();f.mains={2,2};const auto duplicate=f.Build();EXPECT_EQ(duplicate.status,s::Status::InvalidInput);EXPECT_EQ(duplicate.primary,1);
}
TEST(ShellContactSource, SourceOrderConnectivityProfileAndExtentRejections) {
  auto f=Mixed();f.Refresh();s::Forecast unchanged{123,456};
  std::swap(f.shells[0],f.shells[1]);EXPECT_EQ(s::Preflight(f.in,{},unchanged).status,s::Status::InvalidInput);
  EXPECT_EQ(unchanged.scratch_bytes,123);EXPECT_EQ(unchanged.output_bytes,456);
  f=Mixed();f.shells[2].source_element_id=1;EXPECT_EQ(f.Build().status,s::Status::InvalidInput);
  f=Mixed();f.shells[2].nodes[3]=8;EXPECT_EQ(f.Build().status,s::Status::InvalidInput);
  f=Mixed();f.in.profile.population=s::Population::Unspecified;EXPECT_EQ(f.Build().status,s::Status::UnsupportedProfile);
  f=Mixed();f.in.profile.contact_thickness_update=1;EXPECT_EQ(f.Build().status,s::Status::UnsupportedProfile);
  f=Mixed();f.Refresh();auto limits=s::Limits{};limits.nodes=9;EXPECT_EQ(s::Preflight(f.in,limits,unchanged).status,s::Status::ResourceLimit);
}
TEST(ShellContactSource, AliasedAndShortScratchRejectBeforePublication) {
  auto f=Mixed();ASSERT_EQ(f.Build().status,s::Status::Ok);s::Forecast forecast;
  ASSERT_EQ(s::Preflight(f.in,{},forecast).status,s::Status::Ok);
  f.nodes[0].stiffness=123;auto output=f.Output();
  EXPECT_EQ(s::Build(f.in,{},f.scratch.data(),forecast.scratch_bytes-1,output).status,s::Status::InvalidInput);
  output.primary_stiffness=&f.nodes[0].stiffness;
  EXPECT_EQ(s::Build(f.in,{},f.scratch.data(),f.scratch.size()*sizeof(std::max_align_t),output).status,s::Status::InvalidInput);
  EXPECT_EQ(f.nodes[0].stiffness,123);
}
TEST(ShellContactSource, MainGapFailurePreservesOutput) {
  auto f=Mixed();ASSERT_EQ(f.Build().status,s::Status::Ok);
  const std::uint32_t bad[]{0,8,9,1};s::MainGapFields output{{1,2,3,4},5};
  EXPECT_EQ(s::MainGaps(f.nodes.data(),f.nodes.size(),bad,&output).status,s::Status::InvalidInput);
  EXPECT_EQ(output.corner[3],4);EXPECT_EQ(output.maximum,5);
}
} // namespace
