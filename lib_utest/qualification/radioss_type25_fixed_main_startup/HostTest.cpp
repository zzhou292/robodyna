// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "lib_src/collision/radioss_type25/startup/FloatNormals.h"
#include <algorithm>
#include <cfenv>
#include <limits>
namespace type25_startup_test {
TEST(Type25FixedStartup, CompleteNativeQuadTriangleAndMixedTopology) {
  for(unsigned mode=0;mode<3;++mode)for(unsigned size=1;size<=3;++size) {
    SCOPED_TRACE(mode);SCOPED_TRACE(size);auto source=Grid(size,2,mode);
    const Built built(source);Same(built,Oracle(source.Input(),source.coefficients.data(),source.coefficients.size()));
  }
}
TEST(Type25FixedStartup, RotatedWarpedScaledAndChangedPrimaryOrderMatchNative) {
  for(unsigned change=0;change<5;++change) {
    SCOPED_TRACE(change);auto source=Grid(2,2,2);
    for(std::size_t i=0;i<source.ids.size();++i) {
      const double x=source.positions[3*i],y=source.positions[3*i+1];
      if(change==0){source.positions[3*i]=.8*x-.6*y;source.positions[3*i+1]=.6*x+.8*y;}
      if(change==1)source.positions[3*i+2]=.125*x*y;
      if(change==2){source.positions[3*i]=y;source.positions[3*i+1]=0;source.positions[3*i+2]=-x;}
      if(change==3)for(unsigned k=0;k<3;++k)source.positions[3*i+k]*=.00001;
    }
    if(change==4)std::reverse(source.primary.begin(),source.primary.end());
    const Built built(source);Same(built,Oracle(source.Input(),source.coefficients.data(),source.coefficients.size()));
  }
}
TEST(Type25FixedStartup, NativeStageFloorsTriangleUnusedSlotAndSignedZeros) {
  for(double scale:{1.,1e-10,1e-20}) {
    auto source=Grid(1,1,1);for(auto& value:source.positions)value*=scale;
    for(std::size_t i=0;i<source.ids.size();++i)source.positions[3*i+2]=-0.;
    const auto expected=Oracle(source.Input(),source.coefficients.data(),source.coefficients.size());
    const Built built(source);Same(built,expected);
    EXPECT_EQ(Bits(s::detail::fp::StarterFloor()),Bits(expected.floors[1]));
    EXPECT_EQ(Bits(s::detail::fp::ReadyFloor()),Bits(expected.floors[3]));
    for(std::size_t m=0;m<built.startup.main_count;++m) {
      EXPECT_EQ(Bits(built.startup.starter.face_normals[4*m+2].x),0u);
      EXPECT_EQ(Bits(built.ready.normals.face_normals[4*m+2].z),0u);
    }
  }
}
TEST(Type25FixedStartup, NativeAndSiInputsRetainPhysicalParentMapping) {
  auto source=Grid(2,1,2);source.units=s::Coordinates::Si;
  for(auto& value:source.positions)value*=source.scale.length_m;
  const Built built(source);Same(built,Oracle(source.Input(),source.coefficients.data(),source.coefficients.size()));
  for(std::size_t m=0;m<built.startup.main_count;++m)
    EXPECT_EQ(built.startup.mains[m].source_id,source.primary[built.startup.expanded_to_primary[m]].source_id);
}
TEST(Type25FixedStartup, ExactCapsAndLateRejectionPreservePublishedBytes) {
  auto source=Grid(2,2,1);Built built(source);
  const std::vector<unsigned char> before(static_cast<const unsigned char*>(built.output.data()),
      static_cast<const unsigned char*>(built.output.data())+built.output.bytes());
  const auto old=built.startup;
  auto input=source.Input();auto limits=s::Limits{};limits.max_output_bytes=built.forecast.output_bytes-1;
  EXPECT_EQ(s::BuildStarter(input,limits,built.output,built.scratch,&built.startup).status,s::Status::ResourceLimit);
  source.primary.back()=source.primary.front();source.primary.back().source_id=999;
  input=source.Input();EXPECT_EQ(s::BuildStarter(input,{},built.output,built.scratch,&built.startup).status,s::Status::UnsupportedTopology);
  EXPECT_EQ(std::memcmp(before.data(),built.output.data(),before.size()),0);
  EXPECT_EQ(built.startup.mains,old.mains);EXPECT_EQ(built.startup.source_generation,old.source_generation);
}
TEST(Type25FixedStartup, NonmanifoldEdgesDisconnectedFansAndUnknownProfilesReject) {
  auto source=Grid(1,1,1);
  source.ids.insert(source.ids.end(),{5,6});source.positions.insert(source.positions.end(),{0.,-1.,0.,0.,0.,1.});
  source.Add(n::ShellLayout::Triangle3,1,0,4,4);
  source.Add(n::ShellLayout::Triangle3,0,1,5,5); // Third distinct primary sharing edge0-1.
  const auto plan=s::Preflight(source.ids.size(),source.primary.size());
  tl::util::HostArena out,scratch;ASSERT_TRUE(out.Initialize(plan.output_bytes));ASSERT_TRUE(scratch.Initialize(plan.scratch_bytes));
  s::Snapshot view;auto input=source.Input();
  EXPECT_EQ(s::BuildStarter(input,{},out,scratch,&view).status,s::Status::UnsupportedTopology);EXPECT_EQ(view.mains,nullptr);
  input.profile=s::Profile::Unspecified;EXPECT_EQ(s::BuildStarter(input,{},out,scratch,&view).status,s::Status::UnsupportedProfile);
  Case fan;fan.ids={1,2,3,4,5};fan.positions={0,0,0,1,0,0,0,1,0,-1,0,0,0,-1,0};
  fan.Add(n::ShellLayout::Triangle3,0,1,2,2);fan.Add(n::ShellLayout::Triangle3,0,3,4,4);
  input=fan.Input();EXPECT_EQ(s::BuildStarter(input,{},out,scratch,&view).status,s::Status::UnsupportedTopology);
  EXPECT_EQ(view.mains,nullptr);
}
TEST(Type25FixedStartup, ReadyRequiresActualPositiveActivityAndPreservesPriorStage) {
  auto source=Grid(1,1);Built built(source);
  const std::vector<unsigned char> before(static_cast<const unsigned char*>(built.ready_output.data()),
      static_cast<const unsigned char*>(built.ready_output.data())+built.ready_output.bytes());
  source.coefficients.back()=0;const auto input=source.Input();
  EXPECT_EQ(s::BuildFixedMain(input,built.startup,{source.coefficients.data(),source.coefficients.size()},
      {},built.ready_output,built.ready_scratch,&built.ready).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(std::memcmp(before.data(),built.ready_output.data(),before.size()),0);
  EXPECT_EQ(built.startup.starter.references[0].boundary,1);
  EXPECT_EQ(built.ready.normals.references[0].boundary,2);
}
TEST(Type25FixedStartup, UnsupportedEnvironmentAndArenaAliasesDoNotPublish) {
  auto source=Grid(1,1);Built built(source);const auto before=built.startup;
  auto input=source.Input();
  EXPECT_EQ(s::BuildStarter(input,{},built.scratch,built.scratch,&built.startup).status,s::Status::InvalidInput);
  const int old_round=std::fegetround();ASSERT_EQ(std::fesetround(FE_UPWARD),0);
  const auto status=s::BuildStarter(input,{},built.output,built.scratch,&built.startup).status;
  const int restored=std::fesetround(old_round);
  EXPECT_EQ(restored,0);EXPECT_EQ(status,s::Status::UnsupportedArithmetic);EXPECT_EQ(built.startup.mains,before.mains);
}
TEST(Type25FixedStartup, CorruptedBorrowedSnapshotRejectsBeforeReadyPublication) {
  auto source=Grid(2,1);Built built(source);const auto input=source.Input();
  const auto before=built.ready;
  const std::vector<unsigned char> bytes(static_cast<const unsigned char*>(built.ready_output.data()),
      static_cast<const unsigned char*>(built.ready_output.data())+built.ready_output.bytes());
  std::vector<s::Main> mains(built.startup.mains,built.startup.mains+built.startup.main_count);
  auto view=built.startup;view.mains=mains.data();
  std::size_t linked=SIZE_MAX;unsigned edge=0;
  for(std::size_t m=0;m<mains.size() && linked==SIZE_MAX;++m)
    for(unsigned k=0;k<4;++k)if(mains[m].neighbors[k]){linked=m;edge=k;break;}
  ASSERT_NE(linked,SIZE_MAX);
  const auto original=mains[linked];
  const auto other=std::size_t(original.neighbors[edge]-1);
  const auto opposite=unsigned(original.neighbor_edges[edge]-1);
  const auto neighbor=mains[other];
  mains[other].neighbors[opposite]=0;mains[other].neighbor_edges[opposite]=0;
  EXPECT_EQ(s::BuildFixedMain(input,view,{source.coefficients.data(),source.coefficients.size()},
      {},built.ready_output,built.ready_scratch,&built.ready).status,s::Status::InvalidInput);
  mains[other]=neighbor;
  mains[linked].normal_reference[edge]=original.normal_reference[(edge+1)%4];
  EXPECT_EQ(s::BuildFixedMain(input,view,{source.coefficients.data(),source.coefficients.size()},
      {},built.ready_output,built.ready_scratch,&built.ready).status,s::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(bytes.data(),built.ready_output.data(),bytes.size()),0);
  EXPECT_EQ(built.ready.normals.face_normals,before.normals.face_normals);
  mains[linked]=original;
  EXPECT_EQ(s::BuildFixedMain(input,view,{source.coefficients.data(),source.coefficients.size()},
      {},built.ready_output,built.ready_scratch,&built.ready).status,s::Status::Ok);
}
TEST(Type25FixedStartup, ReadyCohortCrossesOriginalEngine129Boundary) {
  const auto source=Grid(13,11);ASSERT_EQ(source.primary.size(),143u);
  ASSERT_LE(source.ids.size(),256u);
  const Built built(source);
  Same(built,Oracle(source.Input(),source.coefficients.data(),source.coefficients.size()));
}
} // namespace type25_startup_test
