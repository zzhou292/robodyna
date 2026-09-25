// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include <cmath>
#include <limits>
namespace type25_search_startup_test {
TEST(Type25SearchStartup, NativeScalarMultiplierRetainsReal4DefaultAndThresholdAssociation) {
  for(int nodes:{1,1500000,1500001,2500000,2500001,2147483647}) {
    double value=-1;ASSERT_EQ(s::ResolveMultiplier(nodes,&value),s::Status::Ok);
    EXPECT_EQ(Bits(value),Bits(OracleMultiplier(nodes)));
  }
  double value=-13;EXPECT_EQ(s::ResolveMultiplier(0,&value),s::Status::InvalidInput);EXPECT_EQ(value,-13);
  EXPECT_EQ(s::ResolveMultiplier(UINT64_MAX,&value),s::Status::InvalidInput);EXPECT_EQ(value,-13);
}
TEST(Type25SearchStartup, FullSerialRemovalAndInverseOrderMatchNativeOnQuadTriangleAndMixedMeshes) {
  for(unsigned mode=0;mode<3;++mode)for(double gap:{.01,.6,3.}) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(gap);
    Fixture f(old::Grid(3,2,mode));f.Gaps(gap);
    const auto expected=Oracle(f.Input());const Built actual(f);Same(actual.view,expected);
    if(gap==3.)EXPECT_GT(actual.view.removal_count,0u);
  }
}
TEST(Type25SearchStartup, SerialMixedExpansionPreservesOriginalWorkerLocalScratchOrder) {
  Fixture f(old::Grid(3,2,2));for(auto& row:f.secondary)row.gap=.01;
  for(std::size_t p=0;p<f.mesh.primary.size();++p) {
    const double gap=p%2?3.:0.;f.main_gaps[p]=gap;f.main_gaps[p+f.mesh.primary.size()]=gap;
  }
  const auto expected=Oracle(f.Input());Built actual(f);Same(actual.view,expected);
  auto in=f.Input();in.profile.initialization=s::Initialization::InvariantNoExpansion;
  const auto bytes=Bytes(actual);
  EXPECT_EQ(s::Build(in,{},actual.output,actual.scratch,&actual.view).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(std::memcmp(bytes.data(),actual.output.data(),bytes.size()),0);
}
TEST(Type25SearchStartup, ProvenNoExpansionIsIndependentOfPreprocessingWorkerAssignment) {
  Fixture f(old::Grid(3,2,1));f.Gaps(.01);const auto expected=Oracle(f.Input());
  f.profile.initialization=s::Initialization::InvariantNoExpansion;
  const Built actual(f);Same(actual.view,expected);EXPECT_EQ(actual.view.removal_count,0u);
}
TEST(Type25SearchStartup, InclusiveDistanceGapBoundaryUsesActualNativeMinimumEdge) {
  Fixture f(old::Grid(2,2));for(auto& row:f.secondary)row.gap=0;
  const double threshold=1./std::sqrt(2.);
  bool accepted=false,rejected=false;
  for(double gap:{std::nextafter(threshold,0.),threshold,std::nextafter(threshold,2.)}) {
    std::fill(f.main_gaps.begin(),f.main_gaps.end(),gap);
    const auto expected=Oracle(f.Input());Built actual(f);Same(actual.view,expected);
    auto in=f.Input();in.profile.initialization=s::Initialization::InvariantNoExpansion;
    const auto status=s::Build(in,{},actual.output,actual.scratch,&actual.view).status;
    const bool expands=expected.scalar[4]<=std::sqrt(2.)*gap;
    EXPECT_EQ(status,expands?s::Status::UnsupportedProfile:s::Status::Ok);
    accepted|=!expands;rejected|=expands;
  }
  EXPECT_TRUE(accepted);EXPECT_TRUE(rejected);
}
TEST(Type25SearchStartup, OneAndTwoPrimaryFacesUseExpandedCountForSmallMarginReduction) {
  for(unsigned primaries:{1u,2u}) {
    auto mesh=old::Grid(primaries,1);
    const auto original_nodes=mesh.ids.size();
    for(unsigned i=0;i<1100;++i){mesh.ids.push_back(mesh.ids.size()+1);mesh.positions.insert(mesh.positions.end(),{double(primaries)+.02,.5,0.});}
    Fixture f(std::move(mesh));f.secondary.erase(f.secondary.begin(),f.secondary.begin()+original_nodes);f.Gaps(0.);
    const Built actual(f);Same(actual.view,Oracle(f.Input()));
    EXPECT_EQ(actual.view.main_count,2*primaries);
    if(primaries==1)EXPECT_LT(actual.view.mean_length,1.);
    else EXPECT_EQ(actual.view.mean_length,1.);
  }
}
TEST(Type25SearchStartup, ExtentBranchIsNativeBoundingLengthAndCoordinatesMayBeSi) {
  auto mesh=old::Grid(2,1,2);
  for(std::size_t i=0;i<mesh.ids.size();++i)mesh.positions[3*i+2]=.25*mesh.positions[3*i]*mesh.positions[3*i+1];
  for(int flag:{0,1}) {
    Fixture f(mesh);f.profile.curvature=flag;const Built native(f);Same(native.view,Oracle(f.Input()));
    if(flag)EXPECT_GT(native.view.maximum_extent,0.);else EXPECT_EQ(native.view.maximum_extent,0.);
    auto si_mesh=mesh;si_mesh.units=st::Coordinates::Si;for(auto& v:si_mesh.positions)v*=si_mesh.scale.length_m;
    Fixture si(std::move(si_mesh));si.profile.curvature=flag;const Built converted(si);Same(converted.view,Oracle(si.Input()));
  }
}
TEST(Type25SearchStartup, ExactRemovalCapacityFailurePreservesEveryOutputByteAndAllowsRetry) {
  Fixture f;f.Gaps(3.);Built actual(f);ASSERT_GT(actual.view.removal_count,0u);
  const auto before=Bytes(actual);const auto old=actual.view;
  auto limits=s::Limits{};limits.max_removals=old.removal_count-1;
  const auto report=s::Build(f.Input(),limits,actual.output,actual.scratch,&actual.view);
  EXPECT_EQ(report.status,s::Status::ResourceLimit);EXPECT_TRUE(report.removal_count_complete);
  EXPECT_EQ(report.required_removals,old.removal_count);EXPECT_EQ(std::memcmp(before.data(),actual.output.data(),before.size()),0);
  EXPECT_EQ(actual.view.removed_mains,old.removed_mains);
  EXPECT_EQ(s::Build(f.Input(),{},actual.output,actual.scratch,&actual.view).status,s::Status::Ok);
  Same(actual.view,Oracle(f.Input()));
}
TEST(Type25SearchStartup, UnknownOrPresentContributorsAndDifferentFlagPolicyRejectAtomically) {
  Fixture f;Built actual(f);const auto before=Bytes(actual);
  for(unsigned variant=0;variant<8;++variant) {
    auto in=f.Input();
    if(variant==0)in.contributors.census=s::Census::Unspecified;
    if(variant==1)in.contributors.tied_interfaces=1;
    if(variant==2)in.contributors.rigid_bodies=1;
    if(variant==3)in.contributors.cin_links=1;
    if(variant==4)in.contributors.other_interfaces=1;
    if(variant==5)in.profile.initial_penetration=0;
    if(variant==6)in.profile.gap_load_cards=s::LoadCards::Unspecified;
    if(variant==7)in.profile.initialization=s::Initialization::Unspecified;
    EXPECT_EQ(s::Build(in,{},actual.output,actual.scratch,&actual.view).status,s::Status::UnsupportedProfile);
    EXPECT_EQ(std::memcmp(before.data(),actual.output.data(),before.size()),0);
  }
  for(std::size_t i=0;i<actual.view.secondary_count;++i)EXPECT_EQ(actual.view.initial_contact[i],0);
}
TEST(Type25SearchStartup, ArenaAliasDuplicateSecondaryAndNumericalOverflowNeverPublish) {
  Fixture f;Built actual(f);const auto before=Bytes(actual);
  EXPECT_EQ(s::Build(f.Input(),{},actual.scratch,actual.scratch,&actual.view).status,s::Status::InvalidInput);
  const auto saved=f.secondary.back();f.secondary.back()=f.secondary.front();
  EXPECT_EQ(s::Build(f.Input(),{},actual.output,actual.scratch,&actual.view).status,s::Status::InvalidInput);
  f.secondary.back()=saved;f.main_gaps.assign(f.main_gaps.size(),std::numeric_limits<double>::max());
  for(auto& row:f.secondary)row.gap=std::numeric_limits<double>::max();
  EXPECT_EQ(s::Build(f.Input(),{},actual.output,actual.scratch,&actual.view).status,s::Status::NonfiniteResult);
  EXPECT_EQ(std::memcmp(before.data(),actual.output.data(),before.size()),0);
}
TEST(Type25SearchStartup, NativeNonterminatingSmallMarginIsRejectedWithoutPublishing) {
  auto mesh=old::Grid(1,1);
  for(auto& value:mesh.positions)value=0.;
  for(unsigned i=0;i<1100;++i){mesh.ids.push_back(mesh.ids.size()+1);mesh.positions.insert(mesh.positions.end(),{0.,0.,0.});}
  Fixture f(std::move(mesh));f.Gaps(0.);for(auto& row:f.secondary)row.stiffness=0.;
  const auto in=f.Input();const auto plan=s::Preflight(in.mesh.node_count,in.mesh.primary_count,in.secondary_count);
  tl::util::HostArena output,scratch;ASSERT_TRUE(output.Initialize(plan.output_bytes));ASSERT_TRUE(scratch.Initialize(plan.scratch_bytes));
  std::memset(output.data(),0xa5,output.bytes());s::Snapshot view;view.margin=-37;
  const auto report=s::Build(in,{},output,scratch,&view);
  EXPECT_EQ(report.status,s::Status::NoProgress);EXPECT_EQ(view.margin,-37);
  const auto* bytes=static_cast<const unsigned char*>(output.data());
  for(std::size_t i=0;i<output.bytes();++i)ASSERT_EQ(bytes[i],0xa5u);
  // The original loop would continue forever: DD0=0, zero first-pass activity,
  // over1000 coincident unmasked second-pass rows, and 0 < 0 is false. Do not
  // execute that native oracle. Restore activity: its source loop never enters.
  for(auto& row:f.secondary)row.stiffness=1.;
  EXPECT_EQ(s::Build(f.Input(),{},output,scratch,&view).status,s::Status::Ok);
  Same(view,Oracle(f.Input()));
}
TEST(Type25SearchStartup, MovingStarterProfilePreservesNativeInitialMarginRemovalAndFlags) {
  for(unsigned mode=0;mode<3;++mode)for(double gap:{.01,3.}) {
    SCOPED_TRACE(mode);SCOPED_TRACE(gap);
    Fixture f(old::Grid(3,2,mode));f.Gaps(gap);Built actual(f);
    const auto expected=Oracle(f.Input());auto input=f.Input();
    input.mesh.profile=st::Profile::OrdinaryExteriorMovingMain;
    ASSERT_EQ(s::Build(input,{},actual.output,actual.scratch,&actual.view).status,s::Status::Ok);
    Same(actual.view,expected);
    const auto before=Bytes(actual);input.mesh.profile=st::Profile::Unspecified;
    EXPECT_EQ(s::Build(input,{},actual.output,actual.scratch,&actual.view).status,s::Status::UnsupportedProfile);
    EXPECT_EQ(std::memcmp(before.data(),actual.output.data(),before.size()),0);
  }
}
} // namespace type25_search_startup_test
