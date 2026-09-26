// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace nodal_correction_test {
TEST(NodalCorrection, FirstNativeStorageWinnerAndRepeatedSlotsMatchFullSource) {
  for(bool reverse:{false,true}) {
    Fixture f;f.solids={Solid(2,6),Solid(3,21)};f.solids[1].nodes[7]=7;
    if(reverse)std::reverse(f.solids.begin(),f.solids.end());
    ASSERT_EQ(f.Prepare().status,c::Status::Ok);ASSERT_EQ(f.Run().status,c::Status::Ok);
    Same(f.output,Oracle(f.Input()));EXPECT_EQ(f.output[0],f.original[0]*(reverse?7:3));
    EXPECT_EQ(f.output[6],f.original[6]*(reverse?7:3));EXPECT_EQ(f.output[8],f.original[8]);
  }
}
TEST(NodalCorrection, Type24TailPreservesDuplicateOccurrencesAndSkipsVirtualAndTaggedNodes) {
  Fixture f;f.solids={Solid(2,6),Solid(3,21)};f.secondaries={8,0,9,8,UINT32_MAX};
  ASSERT_EQ(f.Prepare().status,c::Status::Ok);ASSERT_EQ(f.Run().status,c::Status::Ok);
  Same(f.output,Oracle(f.Input()));EXPECT_EQ(f.output[8],23*7*7);EXPECT_EQ(f.output[9],29*7);EXPECT_EQ(f.output[0],2*3);
}
TEST(NodalCorrection, EmptyAndDisabledSolidRowsDoNotConsumeUnusedOperands) {
  for(bool disabled:{false,true}) {
    Fixture f;f.original[0]=-0.;f.secondaries={0,1,UINT32_MAX};
    if(disabled){auto s=Solid(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),2);s.nodes[0]=UINT32_MAX;f.solids.push_back(s);}
    ASSERT_EQ(f.Prepare().status,c::Status::Ok);ASSERT_EQ(f.Run().status,c::Status::Ok);
    Same(f.output,Oracle(f.Input()));Same(f.output,f.original);
  }
}
TEST(NodalCorrection, NativePressureFloorNeighborsControlZeroAndSignedZero) {
  const double floor=1./1e20;
  for(double bulk:{0.,-0.,std::nextafter(floor,0.),floor,std::nextafter(floor,1.),7.})
    for(double controlled:{0.,-0.,2e-20,14.}) {
      Fixture f;f.solids={Solid(bulk,controlled)};f.original[0]=-0.;
      ASSERT_EQ(f.Prepare().status,c::Status::Ok);ASSERT_EQ(f.Run().status,c::Status::Ok);Same(f.output,Oracle(f.Input()));
    }
}
TEST(NodalCorrection, WholeAsstifiValueIsScaledOnceRatherThanPerSolidSeed) {
  Fixture f;f.original[0]=11+5*std::pow(8.,1./3.)+7;f.solids={Solid(2,6)};
  ASSERT_EQ(f.Prepare().status,c::Status::Ok);ASSERT_EQ(f.Run().status,c::Status::Ok);
  Same(f.output,Oracle(f.Input()));EXPECT_EQ(f.output[0],3*f.original[0]);
  EXPECT_NE(f.output[0],11+15*std::pow(8.,1./3.)+7);
}
TEST(NodalCorrection, LateOverflowDoesNotPublishThenRetry) {
  Fixture f;f.solids={Solid(2,6)};f.secondaries={8,9};ASSERT_EQ(f.Prepare().status,c::Status::Ok);
  const auto prior=f.output;f.original[9]=std::numeric_limits<double>::max();
  auto r=f.Run();EXPECT_EQ(r.status,c::Status::NonfiniteResult);EXPECT_EQ(r.type24_occurrence,1u);Same(f.output,prior);
  f.secondaries.clear();f.solids.push_back(Solid(1e-20,std::numeric_limits<double>::max()));
  r=f.Run();EXPECT_EQ(r.status,c::Status::NonfiniteResult);EXPECT_EQ(r.solid,1u);Same(f.output,prior);
  f.solids.resize(1);f.original[9]=29;ASSERT_EQ(f.Run().status,c::Status::Ok);Same(f.output,Oracle(f.Input()));
}
TEST(NodalCorrection, ResourceBoundaryAndInvalidAliasedBuffersPreserveValues) {
  Fixture f;f.solids={Solid(2,6)};ASSERT_EQ(f.Prepare().status,c::Status::Ok);const auto prior=f.output;
  auto limits=c::Limits{};limits.scratch_bytes=f.forecast.scratch_bytes;c::Forecast forecast;
  EXPECT_EQ(c::Preflight(f.Input(),limits,forecast).status,c::Status::Ok);--limits.scratch_bytes;
  EXPECT_EQ(c::Preflight(f.Input(),limits,forecast).status,c::Status::ResourceLimit);
  auto in=f.Input();in.solid_count=c::Limits{}.solids+1;in.solids=reinterpret_cast<const c::Solid*>(1);
  EXPECT_EQ(c::Preflight(in,{},forecast).status,c::Status::ResourceLimit);
  f.solids.back().nodes[7]=f.original.size();EXPECT_EQ(f.Run().status,c::Status::InvalidInput);Same(f.output,prior);
  f.solids.back().nodes[7]=6;
  EXPECT_EQ(c::Apply(f.Input(),{},f.scratch.data(),f.scratch.bytes()-1,f.Output()).status,c::Status::InvalidInput);
  EXPECT_EQ(c::Apply(f.Input(),{},f.scratch.data(),f.scratch.bytes(),{f.original.data(),f.original.size()}).status,c::Status::InvalidInput);
  auto* alias=static_cast<double*>(f.scratch.data());
  EXPECT_EQ(c::Apply(f.Input(),{},f.scratch.data(),f.scratch.bytes(),{alias,f.original.size()}).status,c::Status::InvalidInput);Same(f.output,prior);
}
}
