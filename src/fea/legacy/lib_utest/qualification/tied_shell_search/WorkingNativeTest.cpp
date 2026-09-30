// SPDX-License-Identifier: MIT
#include "WorkingFixture.h"

namespace tied_search_test {
TEST(TiedSearchWorkingNative, ExactBoxDecisionDetectsLossySourceToSiRoundTrip) {
  auto original=WorkingBounds(OriginalWorkingShape());
  original.master_thickness=1e-20;
  const ts::Vec3 point{1000.1234,0,0};
  for(auto& vertex:original.master_position) vertex=point;
  // A native regularized zero-area master is deliberately used here: its
  // positive radius rounds away at this coordinate, making bit loss decisive.
  // This gate never claims that this singular patch can carry structural force.
  ts::NativeSearchBounds direct,roundtrip;
  ASSERT_EQ(ts::PrepareSearchBounds(original,direct),ts::Status::Success);
  const auto native=NativeBounds(original,point);
  EXPECT_EQ(direct.minimum.x,native[0]);
  EXPECT_EQ(direct.maximum.x,native[3]);
  EXPECT_EQ(direct.minimum.x,point.x);
  EXPECT_EQ(direct.maximum.x,point.x);
  ts::SearchBoundsInput si;
  si.topology=original.topology;
  si.working_length_to_m=original.working_length_to_m;
  si.master_thickness_m=original.master_thickness*si.working_length_to_m;
  for(auto& vertex:si.master_position_m)
    vertex={point.x*si.working_length_to_m,0,0};
  ASSERT_EQ(ts::PrepareSearchBounds(si,roundtrip),ts::Status::Success);
  EXPECT_NE(roundtrip.minimum.x,direct.minimum.x);
  bool within=false;
  ASSERT_EQ(ts::WithinWorkingSearchBounds(direct,point,within),ts::Status::Success);
  EXPECT_TRUE(within);
  EXPECT_EQ(native[7],1.);
  ASSERT_EQ(ts::WithinWorkingSearchBounds(roundtrip,point,within),ts::Status::Success);
  EXPECT_FALSE(within);
}
TEST(TiedSearchWorkingNative, PositiveSiThicknessUnderflowRetainsNativeDiagonalAdmission) {
  auto input=Shape();
  input.working_length_to_m=2.;
  input.master_thickness_m=std::numeric_limits<double>::denorm_min();
  ASSERT_GT(input.master_thickness_m,0.);
  ASSERT_EQ(input.master_thickness_m/input.working_length_to_m,0.);
  ts::CandidateProjection projection;NativeChoice native;
  ASSERT_EQ(ts::ProjectCandidate(input,projection),ts::Status::Success);
  Compare(projection,Native(input,1,native));
  ts::NativeSearchBounds bounds;
  ASSERT_EQ(ts::PrepareSearchBounds(BoundsInput(input),bounds),ts::Status::Success);
  const auto expected=NativeBounds(BoundsInput(input),input.geometry_m.secondary_position);
  const auto values=BoundValues(bounds);
  for(unsigned i=0;i<values.size();++i) EXPECT_DOUBLE_EQ(values[i],expected[i]);
  auto working=OriginalWorkingShape();working.master_thickness=0;
  EXPECT_EQ(ts::ProjectCandidate(working,projection),ts::Status::InvalidInput);
  EXPECT_EQ(ts::PrepareSearchBounds(WorkingBounds(working),bounds),ts::Status::InvalidInput);
}
TEST(TiedSearchWorkingNative, OriginalDoublesReachNativeProjectionBoundsAndChoiceWithoutSiRoundTrip) {
  volatile double original=1000.1234,unit=.001;
  const double si=original*unit;
  ASSERT_NE(original,si/unit); // This source value cannot round trip through SI.
  for(unsigned shape=0;shape<3;++shape) for(double z:{-.7,0.,.1,.7,2.}) {
    auto input=OriginalWorkingShape(shape);
    input.geometry.secondary_position.z=z;
    ts::CandidateProjection projection;ts::NativeSearchBounds bounds;bool within=false;
    ASSERT_EQ(ts::ProjectCandidate(input,projection),ts::Status::Success);
    ASSERT_EQ(ts::PrepareSearchBounds(WorkingBounds(input),bounds),ts::Status::Success);
    ASSERT_EQ(ts::WithinWorkingSearchBounds(bounds,input.geometry.secondary_position,within),ts::Status::Success);
    CompareWorking(input,projection,bounds,within);
    ts::SearchChoice actual;NativeChoice native;
    for(int rank:{8,3}) {
      ASSERT_EQ(ts::ConsiderCandidate(input,rank,actual),ts::Status::Success);
      Native(input,rank,native);
      ASSERT_EQ(actual.matched,native.selected!=0);
      if(actual.matched) EXPECT_EQ(actual.ordered_master,native.selected);
    }
    ASSERT_FALSE(HasFailure());
  }
}
TEST(TiedSearchWorkingNative, ExactNativeBoxBoundaryAndRejectedInputsPreservePublishedValues) {
  auto input=OriginalWorkingShape();
  ts::NativeSearchBounds bounds;
  ASSERT_EQ(ts::PrepareSearchBounds(WorkingBounds(input),bounds),ts::Status::Success);
  auto point=input.geometry.secondary_position;point.x=bounds.maximum.x;
  bool within=false;
  ASSERT_EQ(ts::WithinWorkingSearchBounds(bounds,point,within),ts::Status::Success);
  EXPECT_TRUE(within);
  point.x=std::nextafter(point.x,std::numeric_limits<double>::infinity());
  ASSERT_EQ(ts::WithinWorkingSearchBounds(bounds,point,within),ts::Status::Success);
  EXPECT_FALSE(within);
  ts::CandidateProjection projection;
  ASSERT_EQ(ts::ProjectCandidate(input,projection),ts::Status::Success);
  const auto retained=projection;
  const auto retained_bounds=BoundValues(bounds);
  for(unsigned fault=0;fault<3;++fault) {
    auto bad=input;
    if(fault==0) bad.geometry.master_position[3].z=std::numeric_limits<double>::quiet_NaN();
    if(fault==1) bad.working_length_to_m=0;
    if(fault==2) bad.geometry.master_position[3].z=std::numeric_limits<double>::max();
    EXPECT_NE(ts::ProjectCandidate(bad,projection),ts::Status::Success);
    Compare(projection,retained);
    EXPECT_NE(ts::PrepareSearchBounds(WorkingBounds(bad),bounds),ts::Status::Success);
    EXPECT_EQ(BoundValues(bounds),retained_bounds);
  }
  point.z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ts::WithinWorkingSearchBounds(bounds,point,within),ts::Status::InvalidInput);
  EXPECT_FALSE(within);
  ASSERT_EQ(ts::ProjectCandidate(input,projection),ts::Status::Success);
  Compare(projection,retained);
}
} // namespace tied_search_test
