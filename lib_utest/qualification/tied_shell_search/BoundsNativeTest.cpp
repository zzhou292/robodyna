// SPDX-License-Identifier: MIT
#include "BoundsFixture.h"
namespace tied_search_test {
TEST(TiedSearchBoundsNative, OriginalBoxUsesMaximumDiagonalAndInterfaceThickness) {
  for(unsigned shape=0;shape<3;++shape) for(double maximum:{0.,.03})
    for(double units:{.001,1.}) {
      auto source=Shape(shape);
      source.working_length_to_m=units;
      if(shape!=2) source.geometry_m.master_position[0].x=-.1;
      const auto input=BoundsInput(source,maximum);
      ts::NativeSearchBounds bounds;
      ASSERT_EQ(ts::PrepareSearchBounds(input,bounds),ts::Status::Success);
      for(const auto point:{ts::Vec3{0,0,0},ts::Vec3{.04,.04,.04},ts::Vec3{-.2,0,0}}) {
        const auto native=NativeBounds(input,point);
        const double actual[]{bounds.minimum.x,bounds.minimum.y,bounds.minimum.z,
            bounds.maximum.x,bounds.maximum.y,bounds.maximum.z,bounds.inflation};
        for(unsigned i=0;i<7;++i) EXPECT_DOUBLE_EQ(actual[i],native[i]);
        bool within=false;
        ASSERT_EQ(ts::WithinSearchBounds(bounds,point,within),ts::Status::Success);
        EXPECT_EQ(within,native[7]==1);
      }
      if(shape!=2 && maximum==0) {
        ts::CandidateProjection projection;
        ASSERT_EQ(ts::ProjectCandidate(source,projection),ts::Status::Success);
        EXPECT_GT(bounds.inflation,projection.gap_m/units);
      }
      ASSERT_FALSE(HasFailure());
    }
}
TEST(TiedSearchBoundsNative, InclusiveFacesAndNextRepresentableOutsideAgree) {
  auto source=Shape();source.working_length_to_m=1;
  const auto input=BoundsInput(source);
  ts::NativeSearchBounds bounds;
  ASSERT_EQ(ts::PrepareSearchBounds(input,bounds),ts::Status::Success);
  for(const auto point:{ts::Vec3{bounds.minimum.x,0,0},ts::Vec3{bounds.maximum.x,0,0},
      ts::Vec3{0,bounds.minimum.y,0},ts::Vec3{0,bounds.maximum.y,0},
      ts::Vec3{0,0,bounds.minimum.z},ts::Vec3{0,0,bounds.maximum.z}}) {
    bool within=false;
    ASSERT_EQ(ts::WithinSearchBounds(bounds,point,within),ts::Status::Success);
    ASSERT_TRUE(within);EXPECT_EQ(NativeBounds(input,point)[7],1);
    auto outside=point;
    const double infinity=std::numeric_limits<double>::infinity();
    if(point.x) outside.x=std::nextafter(point.x,std::copysign(infinity,point.x));
    if(point.y) outside.y=std::nextafter(point.y,std::copysign(infinity,point.y));
    if(point.z) outside.z=std::nextafter(point.z,std::copysign(infinity,point.z));
    ASSERT_EQ(ts::WithinSearchBounds(bounds,outside,within),ts::Status::Success);
    EXPECT_FALSE(within);EXPECT_EQ(NativeBounds(input,outside)[7],0);
  }
}
TEST(TiedSearchBoundsNative, NativeAreaFloorDoesNotReplaceCandidateBoxAdmission) {
  auto source=Shape();
  for(auto& p:source.geometry_m.master_position) p={};
  source.geometry_m.secondary_position={0,0,1};
  ts::CandidateProjection raw;
  ASSERT_EQ(ts::ProjectCandidate(source,raw),ts::Status::Success);
  ASSERT_TRUE(raw.admissible);  // Native regularization alone has no finite reach here.
  const auto input=BoundsInput(source);
  ts::NativeSearchBounds bounds;
  ASSERT_EQ(ts::PrepareSearchBounds(input,bounds),ts::Status::Success);
  bool within=true;
  ASSERT_EQ(ts::WithinSearchBounds(bounds,source.geometry_m.secondary_position,within),ts::Status::Success);
  EXPECT_FALSE(within);
  EXPECT_EQ(NativeBounds(input,source.geometry_m.secondary_position)[7],0);
}
TEST(TiedSearchBoundsNative, InvalidLastCoordinateAndFiniteOverflowPreserveOutputAndRetry) {
  const auto input=BoundsInput(Shape());
  ts::NativeSearchBounds bounds;
  ASSERT_EQ(ts::PrepareSearchBounds(input,bounds),ts::Status::Success);
  const auto before=Bytes(bounds);
  auto bad=input;bad.master_position_m[3].z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ts::PrepareSearchBounds(bad,bounds),ts::Status::InvalidInput);
  EXPECT_EQ(Bytes(bounds),before);
  bad=input;bad.master_position_m[3].z=1e307;
  EXPECT_EQ(ts::PrepareSearchBounds(bad,bounds),ts::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(bounds),before);
  ASSERT_EQ(ts::PrepareSearchBounds(input,bounds),ts::Status::Success);
  EXPECT_EQ(Bytes(bounds),before);
  bool within=true;
  EXPECT_EQ(ts::WithinSearchBounds(bounds,{0,0,std::numeric_limits<double>::quiet_NaN()},within),ts::Status::InvalidInput);
  EXPECT_TRUE(within);
  ASSERT_EQ(ts::WithinSearchBounds(bounds,{0,0,1},within),ts::Status::Success);
  EXPECT_FALSE(within);
}
} // namespace tied_search_test
