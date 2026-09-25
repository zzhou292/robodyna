#include "Fixture.h"
#include <gtest/gtest.h>
using namespace candidate_test;
TEST(NativeCandidateFilter,AllNativePackedOutputsAndMembership) {
  const auto cases=Cases();std::size_t positive=0,quad_positive=0;
  for(std::size_t i=0;i<cases.size();++i) {
    SCOPED_TRACE(i);
    SCOPED_TRACE(cases[i].family);
    const auto row=Pack(cases[i]);c::FilterResult out;
    ASSERT_EQ(c::EvaluatePacked(row,&out),c::Status::Ok);
    for(unsigned mode=0;mode<4;++mode) {
      const auto expected=pen3_test::Evaluate(cases[i],mode)[0];
      ASSERT_EQ(Bits(out.squared_clearance),Bits(expected));
      ASSERT_EQ(out.included,expected!=0.);
    }
    positive+=out.included;quad_positive+=out.included&&row.nodes[2]!=row.nodes[3];
  }
  EXPECT_GT(positive,704u);EXPECT_GT(quad_positive,100u);
}
TEST(NativeCandidateScreen,NativeStrictScreensAndSweepContainment) {
  for(const auto& item:Cases()) {
    const auto packet=Pack(item);c::ScreenRow row;
    for(unsigned i=0;i<4;++i)row.vertices[i]=packet.vertices[i];
    row.secondary=packet.secondary;row.secondary_gap=packet.gap;row.margin=packet.margin;
    for(double load:{-.125,0.,.125})for(double drad:{0.,.5}) {
      row.gap_load=load;row.drad=drad;row.curvature=.125;row.stored_motion=.03125;
      bool included=false;ASSERT_EQ(c::EvaluateScreen(row,&included),c::Status::Ok);
      ASSERT_EQ(included,NativeScreen(row));
      c::Envelope result;
      auto envelope=row;envelope.secondary_gap=std::max(1.,row.secondary_gap);
      ASSERT_EQ(c::ScreenBounds(envelope,&result),c::Status::Ok);
      const auto bounds=result.bounds;
      if(included) {
        EXPECT_GE(row.secondary.x,bounds.minimum.x);EXPECT_LE(row.secondary.x,bounds.maximum.x);
        EXPECT_GE(row.secondary.y,bounds.minimum.y);EXPECT_LE(row.secondary.y,bounds.maximum.y);
        EXPECT_GE(row.secondary.z,bounds.minimum.z);EXPECT_LE(row.secondary.z,bounds.maximum.z);
      }
    }
  }
}
TEST(NativeCandidateScreen,ExactAndUlpExpandedBoundaries) {
  c::ScreenRow row;row.vertices[0]={-1,-1,0};row.vertices[1]={1,-1,0};
  row.vertices[2]={1,1,0};row.vertices[3]={-1,1,0};row.secondary_gap=.5;
  for(unsigned axis=0;axis<3;++axis)for(unsigned side=0;side<2;++side) {
    double edge=(axis==2?.5:1.5)*(side?-1.:1.);
    for(double value:{edge,std::nextafter(edge,0.),std::nextafter(edge,side?-INFINITY:INFINITY)}) {
      row.secondary={0,0,0};
      if(axis==0)row.secondary.x=value;else if(axis==1)row.secondary.y=value;else row.secondary.z=value;
      bool included=true;ASSERT_EQ(c::EvaluateScreen(row,&included),c::Status::Ok);
      EXPECT_EQ(included,NativeScreen(row));
      if(value==edge)EXPECT_FALSE(included);
    }
  }
}
TEST(NativeCandidateFilter,FailureIsAtomicAndNonfiniteIsRejected) {
  auto row=Pack(Cases().front());c::FilterResult out{123.,true};
  row.gap=-1.;EXPECT_EQ(c::EvaluatePacked(row,&out),c::Status::InvalidInput);
  EXPECT_EQ(out.squared_clearance,123.);EXPECT_TRUE(out.included);
  row.gap=std::numeric_limits<double>::max();
  EXPECT_EQ(c::EvaluatePacked(row,&out),c::Status::NonfiniteResult);EXPECT_EQ(out.squared_clearance,123.);
  row.gap=1.;row.vertices[3].x+=1.;
  EXPECT_EQ(c::EvaluatePacked(row,&out),c::Status::InvalidInput);EXPECT_EQ(out.squared_clearance,123.);
}
