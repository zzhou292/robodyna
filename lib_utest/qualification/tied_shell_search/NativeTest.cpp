// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
namespace tied_search_test {
TEST(TiedSearchNative, CompleteProjectionForWarpedQuadTriangleEdgesAndCorners) {
  for(unsigned shape=0;shape<3;++shape) for(int ix=-9;ix<=9;++ix)
    for(int iy=-9;iy<=9;++iy) for(double z:{-.015,-.001,0.,.001,.015}) {
      SCOPED_TRACE(shape);
      SCOPED_TRACE(ix);
      SCOPED_TRACE(iy);
      SCOPED_TRACE(z);
      auto in=Shape(shape);in.geometry_m.secondary_position={ix*.002,iy*.002,z};
      ts::CandidateProjection actual;NativeChoice native;
      ASSERT_EQ(ts::ProjectCandidate(in,actual),ts::Status::Success);
      Compare(actual,Native(in,1,native));
      ASSERT_FALSE(HasFailure());
      EXPECT_EQ(actual.admissible,native.selected==1);
    }
}
TEST(TiedSearchNative, OriginalUnitFloorsAndTriangleTipBranches) {
  bool unit_sensitive=false;
  for(double scale:{1e-8,1e-11,1e-13}) for(unsigned shape:{0u,2u}) {
    ts::CandidateProjection previous;
    for(double unit:{.001,1.}) {
      auto in=Shape(shape,scale);in.working_length_to_m=unit;
      ts::CandidateProjection actual;NativeChoice native;
      ASSERT_EQ(ts::ProjectCandidate(in,actual),ts::Status::Success);
      Compare(actual,Native(in,1,native));
      if(unit==1. && (actual.s!=previous.s || actual.t!=previous.t)) unit_sensitive=true;
      previous=actual;
    }
  }
  EXPECT_TRUE(unit_sensitive);
  for(double x:{-.01,-1e-12,0.,1e-12,.01}) {
    auto in=Shape(2);in.geometry_m.secondary_position={x,.01,.0001};
    ts::CandidateProjection actual;NativeChoice native;
    ASSERT_EQ(ts::ProjectCandidate(in,actual),ts::Status::Success);Compare(actual,Native(in,1,native));
  }
}
TEST(TiedSearchNative, IndependentOrderedSelectionAndStrictBoundaryControls) {
  for(bool reversed:{false,true}) {
    ts::SearchChoice actual;NativeChoice native;
    auto in=Shape();in.geometry_m.secondary_position={.005,0,.001};
    for(int id:reversed?std::vector<int>{2,1}:std::vector<int>{1,2}) {
      ASSERT_EQ(ts::ConsiderCandidate(in,id,actual),ts::Status::Success);
      Native(in,id,native);EXPECT_EQ(actual.ordered_master,native.selected);
    }
    for(auto& p:in.geometry_m.master_position) p.x+=.005;
    ASSERT_EQ(ts::ConsiderCandidate(in,3,actual),ts::Status::Success);
    Native(in,3,native);EXPECT_EQ(actual.ordered_master,native.selected);EXPECT_EQ(native.selected,3);
  }
  for(double x:{-1.5,std::nextafter(-1.5,0.),1.02,1.5,std::nextafter(1.5,0.)}) {
    auto in=Shape(0,1.);in.working_length_to_m=1;in.master_thickness_m=1;
    in.geometry_m.secondary_position={x,0,0};
    ts::CandidateProjection actual;NativeChoice native;
    ASSERT_EQ(ts::ProjectCandidate(in,actual),ts::Status::Success);Compare(actual,Native(in,1,native));
  }
}
} // namespace tied_search_test
