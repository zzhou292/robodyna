// SPDX-License-Identifier: MIT
#include "Fixture.h"
namespace tied_search_test {
TEST(TiedSearch, StrictDistanceAndParameterBoundaries) {
  auto in=Shape(0,1.);in.working_length_to_m=1;in.master_thickness_m=1;
  ts::CandidateProjection out;
  in.geometry_m.secondary_position={1.5,0,0};
  ASSERT_EQ(ts::ProjectCandidate(in,out),ts::Status::Success);
  EXPECT_DOUBLE_EQ(out.s,1.5); EXPECT_FALSE(out.admissible); EXPECT_TRUE(out.outside_warning);
  in.geometry_m.secondary_position={1.49,0,0};
  ASSERT_EQ(ts::ProjectCandidate(in,out),ts::Status::Success); EXPECT_TRUE(out.admissible);
  in.master_thickness_m=.05;
  const double gap=.05*std::sqrt(8.);
  in.geometry_m.secondary_position={0,0,gap};
  ASSERT_EQ(ts::ProjectCandidate(in,out),ts::Status::Success); EXPECT_EQ(out.penetration_m,0);
  EXPECT_FALSE(out.admissible);
  in.geometry_m.secondary_position.z=std::nextafter(gap,0.);
  ASSERT_EQ(ts::ProjectCandidate(in,out),ts::Status::Success); EXPECT_TRUE(out.admissible);
}
TEST(TiedSearch, OrderedTiesRetainFirstAndMoreCentralCandidateWins) {
  auto in=Shape();in.geometry_m.secondary_position={.005,0,.001};
  ts::SearchChoice a,b;
  ASSERT_EQ(ts::ConsiderCandidate(in,91,a),ts::Status::Success);
  ASSERT_EQ(ts::ConsiderCandidate(in,12,a),ts::Status::Success); EXPECT_EQ(a.ordered_master,91);
  ASSERT_EQ(ts::ConsiderCandidate(in,12,b),ts::Status::Success);
  ASSERT_EQ(ts::ConsiderCandidate(in,91,b),ts::Status::Success); EXPECT_EQ(b.ordered_master,12);
  for(auto& p:in.geometry_m.master_position) p.x+=.005;
  ASSERT_EQ(ts::ConsiderCandidate(in,63,a),ts::Status::Success);
  EXPECT_EQ(a.ordered_master,63);EXPECT_NEAR(a.projection.s,0,1e-15);
}
TEST(TiedSearch, LateInvalidInputAndUnitMismatchPreservePriorChoiceThenRetry) {
  auto in=Shape(2);ts::SearchChoice choice;
  ASSERT_EQ(ts::ConsiderCandidate(in,17,choice),ts::Status::Success);
  const auto saved=Bytes(choice);
  for(unsigned kind=0;kind<5;++kind) {
    auto bad=in;
    if(kind==0) bad.geometry_m.master_position[3].x+=1;
    if(kind==1) bad.geometry_m.master_position[3].z=std::numeric_limits<double>::quiet_NaN();
    if(kind==2) bad.working_length_to_m=0;
    if(kind==3) bad.working_length_to_m=1;
    if(kind==4) bad.master_thickness_m=-1;
    EXPECT_NE(ts::ConsiderCandidate(bad,99,choice),ts::Status::Success);
    EXPECT_EQ(Bytes(choice),saved);
  }
  ASSERT_EQ(ts::ConsiderCandidate(in,99,choice),ts::Status::Success);EXPECT_EQ(Bytes(choice),saved);
}
TEST(TiedSearch, FiniteOverflowAndRoundedTriangleCannotPublishAFalseMatch) {
  const auto good=Shape();
  ts::CandidateProjection projection;
  ASSERT_EQ(ts::ProjectCandidate(good,projection),ts::Status::Success);
  const auto expected=projection;
  const auto saved=Bytes(projection);
  const auto inputs=InvalidFiniteInputs();
  ASSERT_EQ(inputs[2].geometry_m.master_position[2].x/.001,
      inputs[2].geometry_m.master_position[3].x/.001);
  for(unsigned i=0;i<inputs.size();++i) {
    EXPECT_EQ(ts::ProjectCandidate(inputs[i],projection),
        i<2?ts::Status::NonfiniteResult:ts::Status::InvalidInput);
    EXPECT_EQ(Bytes(projection),saved);
    ts::SearchChoice choice;
    const auto before=Bytes(choice);
    EXPECT_NE(ts::ConsiderCandidate(inputs[i],99,choice),ts::Status::Success);
    EXPECT_EQ(Bytes(choice),before);
  }
  ASSERT_EQ(ts::ProjectCandidate(good,projection),ts::Status::Success);
  Compare(projection,expected);
}
} // namespace tied_search_test
