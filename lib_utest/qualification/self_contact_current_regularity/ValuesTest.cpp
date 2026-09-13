// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "Oracle.h"

#include <cfloat>
#include <cstring>

namespace current_regularity_test {

TEST(SelfContactCurrentRegularityValues,
    CollinearReversedAndNearDegenerateFacetsRejectExactly) {
  const c::Vec3 regular[3]{{0,0,0},{2,0,0},{0,1,0}};
  c::SelfContactCurrentFacetWitness output;
  ASSERT_EQ(c::EvaluateCurrentFacetRegularity(
      regular,{0,0,1},&output),c::SelfContactCurrentFacetStatus::Ok);
  EXPECT_EQ(output.exact_orientation_sign,1);
  EXPECT_GT(output.double_area_m2.lower,0);
  EXPECT_GT(oracle::DirectedTriangle(
      regular[0],regular[1],regular[2],{0,0,1}),0);

  c::SelfContactCurrentFacetWitness held=output;
  const auto before=held;
  const c::Vec3 collinear[3]{{0,0,0},{1,0,0},{2,0,0}};
  EXPECT_EQ(c::EvaluateCurrentFacetRegularity(
      collinear,{0,0,1},&held),
      c::SelfContactCurrentFacetStatus::Degenerate);
  EXPECT_EQ(std::memcmp(&held,&before,sizeof(held)),0);

  const c::Vec3 reversed[3]{{0,0,0},{0,1,0},{2,0,0}};
  EXPECT_EQ(c::EvaluateCurrentFacetRegularity(
      reversed,{0,0,1},&held),
      c::SelfContactCurrentFacetStatus::Reversed);
  EXPECT_EQ(std::memcmp(&held,&before,sizeof(held)),0);
  EXPECT_LT(oracle::DirectedTriangle(
      reversed[0],reversed[1],reversed[2],{0,0,1}),0);

  const c::Vec3 skinny[3]{{0,0,0},{1,0,0},{1,DBL_EPSILON,0}};
  EXPECT_EQ(c::EvaluateCurrentFacetRegularity(
      skinny,{0,0,1},&held),
      c::SelfContactCurrentFacetStatus::Degenerate);
  EXPECT_EQ(std::memcmp(&held,&before,sizeof(held)),0);
}

TEST(SelfContactCurrentRegularityValues,
    ExactZeroChartProjectionIsNotApproximatedAsRegular) {
  const c::Vec3 triangle[3]{{0,0,0},{1,0,0},{0,1,0}};
  c::SelfContactCurrentFacetWitness output;
  output.scaled_jacobian_quality=123;
  const auto before=output;
  EXPECT_EQ(c::EvaluateCurrentFacetRegularity(
      triangle,{1,0,0},&output),
      c::SelfContactCurrentFacetStatus::Reversed);
  EXPECT_EQ(std::memcmp(&output,&before,sizeof(output)),0);
}

} // namespace current_regularity_test
