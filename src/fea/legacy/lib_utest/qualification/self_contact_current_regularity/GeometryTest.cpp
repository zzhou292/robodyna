// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "Oracle.h"

#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"

#include <cfloat>

namespace current_regularity_test {
namespace {

void CheckQ4(unsigned level, const std::array<c::Vec3,4>& points,
             bool expect_warp) {
  Fixture fixture(level);
  const auto parent = fixture.FirstParent(4);
  ASSERT_NE(parent,SIZE_MAX);
  fixture.Only(parent);
  fixture.SetParent(parent,points);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  ASSERT_TRUE(receipt.prepared());
  const auto view=fixture.regularity.results();
  ASSERT_TRUE(view.complete);
  const auto& result=view.data[parent];
  EXPECT_EQ(result.chart,
      c::SelfContactCurrentChartStatus::CertifiedQ4FixedDirection);
  EXPECT_EQ(result.state,c::SelfContactCurrentParentState::Active);
  EXPECT_EQ(result.facet_count,2u*(1u<<level)*(1u<<level));
  EXPECT_EQ(result.facets_evaluated,result.facet_count);
  EXPECT_GT(result.current_area_enclosure_m2.lower,0);
  EXPECT_GT(result.minimum_area_witness.double_area_m2.lower,0);
  EXPECT_GT(result.minimum_scaled_jacobian_quality,64*DBL_EPSILON);
  EXPECT_EQ(result.minimum_area_witness.exact_orientation_sign,1);
  EXPECT_EQ(result.approximation.bilinear_error_upper_m>0,expect_warp);
  EXPECT_TRUE(oracle::Q4ChartPositive(points,result.chart_direction));

  for(std::uint32_t local=0;local<result.facet_count;++local) {
    c::FixedContactFacet descriptor;
    ASSERT_EQ(fixture.source.facets.Describe(
        result.surface_parent,local,&descriptor).status,
        c::FixedContactFacetStatus::Ok);
    c::CurrentFixedTriangle triangle;
    ASSERT_EQ(c::EvaluateCurrentFixedTriangle(
        descriptor,fixture.Positions(),&triangle),c::Status::kOk);
    EXPECT_GT(oracle::DirectedTriangle(
        triangle.vertices[0],triangle.vertices[1],triangle.vertices[2],
        result.chart_direction),0);
  }
  EXPECT_EQ(view.summary.certified_parents,1u);
  EXPECT_EQ(view.summary.skipped_parents,view.count-1);
  EXPECT_EQ(view.summary.facets_evaluated,result.facet_count);
}

void CheckT3(unsigned level) {
  Fixture fixture(level);
  const auto parent=fixture.FirstParent(3);
  ASSERT_NE(parent,SIZE_MAX);
  fixture.Only(parent);
  fixture.SetParent(parent,FlatT3);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  const auto& result=fixture.Result(parent);
  EXPECT_EQ(result.chart,
      c::SelfContactCurrentChartStatus::CertifiedT3Native);
  EXPECT_EQ(result.facet_count,(1u<<level)*(1u<<level));
  EXPECT_EQ(result.facets_evaluated,result.facet_count);
  EXPECT_GT(result.current_area_enclosure_m2.lower,0);
  EXPECT_GT(result.minimum_area_witness.double_area_m2.lower,0);
  EXPECT_GT(result.minimum_scaled_jacobian_quality,64*DBL_EPSILON);
  EXPECT_EQ(result.approximation.bilinear_error_upper_m,0);
  for(std::uint32_t local=0;local<result.facet_count;++local) {
    c::FixedContactFacet descriptor;
    ASSERT_EQ(fixture.source.facets.Describe(
        result.surface_parent,local,&descriptor).status,
        c::FixedContactFacetStatus::Ok);
    c::CurrentFixedTriangle triangle;
    ASSERT_EQ(c::EvaluateCurrentFixedTriangle(
        descriptor,fixture.Positions(),&triangle),c::Status::kOk);
    EXPECT_GT(oracle::DirectedTriangle(
        triangle.vertices[0],triangle.vertices[1],triangle.vertices[2],
        result.chart_direction),0);
  }
}

} // namespace

TEST(SelfContactCurrentRegularity,
    FlatSkewWarpedQ4AndT3PassAtLevelsZeroOneTwo) {
  for(const unsigned level:{0u,1u,2u}) {
    CheckQ4(level,FlatQ4,false);
    CheckQ4(level,SkewQ4,false);
    CheckQ4(level,SaddleQ4,true);
    CheckT3(level);
  }
}

TEST(SelfContactCurrentRegularity,
    SaddleHasFixedDirectionChartAndPositiveApproximationMetadataOnly) {
  Fixture fixture(2);
  const auto parent=fixture.FirstParent(4);
  fixture.Only(parent);
  fixture.SetParent(parent,SaddleQ4);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  const auto& result=fixture.Result(parent);
  EXPECT_TRUE(oracle::Q4ChartPositive(SaddleQ4,result.chart_direction));
  EXPECT_GT(result.approximation.bilinear_error_upper_m,0);
  EXPECT_GT(result.approximation.total_error_upper_m,0);
  EXPECT_GT(result.minimum_scaled_jacobian_quality,64*DBL_EPSILON);
}

} // namespace current_regularity_test
