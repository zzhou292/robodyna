// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Native.h"
#include "lib_src/elements/beam18/Reference.h"
#include "lib_src/collision/RadiossType25GapSource.h"
#include "lib_src/math/ScalarBits.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
namespace beam18_property_test {
namespace beam=tl::fea::beam18;
TEST(Beam18PropertyArea, OriginalPreMassAreaMatchesInEachWorkingUnitSpace) {
  std::vector<double> radii{.001, .0045, .0059, 1., 4.5, 5.9, 100., std::ldexp(1.,-200), std::ldexp(1.,200)};
  for(const double r:{4.5,5.9,.0045,.0059}) {
    radii.push_back(std::nextafter(r,0.));radii.push_back(std::nextafter(r,100.));
  }
  std::size_t sum_differences=0;
  for(const auto radius:radii) {
    SCOPED_TRACE(radius);
    const auto expected=Native(radius);
    double area=-1.;
    ASSERT_EQ(beam::EvaluateCircularPropertyArea(radius,&area),beam::Status::Success);
    EXPECT_TRUE(tl::math::SameScalarBits(area,expected[0]));
    EXPECT_TRUE(tl::math::SameScalarBits(area,expected[1]));
    EXPECT_TRUE(std::isfinite(expected[2]));
    sum_differences+=!tl::math::SameScalarBits(expected[1],expected[2]);
  }
  RecordProperty("sampled_property_vs_point_sum_differences",std::to_string(sum_differences));
  // This reports the sampled phase distinction without inventing a mismatch.
}
TEST(Beam18PropertyArea, InvalidAndUnrepresentableAreaPreserveOutputThenRetry) {
  const auto inf=std::numeric_limits<double>::infinity();
  for(const auto radius:{0.,-0.,-1.,inf,std::numeric_limits<double>::quiet_NaN()}) {
    double area=123.;
    EXPECT_EQ(beam::EvaluateCircularPropertyArea(radius,&area),beam::Status::InvalidInput);
    EXPECT_TRUE(tl::math::SameScalarBits(area,123.));
  }
  for(const auto radius:{std::numeric_limits<double>::max(),std::numeric_limits<double>::denorm_min()}) {
    double area=123.;
    EXPECT_EQ(beam::EvaluateCircularPropertyArea(radius,&area),beam::Status::NonfiniteResult);
    EXPECT_TRUE(tl::math::SameScalarBits(area,123.));
    ASSERT_EQ(beam::EvaluateCircularPropertyArea(4.5,&area),beam::Status::Success);
    EXPECT_TRUE(tl::math::SameScalarBits(area,Native(4.5)[1]));
  }
  EXPECT_EQ(beam::EvaluateCircularPropertyArea(4.5,nullptr),beam::Status::InvalidInput);
}
TEST(Beam18PropertyArea, GenuineAreaFeedsQualifiedGapValueController) {
  namespace gap=tlfea::contact::radioss_type25::source_gaps;
  for(const auto radius:{4.5,5.9,.0045,.0059}) {
    double area;
    ASSERT_EQ(beam::EvaluateCircularPropertyArea(radius,&area),beam::Status::Success);
    gap::Line line{101,{0,1},0.,area};
    const std::uint32_t secondary[]{0,1};
    gap::Input input;
    input.profile={1,0,1,1,0,0,1.,1e30,1e30};input.node_count=2;
    input.beams=&line;input.beam_count=1;input.secondary_nodes=secondary;input.secondary_count=2;
    gap::Forecast forecast;
    ASSERT_EQ(gap::Preflight(input,{},forecast).status,gap::Status::Ok);
    tl::util::HostArena scratch;
    ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));
    double values[2]{-1.,-1.};
    const auto result=gap::Build(input,{},scratch.data(),scratch.bytes(),{values,2,nullptr,0,nullptr,0});
    ASSERT_EQ(result.status,gap::Status::Ok);
    const auto expected=Native(radius);
    EXPECT_TRUE(tl::math::SameScalarBits(values[0],expected[3]));
    EXPECT_TRUE(tl::math::SameScalarBits(values[1],expected[3]));
  }
}
}
