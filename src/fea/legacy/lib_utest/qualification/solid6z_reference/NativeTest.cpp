// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <gtest/gtest.h>
namespace solid6z_test {
TEST(Solid6zNative, CompleteSelectedFrameJacobianGeometryAndMass) {
  auto reverse=Distorted();
  for (unsigned n=0;n<3;++n) std::swap(reverse.position_m[n],reverse.position_m[n+3]);
  for (const auto& input:{Wedge(),Distorted(),reverse}) {
    const auto native=Native(input);
    ASSERT_EQ(native.status,0);
    s::Reference result;
    ASSERT_EQ(s::InitializeReference(input,result),s::Status::Success);
    ASSERT_TRUE(Agree(Values(result),native.values));
    for (unsigned n=0;n<6;++n) EXPECT_EQ(result.source_slot(n),unsigned(native.permutation[n]));
  }
}
TEST(Solid6zNative, ImasZeroIsExplicitAndDiffersFromUnequalAngleWeights) {
  const auto input=Wedge();
  const auto plain=Native(input),angular=Native(input,1);
  ASSERT_EQ(plain.status,0); ASSERT_EQ(angular.status,0);
  EXPECT_EQ(plain.values[46],angular.values[46]);
  EXPECT_NE(angular.values[40],angular.values[41]);
  EXPECT_NE(plain.values[40],angular.values[40]);
  s::Reference result; auto rejected=input; rejected.profile.mass_distribution=1;
  EXPECT_EQ(s::InitializeReference(rejected,result),s::Status::UnsupportedProfile);
}
TEST(Solid6zNative, NativeCollapsedRejectionAndSourceFirstFaceYOrder) {
  auto bad=Wedge(); for (auto& x:bad.position_m) x.z=0;
  EXPECT_NE(Native(bad).status,0);
  const auto input=FirstFaceControl(); const auto native=Native(input);
  ASSERT_EQ(native.status,0);
  s::Reference result;
  ASSERT_EQ(s::InitializeReference(input,result),s::Status::Success);
  ASSERT_TRUE(Agree(Values(result),native.values));
  namespace brick=tl::fea::solid_common;
  const auto& x=result.geometry().local_position_m;
  const double corrected[5]{brick::FaceMeasure(x[0],x[1],x[4],x[3]),
    brick::FaceMeasure(x[1],x[4],x[5],x[2]),brick::FaceMeasure(x[0],x[3],x[5],x[2]),
    brick::FaceMeasure(x[0],x[1],x[2],x[2]),brick::FaceMeasure(x[3],x[4],x[5],x[5])};
  const double maximum=*std::max_element(std::begin(corrected),std::end(corrected));
  const double alternate=4*result.geometry().volume_m3/std::sqrt(maximum);
  EXPECT_GT(std::abs(alternate-native.values[39]),1e-8);
}
}  // namespace solid6z_test
