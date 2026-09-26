// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "lib_src/collision/RadiossType25Coefficients.h"
namespace main_geometry_test {
TEST(MainGeometry, CompleteNativeFieldsAcrossWarpRotationFloorAndRawPenta) {
  const auto cases=Cases();
  ASSERT_EQ(cases.size(),184u);
  for(std::size_t i=0;i<cases.size();++i) {
    SCOPED_TRACE(i);
    auto result=Sentinel();
    ASSERT_EQ(n::EvaluateNativeExteriorMainGeometry(cases[i],&result),n::CoefficientStatus::Ok);
    Same(result,Oracle(cases[i]));
  }
}
TEST(MainGeometry, NativeOrientationAtZeroAndBothNearestSidesPreservesPreFlipNormal) {
  for(bool triangle:{false,true})for(double distance:{-std::numeric_limits<double>::denorm_min(),
        -0.,0.,std::numeric_limits<double>::denorm_min()}) {
    SCOPED_TRACE(triangle);
    SCOPED_TRACE(distance);
    auto in=Basic(triangle);
    for(auto& x:in.solid_raw)x.z=distance;
    n::NativeExteriorMainGeometryResult result;
    ASSERT_EQ(n::EvaluateNativeExteriorMainGeometry(in,&result),n::CoefficientStatus::Ok);
    Same(result,Oracle(in));
    EXPECT_EQ(result.reversed,!(distance<0.));
    EXPECT_GT(result.normal_before_orientation.z,0.);
    if(triangle) {
      EXPECT_EQ(result.source_corner[2],2u);EXPECT_EQ(result.source_corner[3],2u);
      EXPECT_EQ(result.source_corner[0],result.reversed?1u:0u);
    } else EXPECT_EQ(result.source_corner[0],result.reversed?3u:0u);
  }
}
TEST(MainGeometry, RealNativeAreaFloorUsesNativeSquaredLengthNotAnSiFloor) {
  auto in=Basic();Transform(in,1.e-11,{},0);
  n::NativeExteriorMainGeometryResult result;
  ASSERT_EQ(n::EvaluateNativeExteriorMainGeometry(in,&result),n::CoefficientStatus::Ok);
  Same(result,Oracle(in));
  Exact(NativeEm20(),1./1.e20);
  Exact(result.area,.5*NativeEm20());
  const double metre_per_native_mm=.001;
  const double physical_area=result.area*(metre_per_native_mm*metre_per_native_mm);
  EXPECT_LT(physical_area,.5*NativeEm20());
  // No SI adapter or source units are inferred by this raw numerical API.
}
TEST(MainGeometry, NativeVolumeReductionCannotBeReplacedByStructuralCyclicReduction) {
  std::size_t different=0;
  for(const auto& in:Cases()) {
    const auto expected=Oracle(in);
    const double other=tl::fea::solid_common::SignedCenterVolume(in.solid_raw);
    different+=Bits(expected.signed_volume)!=Bits(other);
  }
  EXPECT_GT(different,0u);
}
TEST(MainGeometry, FiniteZeroAndNegativeRawVolumeDoNotAuthorizeCoefficientAdmission) {
  for(bool zero:{false,true}) {
    auto in=Basic();
    for(auto& x:in.solid_raw) { x.x=-x.x;if(zero)x.z=0.; }
    n::NativeExteriorMainGeometryResult result;
    ASSERT_EQ(n::EvaluateNativeExteriorMainGeometry(in,&result),n::CoefficientStatus::Ok);
    Same(result,Oracle(in));EXPECT_LE(result.signed_volume,0.);
    n::NativeSolidMainCoefficientInput coefficient;
    coefficient.face=n::MainFaceKind::OrdinaryExterior;coefficient.layout=n::SolidLayout::EightSlot;
    coefficient.area=result.area;coefficient.volume=result.signed_volume;coefficient.bulk=1;
    n::NativeSolidMainCoefficientResult retained{71.,19.};
    EXPECT_EQ(n::EvaluateNativeSolidMainCoefficient(coefficient,&retained),n::CoefficientStatus::InvalidInput);
    Exact(retained.stiffness,71.);Exact(retained.characteristic_length,19.);
  }
}
TEST(MainGeometry, RejectionPreservesEveryPublishedFieldAndInputThenRetry) {
  auto cases=Invalid();
  for(unsigned i=0;i<cases.size();++i) {
    SCOPED_TRACE(i);
    const auto before=cases[i];auto result=Sentinel();
    EXPECT_EQ(n::EvaluateNativeExteriorMainGeometry(cases[i],&result),InvalidStatus(i));
    Same(result,Sentinel());EXPECT_EQ(std::memcmp(&before,&cases[i],sizeof(before)),0);
    EXPECT_EQ(n::EvaluateNativeExteriorMainGeometry(Basic(),&result),n::CoefficientStatus::Ok);
    Same(result,Oracle(Basic()));
  }
  EXPECT_EQ(n::EvaluateNativeExteriorMainGeometry(Basic(),nullptr),n::CoefficientStatus::InvalidInput);
}
} // namespace main_geometry_test
