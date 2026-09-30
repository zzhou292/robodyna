// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
namespace coated_coefficient_test {
TEST(CoatedMainCoefficients, NativeBothLayoutsOverridesDominanceAndSignedZero) {
  const auto cases=Cases();ASSERT_EQ(cases.size(),53u);
  for(std::size_t i=0;i<cases.size();++i) {
    SCOPED_TRACE(i);n::NativeCoatedMainCoefficientResult actual;
    ASSERT_EQ(n::EvaluateNativeCoatedMainCoefficient(cases[i],&actual),n::CoefficientStatus::Ok);
    Same(actual,Oracle(cases[i]),true);
  }
}
TEST(CoatedMainCoefficients, PrimaryAndPartnerAreDistinctAndGapIsNotContactThickness) {
  auto p=Base();n::NativeCoatedMainCoefficientResult out;
  ASSERT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::Ok);
  EXPECT_EQ(out.primary_stiffness,175000);EXPECT_EQ(out.partner_stiffness,52500);
  EXPECT_EQ(out.solid_characteristic_length,2);
  p.solid.bulk=1;
  ASSERT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::Ok);
  EXPECT_EQ(out.primary_stiffness,out.partner_stiffness);
}
TEST(CoatedMainCoefficients, UnqualifiedProfilesAndInvalidOperandsPreserveOutput) {
  n::NativeCoatedMainCoefficientResult out{11,13,17};const auto before=out;
  auto p=Base();p.solid.face=n::MainFaceKind::Internal;
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::UnsupportedProfile);Same(out,before,true);
  p=Base();p.solid.layout=n::SolidLayout::TenNode;
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::UnsupportedProfile);Same(out,before,true);
  p=Base();p.shell.face=n::MainFaceKind::OrdinaryExterior;
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::UnsupportedProfile);Same(out,before,true);
  p=Base();p.solid.scale=2;
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::InvalidInput);Same(out,before,true);
  p=Base();p.shell.scale=0;p.solid.scale=-0.;
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::InvalidInput);Same(out,before,true);
  p=Base();p.solid.volume=0;
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::InvalidInput);Same(out,before,true);
  p=Base();p.solid.bulk=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::InvalidInput);Same(out,before,true);
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(Base(),nullptr),n::CoefficientStatus::InvalidInput);
}
TEST(CoatedMainCoefficients, LateOverflowAndStackProfilesCannotPartiallyPublishThenRetry) {
  auto p=Base();n::NativeCoatedMainCoefficientResult out{11,13,17};const auto before=out;
  p.solid.area=std::numeric_limits<double>::max();
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::NonfiniteResult);Same(out,before,true);
  p=Base();p.shell.scale=p.solid.scale=2;p.shell.element_thickness=std::numeric_limits<double>::max();p.shell.young=0;
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::NonfiniteResult);Same(out,before,true);
  p=Base();p.shell.property_type=52;
  EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::UnsupportedProfile);Same(out,before,true);
  p=Base();ASSERT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&out),n::CoefficientStatus::Ok);Same(out,Oracle(p),true);
}
TEST(CoatedMainCoefficients, SignedReaderVolumeUsesShellMaxWithoutChangingOrdinarySolidAdmission) {
  auto p=Base();p.solid.volume=-8.;
  n::NativeCoatedMainCoefficientResult result;
  ASSERT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&result),n::CoefficientStatus::Ok);
  Same(result,Oracle(p),true);
  EXPECT_EQ(result.primary_stiffness,52500.);
  EXPECT_EQ(result.partner_stiffness,52500.);
  EXPECT_EQ(result.solid_characteristic_length,-2.);
  n::NativeSolidMainCoefficientResult plain{11,13};
  EXPECT_EQ(n::EvaluateNativeSolidMainCoefficient(p.solid,&plain),n::CoefficientStatus::InvalidInput);
  EXPECT_EQ(plain.stiffness,11.);EXPECT_EQ(plain.characteristic_length,13.);
  const auto before=result;
  for(double volume:{0.,-0.,std::numeric_limits<double>::quiet_NaN()}) {
    p.solid.volume=volume;
    EXPECT_EQ(n::EvaluateNativeCoatedMainCoefficient(p,&result),n::CoefficientStatus::InvalidInput);
    Same(result,before,true);
  }
}

}
