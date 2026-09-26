// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
namespace reader_solid_test {
TEST(ReaderSolid, MatchesOriginalSignedBranchesAndSecondVolume) {
  unsigned count=0;
  for(const auto& c:Cases()) {SCOPED_TRACE(count++);const auto expected=Oracle(c);
    if(c.internal)EXPECT_DOUBLE_EQ(expected.second_volume,c.input.second_volume);
    n::NativeSolidMainCoefficientResult actual;
    ASSERT_EQ(Evaluate(c,&actual),n::CoefficientStatus::Ok);Same(actual,expected.value);
  }
}
TEST(ReaderSolid, FirstControlDoesNotInventSecondControl) {
  auto c=Base();c.input.first.incompressibility_control=1;
  const auto first=Oracle(c);c.unused_second_controlled_bulk=1e20;
  const auto second=Oracle(c);Same(first.value,second.value);
  n::NativeSolidMainCoefficientResult actual;ASSERT_EQ(Evaluate(c,&actual),n::CoefficientStatus::Ok);
  Same(actual,second.value);
  c.input.second_bulk*=3;const auto changed=Oracle(c);EXPECT_NE(changed.value.stiffness,first.value.stiffness);
  ASSERT_EQ(Evaluate(c,&actual),n::CoefficientStatus::Ok);Same(actual,changed.value);
}
TEST(ReaderSolid, InvalidOrUnsupportedInputsLeaveOutputUntouched) {
  for(bool internal:{false,true})for(unsigned fault=0;fault<10;++fault) {
    auto c=Base(internal);auto& a=c.input.first;
    switch(fault){case 0:a.volume=0;break;case 1:a.area=0;break;case 2:a.fill=-1;break;
      case 3:a.bulk=std::numeric_limits<double>::quiet_NaN();break;
      case 4:a.layout=n::SolidLayout::TenNode;break;
      case 5:a.face=internal?n::MainFaceKind::OrdinaryExterior:n::MainFaceKind::Internal;break;
      case 6:if(internal)c.input.second_volume=0;else a.volume=-0.;break;
      case 7:if(internal)c.input.second_fill=-1;else a.scale=-1;break;
      case 8:if(internal)c.input.second_bulk=-1;else a.controlled_bulk=-1;break;
      case 9:a.area=std::numeric_limits<double>::max();break;}
    n::NativeSolidMainCoefficientResult actual{11,13};EXPECT_NE(Evaluate(c,&actual),n::CoefficientStatus::Ok);
    EXPECT_DOUBLE_EQ(actual.stiffness,11);EXPECT_DOUBLE_EQ(actual.characteristic_length,13);
  }
  EXPECT_EQ(Evaluate(Base(),nullptr),n::CoefficientStatus::InvalidInput);
}
TEST(ReaderSolid, LegacyExteriorAdmissionAndArithmeticStayUnchanged) {
  auto c=Base(false);n::NativeSolidMainCoefficientResult old_value,new_value;
  ASSERT_EQ(n::EvaluateNativeSolidMainCoefficient(c.input.first,&old_value),n::CoefficientStatus::Ok);
  ASSERT_EQ(Evaluate(c,&new_value),n::CoefficientStatus::Ok);Same(old_value,new_value);
  c.input.first.volume=-3;
  EXPECT_EQ(n::EvaluateNativeSolidMainCoefficient(c.input.first,&old_value),n::CoefficientStatus::InvalidInput);
  ASSERT_EQ(Evaluate(c,&new_value),n::CoefficientStatus::Ok);Same(new_value,Oracle(c).value);
}
}
