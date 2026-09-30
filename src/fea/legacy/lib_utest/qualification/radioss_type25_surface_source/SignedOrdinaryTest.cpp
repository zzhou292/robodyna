#include "lib_src/collision/RadiossType25Coefficients.h"
#include "lib_utest/qualification/radioss_type25_coefficients/NativeOracle.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
namespace type25_surface_source_test {
namespace n=tlfea::contact::radioss_type25;
TEST(Type25OrdinarySignedVolumeAudit, NativeSolidBranchRetainsSignedCoefficientWhilePublicPositiveContractRejects) {
  for(const int control:{0,1}) {
    n::NativeSolidMainCoefficientInput in;
    in.face=n::MainFaceKind::OrdinaryExterior;in.layout=n::SolidLayout::EightSlot;
    in.scale=1.25;in.fill=.75;in.area=4;in.volume=8;
    in.bulk=100;in.controlled_bulk=250;in.incompressibility_control=control;
    const auto positive=type25_coefficient_test::Oracle(in);
    n::NativeSolidMainCoefficientResult old;
    ASSERT_EQ(n::EvaluateNativeSolidMainCoefficient(in,&old),n::CoefficientStatus::Ok);
    EXPECT_EQ(old.stiffness,positive.stiffness);
    EXPECT_EQ(old.characteristic_length,positive.characteristic_length);
    in.volume=-8;
    const auto actual=type25_coefficient_test::Oracle(in);
    EXPECT_EQ(actual.stiffness,-positive.stiffness);
    EXPECT_EQ(actual.characteristic_length,-positive.characteristic_length);
    EXPECT_LT(actual.stiffness,0);
    EXPECT_FALSE(actual.stiffness>0); // Exact native I25TRIVOX/OPTCD activity predicate, source audit only.
    const auto before=old;
    EXPECT_EQ(n::EvaluateNativeSolidMainCoefficient(in,&old),n::CoefficientStatus::InvalidInput);
    EXPECT_EQ(std::memcmp(&old,&before,sizeof(old)),0);
    in.scale=0;
    const auto zero=type25_coefficient_test::Oracle(in);
    EXPECT_EQ(zero.stiffness,0);EXPECT_TRUE(std::signbit(zero.stiffness));
    EXPECT_FALSE(zero.stiffness>0);
  }
  RecordProperty("scope","Prescribed original solid coefficient branch; no signed ordinary production opt-in, source census or runtime admission");
}
}
