#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
#include <gtest/gtest.h>
#include <array>
namespace {
namespace native=tl::fea::native_physical_coefficients;
using Info=tl::fea::NodalRigidGroupInfo;
using Mass=tlfea::contact::TranslationMassModel;
TEST(NativeCoefficientAdmission, ExactCompleteScopeWithoutConstrainedMassRetagging) {
  const Info grouped{17,2,6};
  EXPECT_TRUE(native::ValidScope({},8)); EXPECT_TRUE(native::ValidScope(grouped,8));
  EXPECT_TRUE(native::Admitted({}, {},Mass::kIsotropicLumped,8));
  EXPECT_TRUE(native::Admitted(grouped,grouped,Mass::kUnspecified,8));
  EXPECT_FALSE(native::Admitted(grouped,grouped,Mass::kIsotropicLumped,8));
  EXPECT_FALSE(native::Admitted({}, {},Mass::kUnspecified,8));
  EXPECT_FALSE(native::Admitted(grouped,{18,2,6},Mass::kUnspecified,8));
  EXPECT_FALSE(native::Admitted(grouped,{17,1,6},Mass::kUnspecified,8));
  EXPECT_FALSE(native::Admitted(grouped,{17,2,7},Mass::kUnspecified,8));
  for(Info partial:std::array<Info,7>{{{17,0,0},{0,2,6},{17,2,0},{17,0,6},{0,0,6},{17,3,6},{17,1,9}}}) {
    EXPECT_FALSE(native::ValidScope(partial,8));
    EXPECT_FALSE(native::Admitted(partial,partial,Mass::kUnspecified,8));
  }
  EXPECT_FALSE(native::ValidScope({17,SIZE_MAX,SIZE_MAX},SIZE_MAX));
}
TEST(NativeCoefficientAdmission, LegacyTranslationPsdProofStillRejectsGroupedOwnerMass) {
  // Early mass-tag rejection must not read fabricated physical buffers.
  const tlfea::contact::LumpedTranslationMassView mass{reinterpret_cast<const double*>(1),
      reinterpret_cast<const std::uint8_t*>(1),6,0,Mass::kUnspecified};
  tlfea::contact::NormalJacobian out;
  const tlfea::contact::SignedNodeWeight weight{0,1};
  EXPECT_EQ(tlfea::contact::BuildNormalJacobian(mass,&weight,1,{1,0,0},1,&out),
            tlfea::contact::Status::kUnsupportedInterpolation);
  EXPECT_FALSE(out.valid);
}
} // namespace
