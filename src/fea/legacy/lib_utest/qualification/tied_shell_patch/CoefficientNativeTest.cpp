#include "NativeOracle.h"

namespace tied_patch_test {
TEST(TiedPatchCoefficientsNative, QuadAndRepeatedTriangleMatchBothInertiaBranchesAndZeroInputs) {
  for (unsigned shape = 0; shape < 4; ++shape) {
    const bool repeated = shape == 1;
    const auto geometry = Geometry(shape);
    tie::Patch patch;
    ASSERT_EQ(tie::PreparePatch(geometry,patch),tie::Status::Success);
    for (unsigned mode = 0; mode < 8; ++mode) {
      SCOPED_TRACE(shape);
      SCOPED_TRACE(mode);
      auto input = Coefficients(mode);
      if (repeated) input.initial_master_inertia[3] = input.initial_master_inertia[2];
      tie::CoefficientTransfer output;
      ASSERT_EQ(tie::TransferCoefficients(patch,input,output),tie::Status::Success);
      const auto native = NativeCoefficients(geometry,input,repeated);
      const auto actual = CoefficientValues(output,repeated);
      for (unsigned i = 0; i < actual.size(); ++i) {
        SCOPED_TRACE(i);
        Near(actual[i],native[i]);
      }
      Near(native[23],output.numerical_mass_delta);
      // Native dependent coefficients are exact zero/EM20, not epsilon masses.
      EXPECT_DOUBLE_EQ(actual[16],native[16]); EXPECT_DOUBLE_EQ(actual[17],native[17]);
      EXPECT_DOUBLE_EQ(actual[18],native[18]); EXPECT_DOUBLE_EQ(actual[19],native[19]);
    }
  }
}
} // namespace tied_patch_test
