// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
namespace type25_coefficient_test {
TEST(Type25Coefficients, CompleteNativeOrdinaryBranchCorpus) {
  const auto cases = Cases(); ASSERT_EQ(cases.size(), 110u);
  for (std::size_t i = 0; i < cases.size(); ++i) {
    SCOPED_TRACE(i); const auto actual = Evaluate(cases[i]);
    ASSERT_EQ(actual.status, n::CoefficientStatus::Ok); Same(actual, Reference(cases[i]));
  }
}
TEST(Type25Coefficients, ShellOverrideAndPropertySeventeenKeepNativePriority) {
  auto in = Shell(); n::NativeScalarCoefficient out;
  ASSERT_EQ(n::EvaluateNativeShellMainCoefficient(in, &out), n::CoefficientStatus::Ok);
  EXPECT_DOUBLE_EQ(out.value, .25 * 210000);
  in.input_thickness_mode = 1;
  ASSERT_EQ(n::EvaluateNativeShellMainCoefficient(in, &out), n::CoefficientStatus::Ok);
  EXPECT_DOUBLE_EQ(out.value, .5 * 210000);
  in.property_type = 17; in.element_thickness = 0;
  ASSERT_EQ(n::EvaluateNativeShellMainCoefficient(in, &out), n::CoefficientStatus::Ok);
  EXPECT_EQ(out.value, 0.); // Source keeps element THK here, even when it is zero.
}
TEST(Type25Coefficients, SolidControlAndHigherOrderCorrectionsAreDistinctFromContactGap) {
  auto in = Solid(); n::NativeSolidMainCoefficientResult base, out;
  ASSERT_EQ(n::EvaluateNativeSolidMainCoefficient(in, &base), n::CoefficientStatus::Ok);
  in.layout = n::SolidLayout::TenNode;
  ASSERT_EQ(n::EvaluateNativeSolidMainCoefficient(in, &out), n::CoefficientStatus::Ok);
  EXPECT_DOUBLE_EQ(out.stiffness, 16 * base.stiffness);
  EXPECT_DOUBLE_EQ(out.characteristic_length, .375 * base.characteristic_length);
  in.layout = n::SolidLayout::SixteenNode; in.incompressibility_control = 1;
  ASSERT_EQ(n::EvaluateNativeSolidMainCoefficient(in, &out), n::CoefficientStatus::Ok);
  Number(out.stiffness, 2 * base.stiffness); Number(out.characteristic_length, base.characteristic_length/4);
}
TEST(Type25Coefficients, GlobalAccumulatedOperandsRetainExistingContributionsAndSeparateNormalization) {
  const n::NativeAccumulatedNodalCoefficients in{4., 20., 21., 3, 11.};
  n::NativeNodalCoefficientResult out;
  ASSERT_EQ(n::FinalizeNativeNodalCoefficient(in, &out), n::CoefficientStatus::Ok);
  Number(out.normalized_bulk, 5.); Number(out.stiffness, 11. + 5. * std::pow(8., 1./3.) + 7.);
  EXPECT_EQ(in.bulk_volume, 20.); EXPECT_EQ(in.existing_stiffness, 11.);
  n::NativeNodalCoefficientResult prior{7, 9}; out = prior;
  auto overflow = in; overflow.bulk_volume = std::numeric_limits<double>::max(); overflow.volume = 1e-30;
  EXPECT_EQ(n::FinalizeNativeNodalCoefficient(overflow, &out), n::CoefficientStatus::NonfiniteResult);
  Number(out.normalized_bulk, prior.normalized_bulk, true); Number(out.stiffness, prior.stiffness, true);
}
TEST(Type25Coefficients, SecondaryZeroRemovalIsExactAndDoesNotConsumeUnusedGlobalValue) {
  for (double zero : {0., -0.}) {
    n::NativeSecondaryCoefficientInput in{zero, std::numeric_limits<double>::quiet_NaN(), -1.};
    n::NativeScalarCoefficient out{7};
    ASSERT_EQ(n::EvaluateNativeSecondaryCoefficient(in, &out), n::CoefficientStatus::Ok);
    Number(out.value, zero, true); Number(out.value, Oracle(in).value, true);
  }
  n::NativeScalarCoefficient out;
  ASSERT_EQ(n::EvaluateNativeSecondaryCoefficient({-2., 17., 0.}, &out), n::CoefficientStatus::Ok);
  EXPECT_EQ(out.value, 17.); // Negative nonzero input is replaced, never treated as removal.
}
TEST(Type25Coefficients, ObservedPairControlsAdmitOnlyTheProvedMassIndependentBranch) {
  const n::NativePairCoefficientInput observed{2800, 2410, 0, 1e30};
  n::NativeScalarCoefficient out;
  ASSERT_EQ(n::EvaluateNativePairCoefficient({4, 0}, observed, &out), n::CoefficientStatus::Ok);
  EXPECT_EQ(out.value, 2410.); Number(out.value, Oracle(observed).value, true);
  for (auto profile : {n::PairCoefficientProfile{}, n::PairCoefficientProfile{6, 0},
                       n::PairCoefficientProfile{4, 1}}) {
    out.value = 97.;
    EXPECT_EQ(n::EvaluateNativePairCoefficient(profile, observed, &out), n::CoefficientStatus::UnsupportedProfile);
    EXPECT_EQ(out.value, 97.);
  }
}
TEST(Type25Coefficients, UnsupportedSurfaceAndMaterialBranchesPreserveOutputs) {
  for (auto face : {n::MainFaceKind::Unspecified, n::MainFaceKind::Coating, n::MainFaceKind::Internal}) {
    auto shell = Shell(); shell.face = face; n::NativeScalarCoefficient out{73};
    EXPECT_EQ(n::EvaluateNativeShellMainCoefficient(shell, &out), n::CoefficientStatus::UnsupportedProfile);
    EXPECT_EQ(out.value, 73.);
    auto solid = Solid(); solid.face = face; n::NativeSolidMainCoefficientResult result{71, 79};
    EXPECT_EQ(n::EvaluateNativeSolidMainCoefficient(solid, &result), n::CoefficientStatus::UnsupportedProfile);
    EXPECT_EQ(result.stiffness, 71.); EXPECT_EQ(result.characteristic_length, 79.);
  }
  for (int property : {11, 17, 51, 52}) {
    auto in = Shell(); in.property_type = property; in.stack_material = 1;
    n::NativeScalarCoefficient out{73};
    EXPECT_EQ(n::EvaluateNativeShellMainCoefficient(in, &out), n::CoefficientStatus::UnsupportedProfile);
    EXPECT_EQ(out.value, 73.);
  }
}
TEST(Type25Coefficients, InvalidOperandsAndLateOverflowNeverPublishPartialResults) {
  auto shell = Shell(); n::NativeScalarCoefficient scalar{73}; shell.young = std::numeric_limits<double>::infinity();
  EXPECT_EQ(n::EvaluateNativeShellMainCoefficient(shell, &scalar), n::CoefficientStatus::InvalidInput);
  EXPECT_EQ(scalar.value, 73.);
  shell = Shell(); shell.scale = std::numeric_limits<double>::max();
  EXPECT_EQ(n::EvaluateNativeShellMainCoefficient(shell, &scalar), n::CoefficientStatus::NonfiniteResult);
  EXPECT_EQ(scalar.value, 73.);
  shell = Shell(); shell.scale = 2.;
  shell.element_thickness = std::numeric_limits<double>::max(); shell.young = 0.;
  EXPECT_EQ(n::EvaluateNativeShellMainCoefficient(shell, &scalar), n::CoefficientStatus::NonfiniteResult);
  EXPECT_EQ(scalar.value, 73.); // Overflow then zero must not disappear through MAX.
  auto solid = Solid(); solid.volume = 0; n::NativeSolidMainCoefficientResult out{71, 79};
  EXPECT_EQ(n::EvaluateNativeSolidMainCoefficient(solid, &out), n::CoefficientStatus::InvalidInput);
  EXPECT_EQ(out.stiffness, 71.); EXPECT_EQ(out.characteristic_length, 79.);
  solid = Solid(); solid.area = std::numeric_limits<double>::max();
  EXPECT_EQ(n::EvaluateNativeSolidMainCoefficient(solid, &out), n::CoefficientStatus::NonfiniteResult);
  EXPECT_EQ(out.stiffness, 71.); EXPECT_EQ(out.characteristic_length, 79.);
  EXPECT_EQ(n::EvaluateNativePairCoefficient({4, 0}, {1, 2, 3, 1}, &scalar), n::CoefficientStatus::InvalidInput);
  EXPECT_EQ(scalar.value, 73.);
  EXPECT_EQ(n::EvaluateNativeShellMainCoefficient(Shell(), nullptr), n::CoefficientStatus::InvalidInput);
  EXPECT_EQ(n::EvaluateNativeSolidMainCoefficient(Solid(), nullptr), n::CoefficientStatus::InvalidInput);
}
} // namespace type25_coefficient_test
