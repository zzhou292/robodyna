#include "CoefficientFixture.h"

namespace tied_patch_test {
TEST(TiedPatchCoefficients, InitialMasterInertiaSelectsNativeTransferAndPhysicalLedger) {
  tie::Patch patch;
  ASSERT_EQ(tie::PreparePatch(Geometry(0),patch),tie::Status::Success);
  tie::CoefficientTransfer all_rotational;
  const auto baseline = Coefficients(4);
  ASSERT_EQ(tie::TransferCoefficients(patch,baseline,all_rotational),tie::Status::Success);
  EXPECT_DOUBLE_EQ(all_rotational.numerical_mass_delta,0);
  EXPECT_DOUBLE_EQ(all_rotational.master[0].mass,.25*baseline.secondary.mass);
  EXPECT_GT(all_rotational.master[0].inertia,0);
  for (unsigned missing = 0; missing < 4; ++missing) {
    tie::CoefficientTransfer next;
    ASSERT_EQ(tie::TransferCoefficients(patch,Coefficients(missing),next),tie::Status::Success);
    EXPECT_GT(next.numerical_mass_delta,0);
    EXPECT_DOUBLE_EQ(next.master[0].inertia,0);
    EXPECT_GT(next.master[0].mass,all_rotational.master[0].mass);
    EXPECT_DOUBLE_EQ(next.source_mass,baseline.secondary.mass);
    EXPECT_DOUBLE_EQ(next.source_inertia,baseline.secondary.inertia);
    EXPECT_DOUBLE_EQ(next.dependent.mass,0);
    EXPECT_DOUBLE_EQ(next.dependent.inertia,0);
    EXPECT_DOUBLE_EQ(next.dependent.translational_stiffness,1e-20);
    EXPECT_DOUBLE_EQ(next.dependent.rotational_stiffness,1e-20);
  }
}

TEST(TiedPatchCoefficients, ZeroPhysicalCoefficientsAndTrueRepeatedNodeAssemblyAreExplicit) {
  tie::Patch patch;
  ASSERT_EQ(tie::PreparePatch(Geometry(1),patch),tie::Status::Success);
  for (unsigned mode : {5,6,7}) {
    auto input = Coefficients(mode);
    tie::CoefficientTransfer next;
    ASSERT_EQ(tie::TransferCoefficients(patch,input,next),tie::Status::Success);
    const auto assembled = CoefficientValues(next,true);
    EXPECT_DOUBLE_EQ(assembled[8],2*next.master[2].mass);
    EXPECT_DOUBLE_EQ(assembled[9],2*next.master[2].inertia);
    EXPECT_DOUBLE_EQ(assembled[10],2*next.master[2].translational_stiffness);
    for (unsigned i = 12; i < 16; ++i) EXPECT_DOUBLE_EQ(assembled[i],0);
    Near(assembled[0]+assembled[4]+assembled[8],input.secondary.mass);
  }
}

TEST(TiedPatchCoefficients, InvalidLateInputAndComputedOverflowPreserveEntireOutputAndRetry) {
  tie::Patch patch;
  ASSERT_EQ(tie::PreparePatch(Geometry(0),patch),tie::Status::Success);
  const auto input = Coefficients(4);
  tie::CoefficientTransfer output;
  ASSERT_EQ(tie::TransferCoefficients(patch,input,output),tie::Status::Success);
  const auto saved = Bytes(output);
  for (unsigned fault = 0; fault < 7; ++fault) {
    auto bad = input;
    switch (fault) {
      case 0: bad.initial_master_inertia[3] = std::numeric_limits<double>::quiet_NaN(); break;
      case 1: bad.initial_master_inertia[0] = -1; break;
      case 2: bad.secondary.mass = -1; break;
      case 3: bad.secondary.inertia = std::numeric_limits<double>::infinity(); break;
      case 4: bad.secondary.translational_stiffness = -1; break;
      case 5: bad.secondary.rotational_stiffness = std::numeric_limits<double>::quiet_NaN(); break;
      case 6: bad.secondary.rotational_stiffness = std::numeric_limits<double>::max(); break;
    }
    EXPECT_EQ(tie::TransferCoefficients(patch,bad,output), fault == 6 ?
        tie::Status::NonfiniteResult : tie::Status::InvalidInput);
    EXPECT_EQ(Bytes(output),saved);
  }
  EXPECT_EQ(tie::TransferCoefficients(tie::Patch{},input,output),tie::Status::InvalidInput);
  EXPECT_EQ(Bytes(output),saved);
  ASSERT_EQ(tie::TransferCoefficients(patch,input,output),tie::Status::Success);
  tie::CoefficientTransfer reference;
  ASSERT_EQ(tie::TransferCoefficients(patch,input,reference),tie::Status::Success);
  EXPECT_EQ(CoefficientValues(output,false),CoefficientValues(reference,false));
}
} // namespace tied_patch_test
