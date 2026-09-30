#include "Fixture.h"
#include "lib_utest/qualification/tied_shell_patch/NativeOracle.h"
namespace cin_test {
TEST(TiedCinNative, MappedReferenceReusesQualifiedNativeLoadMotionAndCoefficientValues) {
  Fixture f;
  tied::TiedCinAttachmentModel out;
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out));
  for (std::size_t row = 0; row < out.rows().count; ++row) {
    const auto& mapped = out.rows().data[row];
    tied::PatchInput input;
    input.secondary_position = out.domain()->nodes()[mapped.secondary_domain_node].position;
    for (std::size_t slot = 0; slot < 4; ++slot)
      input.master_position[slot] = out.domain()->nodes()[mapped.master_domain_nodes[slot]].position;
    const bool triangle = row == 1;
    const auto load = tied_patch_test::Load(2);
    const auto motion = tied_patch_test::Motion(2,triangle);
    const auto native = tied_patch_test::Native(input,load,motion,triangle);
    tied::MasterLoads loads;
    tied::SecondaryMotion velocity;
    ASSERT_EQ(tied::TransferLoad(mapped.reference_patch,load,loads),tied::Status::Success);
    ASSERT_EQ(tied::RecoverMotion(mapped.reference_patch,motion,velocity),tied::Status::Success);
    tied_patch_test::Agreement(mapped.reference_patch,loads,velocity,native);
    const auto coefficient = tied_patch_test::Coefficients(0);
    tied::CoefficientTransfer transfer;
    ASSERT_EQ(tied::TransferCoefficients(mapped.reference_patch,coefficient,transfer),tied::Status::Success);
    const auto expected = tied_patch_test::NativeCoefficients(input,coefficient,triangle);
    const auto values = tied_patch_test::CoefficientValues(transfer,triangle);
    for (std::size_t i = 0; i < values.size(); ++i) tied_patch_test::Near(values[i],expected[i]);
    // A later supplied current patch is prepared separately. The immutable
    // reference model retains its original domain and original patch.
    const auto reference = tied_patch_test::PatchBits(mapped.reference_patch);
    input.master_position[0].z += .003;
    tied::Patch current;
    ASSERT_EQ(tied::PreparePatch(input,current),tied::Status::Success);
    EXPECT_NE(reference,tied_patch_test::PatchBits(current));
    EXPECT_EQ(reference,tied_patch_test::PatchBits(mapped.reference_patch));
  }
}
}
