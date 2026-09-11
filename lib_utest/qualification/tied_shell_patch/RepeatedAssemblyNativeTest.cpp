#include "NativeOracle.h"
#include "RepeatedAssemblyFixture.h"

namespace tied_patch_test {
TEST(TiedPatchAssemblyNative, LoadedPatchForcesAccumulateLikeOriginalNativeIncludingTriangle) {
  for (unsigned shape = 0; shape < 4; ++shape) {
    const bool repeated = shape == 1;
    const auto geometry = Geometry(shape);
    tie::Patch patch;
    ASSERT_EQ(tie::PreparePatch(geometry,patch),tie::Status::Success);
    for (unsigned step = 0; step < 16; ++step) {
      SCOPED_TRACE(shape);
      SCOPED_TRACE(step);
      const auto load = Load(step);
      const auto native = Native(geometry,load,Motion(step,repeated),repeated);
      tie::MasterLoads transferred;
      ASSERT_EQ(tie::TransferLoad(patch,load,transferred),tie::Status::Success);
      AssemblyPacket packet;
      if (!repeated) packet.nodes[3] = 3;
      std::array<double,12> initial{};
      for (unsigned n = 0; n < 4; ++n) {
        packet.force[n] = transferred.force[n];
        for (unsigned c = 0; c < 3; ++c) {
          initial[3*n+c] = packet.destination[c][n] = 113.*n-7.*c;
          packet.destination[c+3][n] = .73*n+.031*c;
        }
      }
      const auto untouched = packet;
      const auto expected = NativeAssembly(native,initial,repeated);
      ASSERT_EQ(tl::fea::AccumulateRepeatedNodalForces<4>(packet.nodes,packet.force,packet.couple,AssemblyView(packet)),
                AssemblyStatus::Success);
      for (unsigned n = 0; n < 4; ++n) {
        for (unsigned c = 0; c < 3; ++c) {
          Near(packet.destination[c][n],expected[3*n+c]);
          EXPECT_DOUBLE_EQ(packet.destination[c+3][n],untouched.destination[c+3][n]);
        }
      }
    }
  }
}
} // namespace tied_patch_test
