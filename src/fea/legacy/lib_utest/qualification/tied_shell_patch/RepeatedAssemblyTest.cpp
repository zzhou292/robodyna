#include "RepeatedAssemblyFixture.h"

namespace tied_patch_test {
TEST(TiedPatchAssembly, RepeatedSlotRetainsNativeOrderAndLegacyDistinctContract) {
  auto packet = OrderedPacket();
  const auto saved = Bytes(packet.destination);
  EXPECT_EQ(tl::fea::AccumulateNodalForces<4>(packet.nodes,packet.force,packet.couple,AssemblyView(packet)),
            AssemblyStatus::InvalidConnectivity);
  EXPECT_EQ(Bytes(packet.destination),saved);
  packet.status = tl::fea::AccumulateRepeatedNodalForces<4>(packet.nodes,packet.force,packet.couple,AssemblyView(packet));
  CheckOrdered(packet);
  const double precombined = -1e16 + 1;
  EXPECT_NE(1e16+precombined,packet.destination[0][2]);
}
TEST(TiedPatchAssembly, EveryDistinctDestinationMatchesLegacyForBothSigns) {
  for (int sign : {-1,1}) {
    AssemblyPacket actual;
    actual.nodes[3] = 3;
    for (unsigned n = 0; n < 4; ++n) {
      actual.force[n] = {.5*n,-2.*n,13.+n};
      actual.couple[n] = {.17*n,3.-n,-.031*n};
      for (unsigned c = 0; c < 6; ++c) actual.destination[c][n] = .3*c-n;
    }
    auto expected = actual;
    ASSERT_EQ(tl::fea::AccumulateNodalForces<4>(expected.nodes,expected.force,expected.couple,AssemblyView(expected),sign),
              AssemblyStatus::Success);
    ASSERT_EQ(tl::fea::AccumulateRepeatedNodalForces<4>(actual.nodes,actual.force,actual.couple,AssemblyView(actual),sign),
              AssemblyStatus::Success);
    EXPECT_EQ(Bytes(actual.destination),Bytes(expected.destination));
  }
}
TEST(TiedPatchAssembly, FinalConnectivityNonfiniteAndOverflowRejectBeforeAnyWrite) {
  const auto original = OrderedPacket();
  for (unsigned fault = 0; fault < 6; ++fault) {
    auto packet = original;
    auto view = AssemblyView(packet);
    switch (fault) {
      case 0: packet.nodes[3] = 4; break;
      case 1: packet.couple[3].z = std::numeric_limits<double>::quiet_NaN(); break;
      case 2:
        packet.destination[5][2] = std::numeric_limits<double>::max();
        packet.couple[2].z = 0;
        packet.couple[3].z = std::numeric_limits<double>::max();
        break;
      case 3: view.couple_z = view.force_z; break;
      case 4: view.couple_z = nullptr; break;
      case 5: packet.sign = 0; break;
    }
    const auto before = Bytes(packet.destination);
    EXPECT_NE(tl::fea::AccumulateRepeatedNodalForces<4>(packet.nodes,packet.force,packet.couple,view,packet.sign),
              AssemblyStatus::Success);
    EXPECT_EQ(Bytes(packet.destination),before);
  }
  auto retry = original;
  retry.status = tl::fea::AccumulateRepeatedNodalForces<4>(retry.nodes,retry.force,retry.couple,AssemblyView(retry));
  CheckOrdered(retry);
}
} // namespace tied_patch_test
