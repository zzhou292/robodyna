#pragma once
#include "Fixture.h"
#include "lib_src/solvers/NodalRepeatedForceAssembly.h"

namespace tied_patch_test {
using AssemblyStatus = tl::fea::NodalForceAssemblyStatus;
struct AssemblyPacket {
  double destination[6][4]{};
  std::size_t nodes[4]{0,1,2,2};
  Vec3 force[4]{}, couple[4]{};
  AssemblyStatus status = AssemblyStatus::InvalidView;
  int sign = 1;
};
#if defined(__CUDACC__)
__host__ __device__
#endif
inline tl::fea::DeviceNodalForceView AssemblyView(AssemblyPacket& packet) {
  return {packet.destination[0],packet.destination[1],packet.destination[2],
          packet.destination[3],packet.destination[4],packet.destination[5],4,0};
}
inline AssemblyPacket OrderedPacket() {
  AssemblyPacket packet;
  for (unsigned c = 0; c < 6; ++c) {
    packet.destination[c][2] = 1e16;
    packet.destination[c][3] = 17+c; // Unreferenced physical node must survive.
  }
  packet.force[2] = packet.couple[2] = {-1e16,-1e16,-1e16};
  packet.force[3] = packet.couple[3] = {1,1,1};
  return packet;
}
inline void CheckOrdered(const AssemblyPacket& packet) {
  ASSERT_EQ(packet.status,AssemblyStatus::Success);
  for (unsigned c = 0; c < 6; ++c) {
    EXPECT_DOUBLE_EQ(packet.destination[c][2],1);
    EXPECT_DOUBLE_EQ(packet.destination[c][3],17+c);
  }
}
} // namespace tied_patch_test
