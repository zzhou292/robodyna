#include "RepeatedAssemblyFixture.h"
#include <cuda_runtime.h>

namespace tied_patch_test {
namespace {
__global__ void AssembleRepeated(AssemblyPacket* packet) {
  auto& p = *packet;
  p.status = tl::fea::AccumulateRepeatedNodalForces<4>(p.nodes,p.force,p.couple,AssemblyView(p),p.sign);
}
struct AssemblyDevice {
  AssemblyPacket* pointer = nullptr;
  AssemblyDevice() = default;
  AssemblyDevice(const AssemblyDevice&) = delete;
  AssemblyDevice& operator=(const AssemblyDevice&) = delete;
  ~AssemblyDevice() { if (pointer) cudaFree(pointer); }
};
void ExecuteAssembly(AssemblyDevice& device, AssemblyPacket& packet) {
  ASSERT_EQ(cudaMemcpy(device.pointer,&packet,sizeof(packet),cudaMemcpyHostToDevice),cudaSuccess);
  AssembleRepeated<<<1,1>>>(device.pointer);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&packet,device.pointer,sizeof(packet),cudaMemcpyDeviceToHost),cudaSuccess);
}
} // namespace
TEST(TiedPatchAssemblyCuda, NativeSlotOrderAndLateOverflowRollbackRetryOnDevice) {
  int count = 0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  AssemblyDevice device;
  ASSERT_EQ(cudaMalloc(&device.pointer,sizeof(AssemblyPacket)),cudaSuccess);
  auto packet = OrderedPacket();
  ASSERT_NO_FATAL_FAILURE(ExecuteAssembly(device,packet));
  CheckOrdered(packet);
  for (bool overflow : {false,true}) {
    packet = OrderedPacket();
    if (overflow) {
      packet.destination[5][2] = std::numeric_limits<double>::max();
      packet.couple[2].z = 0;
      packet.couple[3].z = std::numeric_limits<double>::max();
    } else packet.nodes[3] = 4;
    const auto before = Bytes(packet.destination);
    ASSERT_NO_FATAL_FAILURE(ExecuteAssembly(device,packet));
    EXPECT_EQ(packet.status,overflow ? AssemblyStatus::NonfiniteResult : AssemblyStatus::InvalidConnectivity);
    EXPECT_EQ(Bytes(packet.destination),before);
  }
  packet = OrderedPacket();
  ASSERT_NO_FATAL_FAILURE(ExecuteAssembly(device,packet));
  CheckOrdered(packet);
}
} // namespace tied_patch_test
