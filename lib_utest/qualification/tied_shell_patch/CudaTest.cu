#include "NativeOracle.h"
#include <cuda_runtime.h>

namespace tied_patch_test {
struct Packet {
  tie::PatchInput input;
  tie::SecondaryLoad load;
  tie::MasterMotion master;
  tie::Patch patch;
  tie::MasterLoads force;
  tie::SecondaryMotion motion;
  tie::Status status = tie::Status::InvalidInput;
};
__global__ void Evaluate(Packet* packet) {
  auto& p = *packet;
  tie::Patch patch;
  tie::MasterLoads force;
  tie::SecondaryMotion motion;
  p.status = tie::PreparePatch(p.input,patch);
  if (p.status != tie::Status::Success) return;
  p.status = tie::TransferLoad(patch,p.load,force);
  if (p.status != tie::Status::Success) return;
  p.status = tie::RecoverMotion(patch,p.master,motion);
  if (p.status != tie::Status::Success) return;
  p.patch = patch; p.force = force; p.motion = motion;
}
struct Device {
  Packet* p = nullptr;
  ~Device() { if (p) cudaFree(p); }
  Device() = default;
  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;
};
void EvaluateDevice(Device& device, Packet& packet) {
  ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof(packet),cudaMemcpyHostToDevice),cudaSuccess);
  Evaluate<<<1,1>>>(device.p);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof(packet),cudaMemcpyDeviceToHost),cudaSuccess);
}
TEST(TiedPatchCuda,DeviceGeometryForceAndMotionAgreeWithIndependentNativePackets) {
  int count = 0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  Device device;
  ASSERT_EQ(cudaMalloc(&device.p,sizeof(Packet)),cudaSuccess);
  for (unsigned shape = 0; shape < 4; ++shape) {
    for (unsigned step = 0; step < 16; ++step) {
      Packet packet;
      packet.input=Geometry(shape); packet.load=Load(step); packet.master=Motion(step,shape == 1);
      ASSERT_NO_FATAL_FAILURE(EvaluateDevice(device,packet));
      ASSERT_EQ(packet.status,tie::Status::Success);
      Agreement(packet.patch,packet.force,packet.motion,Native(packet.input,packet.load,packet.master,shape == 1));
    }
  }
}
TEST(TiedPatchCuda,LateMotionRejectionPreservesPublishedValuePacketAndRetry) {
  Device device;
  ASSERT_EQ(cudaMalloc(&device.p,sizeof(Packet)),cudaSuccess);
  Packet packet;
  packet.input=Geometry(3); packet.load=Load(3); packet.master=Motion(3);
  ASSERT_NO_FATAL_FAILURE(EvaluateDevice(device,packet));
  ASSERT_EQ(packet.status,tie::Status::Success);
  const auto geometry=Bytes(packet.patch);
  const auto geometry_values=PatchBits(packet.patch);
  const auto force=Bytes(packet.force);
  const auto motion=Bytes(packet.motion);
  packet.master.acceleration[3].z=std::numeric_limits<double>::quiet_NaN();
  ASSERT_NO_FATAL_FAILURE(EvaluateDevice(device,packet));
  ASSERT_EQ(packet.status,tie::Status::InvalidInput);
  EXPECT_EQ(Bytes(packet.patch),geometry);
  EXPECT_EQ(Bytes(packet.force),force);
  EXPECT_EQ(Bytes(packet.motion),motion);
  packet.master=Motion(3);
  ASSERT_NO_FATAL_FAILURE(EvaluateDevice(device,packet));
  ASSERT_EQ(packet.status,tie::Status::Success);
  EXPECT_TRUE(packet.patch.prepared());
  EXPECT_EQ(PatchBits(packet.patch),geometry_values);
  EXPECT_EQ(Bytes(packet.force),force);
  EXPECT_EQ(Bytes(packet.motion),motion);
}
} // namespace tied_patch_test
