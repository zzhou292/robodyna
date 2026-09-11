#include "NativeOracle.h"
#include <cuda_runtime.h>

namespace tied_patch_test {
struct Packet {
  tie::PatchInput input;
  tie::SecondaryLoad load;
  tie::MasterMotion master;
  tie::CoefficientInput coefficients;
  tie::Patch patch;
  tie::MasterLoads force;
  tie::SecondaryMotion motion;
  tie::CoefficientTransfer transfer;
  tie::Status status = tie::Status::InvalidInput;
};
__global__ void Evaluate(Packet* packet) {
  auto& p = *packet;
  tie::Patch patch;
  tie::MasterLoads force;
  tie::SecondaryMotion motion;
  tie::CoefficientTransfer transfer;
  p.status = tie::PreparePatch(p.input,patch);
  if (p.status != tie::Status::Success) return;
  p.status = tie::TransferLoad(patch,p.load,force);
  if (p.status != tie::Status::Success) return;
  p.status = tie::RecoverMotion(patch,p.master,motion);
  if (p.status != tie::Status::Success) return;
  p.status = tie::TransferCoefficients(patch,p.coefficients,transfer);
  if (p.status != tie::Status::Success) return;
  p.patch = patch; p.force = force; p.motion = motion; p.transfer = transfer;
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
TEST(TiedPatchCuda, CoefficientTransfersMatchNativeIncludingDependentAndRepeatedNodes) {
  Device device;
  ASSERT_EQ(cudaMalloc(&device.p,sizeof(Packet)),cudaSuccess);
  for (unsigned shape = 0; shape < 4; ++shape) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      SCOPED_TRACE(shape);
      SCOPED_TRACE(mode);
      Packet packet;
      packet.input = Geometry(shape); packet.load = Load(3); packet.master = Motion(3,shape == 1);
      packet.coefficients = Coefficients(mode);
      if (shape == 1) packet.coefficients.initial_master_inertia[3] = packet.coefficients.initial_master_inertia[2];
      ASSERT_NO_FATAL_FAILURE(EvaluateDevice(device,packet));
      ASSERT_EQ(packet.status,tie::Status::Success);
      const auto native = NativeCoefficients(packet.input,packet.coefficients,shape == 1);
      const auto actual = CoefficientValues(packet.transfer,shape == 1);
      for (unsigned i = 0; i < actual.size(); ++i) Near(actual[i],native[i]);
      EXPECT_DOUBLE_EQ(actual[18],native[18]); EXPECT_DOUBLE_EQ(actual[19],native[19]);
    }
  }
}
TEST(TiedPatchCuda, LateCoefficientInvalidityAndOverflowKeepPriorCompletePacketAndRetry) {
  Device device;
  ASSERT_EQ(cudaMalloc(&device.p,sizeof(Packet)),cudaSuccess);
  Packet packet;
  packet.input = Geometry(0); packet.load = Load(3); packet.master = Motion(3);
  packet.coefficients = Coefficients(4);
  ASSERT_NO_FATAL_FAILURE(EvaluateDevice(device,packet));
  ASSERT_EQ(packet.status,tie::Status::Success);
  const auto saved_transfer = Bytes(packet.transfer);
  const auto saved_force = Bytes(packet.force);
  const auto saved_motion = Bytes(packet.motion);
  const auto saved_patch = Bytes(packet.patch);
  const auto transfer_values = CoefficientValues(packet.transfer,false);
  for (bool overflow : {false,true}) {
    packet.coefficients = Coefficients(4);
    if (overflow) packet.coefficients.secondary.rotational_stiffness = std::numeric_limits<double>::max();
    else packet.coefficients.initial_master_inertia[3] = std::numeric_limits<double>::quiet_NaN();
    ASSERT_NO_FATAL_FAILURE(EvaluateDevice(device,packet));
    ASSERT_EQ(packet.status,overflow ? tie::Status::NonfiniteResult : tie::Status::InvalidInput);
    EXPECT_EQ(Bytes(packet.transfer),saved_transfer);
    EXPECT_EQ(Bytes(packet.force),saved_force);
    EXPECT_EQ(Bytes(packet.motion),saved_motion);
    EXPECT_EQ(Bytes(packet.patch),saved_patch);
  }
  packet.coefficients = Coefficients(4);
  ASSERT_NO_FATAL_FAILURE(EvaluateDevice(device,packet));
  ASSERT_EQ(packet.status,tie::Status::Success);
  EXPECT_EQ(CoefficientValues(packet.transfer,false),transfer_values);
}
} // namespace tied_patch_test
