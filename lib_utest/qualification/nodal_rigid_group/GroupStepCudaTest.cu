#include "GroupStepTestSupport.h"
#include <cuda_runtime.h>
#include <limits>

namespace rigid_step_test {
namespace {
struct Packet { Input input; Trial output; rigid::StepStatus status=rigid::StepStatus::InvalidInput; };
static_assert(sizeof(Packet)<4096,"One bounded four-member pure-math packet");
__global__ void Evaluate(Packet* packet) { packet->status=EvaluatePacket(packet->input,packet->output); }
class DevicePacket {
 public:
  ~DevicePacket() { if(pointer_) EXPECT_EQ(cudaFree(pointer_),cudaSuccess); }
  cudaError_t Allocate() { return cudaMalloc(reinterpret_cast<void**>(&pointer_),sizeof(Packet)); }
  cudaError_t Run(Packet& packet) {
    auto status=cudaMemcpy(pointer_,&packet,sizeof packet,cudaMemcpyHostToDevice);
    if(status!=cudaSuccess) return status;
    Evaluate<<<1,1>>>(pointer_); status=cudaGetLastError(); if(status!=cudaSuccess) return status;
    status=cudaDeviceSynchronize(); if(status!=cudaSuccess) return status;
    return cudaMemcpy(&packet,pointer_,sizeof packet,cudaMemcpyDeviceToHost);
  }
 private:
  Packet* pointer_=nullptr;
};
}
TEST(NodalRigidGroupStepCuda,CompleteFourMemberPacketMatchesHostAcrossDurationsAndSpinBranches) {
  DevicePacket device; ASSERT_EQ(device.Allocate(),cudaSuccess);
  for(const auto d:{rigid::StepDurations{0,1./256,1./128},rigid::StepDurations{1./256,3./512,1./128},
      rigid::StepDurations{1./128,1./128,1./128}}) {
    for(double spin:{0.,1e-6,1.}) {
      Packet packet; packet.input=Fixture(d); auto& b=packet.input.body;
      b.omega={spin*.7,-spin*1.2,spin*.4};
      Trial expected; ASSERT_EQ(EvaluatePacket(packet.input,expected),rigid::StepStatus::Success);
      ASSERT_EQ(device.Run(packet),cudaSuccess); ASSERT_EQ(packet.status,rigid::StepStatus::Success);
      Agreement(packet.output,expected);
    }
  }
  RecordProperty("owned_device_bytes",int(sizeof(Packet))); RecordProperty("kernel_threads",1);
}
TEST(NodalRigidGroupStepCuda,LateMemberFailurePreservesValuePacketAndCleanRetryMatchesExactly) {
  DevicePacket device; ASSERT_EQ(device.Allocate(),cudaSuccess);
  Packet packet; packet.input=Fixture(); ASSERT_EQ(device.Run(packet),cudaSuccess);
  ASSERT_EQ(packet.status,rigid::StepStatus::Success); const auto good=packet;
  packet.input.member[Count-1].inertia=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(device.Run(packet),cudaSuccess); EXPECT_EQ(packet.status,rigid::StepStatus::InvalidInput);
  EXPECT_EQ(Bytes(packet.output),Bytes(good.output));
  packet.input=good.input;
  ASSERT_EQ(device.Run(packet),cudaSuccess); ASSERT_EQ(packet.status,rigid::StepStatus::Success);
  Agreement(packet.output,good.output,0);
}
} // namespace rigid_step_test
