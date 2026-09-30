#include "GroupTestSupport.h"
#include <cuda_runtime.h>
#include <limits>

namespace rigid_test {
namespace {
struct Packet {
  Vec3 positions[3]{{1,2,3},{-.5,.4,.9},{.3,-.7,.2}};
  Vec3 forces[3]{{2,3,5},{7,-4,2},{-1,9,3}},couples[3]{{.1,.2,.3},{-.4,.8,.2},{.3,-.1,.6}};
  Vec3 center{.25,.1,-.5},omega{.7,-1.2,.4},torque{2,-3,1},raw{.0001,2,1000};
  rigid::PrincipalFrame frame{};
  rigid::Wrench wrench{{11,12,13},{14,15,16}};
  Vec3 acceleration{17,18,19};
  rigid::PrincipalCorrection correction{};
  rigid::MathStatus wrench_status=rigid::MathStatus::InvalidInput;
  rigid::MathStatus acceleration_status=rigid::MathStatus::InvalidInput;
  rigid::MathStatus correction_status=rigid::MathStatus::InvalidInput;
};
static_assert(sizeof(Packet)<2048,"One small pure-math CUDA packet");
__global__ void Evaluate(Packet* p) {
  p->wrench_status=rigid::AggregateWrench(p->center,p->positions,p->forces,p->couples,3,p->wrench);
  p->acceleration_status=rigid::AngularAcceleration(p->frame,p->omega,p->torque,p->acceleration);
  p->correction_status=rigid::CorrectPrincipalInertia(p->raw,p->correction);
}
class DevicePacket {
 public:
  ~DevicePacket() { if(pointer_) EXPECT_EQ(cudaFree(pointer_),cudaSuccess); }
  cudaError_t Allocate() { return cudaMalloc(reinterpret_cast<void**>(&pointer_),sizeof(Packet)); }
  cudaError_t Run(Packet& p) {
    auto status=cudaMemcpy(pointer_,&p,sizeof p,cudaMemcpyHostToDevice);
    if(status!=cudaSuccess) return status;
    Evaluate<<<1,1>>>(pointer_); status=cudaGetLastError(); if(status!=cudaSuccess) return status;
    status=cudaDeviceSynchronize(); if(status!=cudaSuccess) return status;
    return cudaMemcpy(&p,pointer_,sizeof p,cudaMemcpyDeviceToHost);
  }
 private:
  Packet* pointer_=nullptr;
};
TEST(NodalRigidGroupCuda,PureMathAgreesWithHostForWrenchGyroAndSourceCorrection) {
  DevicePacket device; ASSERT_EQ(device.Allocate(),cudaSuccess);
  Packet p; p.frame=Frame();
  rigid::Wrench expected; Vec3 acceleration; rigid::PrincipalCorrection correction;
  ASSERT_EQ(rigid::AggregateWrench(p.center,p.positions,p.forces,p.couples,3,expected),rigid::MathStatus::Success);
  ASSERT_EQ(rigid::AngularAcceleration(p.frame,p.omega,p.torque,acceleration),rigid::MathStatus::Success);
  ASSERT_EQ(rigid::CorrectPrincipalInertia(p.raw,correction),rigid::MathStatus::Success);
  ASSERT_EQ(device.Run(p),cudaSuccess);
  EXPECT_EQ(p.wrench_status,rigid::MathStatus::Success); EXPECT_EQ(p.acceleration_status,rigid::MathStatus::Success);
  EXPECT_EQ(p.correction_status,rigid::MathStatus::Success);
  Near(p.wrench.force,expected.force); Near(p.wrench.couple,expected.couple);
  Near(p.acceleration,acceleration); Near(p.correction.effective,correction.effective);
  EXPECT_EQ(p.correction.changed,correction.changed);
  RecordProperty("owned_device_bytes",int(sizeof(Packet))); RecordProperty("kernel_threads",1);
}
TEST(NodalRigidGroupCuda,LateInvalidInputPreservesPriorOutputsAndAllowsExactRetry) {
  DevicePacket device; ASSERT_EQ(device.Allocate(),cudaSuccess);
  Packet p; p.frame=Frame(); ASSERT_EQ(device.Run(p),cudaSuccess); const auto accepted=p;
  p.forces[2].z=std::numeric_limits<double>::quiet_NaN(); p.frame.axes.v[0]=1; p.raw.x=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(device.Run(p),cudaSuccess);
  EXPECT_EQ(p.wrench_status,rigid::MathStatus::InvalidInput); EXPECT_EQ(p.acceleration_status,rigid::MathStatus::InvalidInput);
  EXPECT_EQ(p.correction_status,rigid::MathStatus::InvalidInput);
  Near(p.wrench.force,accepted.wrench.force,0); Near(p.wrench.couple,accepted.wrench.couple,0);
  Near(p.acceleration,accepted.acceleration,0); Near(p.correction.effective,accepted.correction.effective,0);
  p.forces[2]=accepted.forces[2]; p.frame=accepted.frame; p.raw=accepted.raw;
  ASSERT_EQ(device.Run(p),cudaSuccess); EXPECT_EQ(p.wrench_status,rigid::MathStatus::Success);
  Near(p.wrench.force,accepted.wrench.force,0); Near(p.wrench.couple,accepted.wrench.couple,0);
  Near(p.acceleration,accepted.acceleration,0); Near(p.correction.effective,accepted.correction.effective,0);
}
} // namespace
} // namespace rigid_test
