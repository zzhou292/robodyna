// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cuda_runtime.h>
#ifdef BEAM18_FORCE_ORIGINAL
#include "OriginalFixture.h"
#endif
namespace beam18_force_test {
struct Packet { b::Reference ref; b::Material material; b::ForceTrial accepted,trial; b::PrescribedInterval interval; b::Status status; };
__global__ void Evaluate(Packet* p,bool initial) {
  p->status=initial ? b::InitializeForce(p->ref,p->material,{11.123,-.37,.129},p->trial) :
      b::EvaluateForce(p->ref,p->material,p->accepted.proposed_history,p->interval,p->trial);
}
class DevicePacket {
 public:
  Packet* device=nullptr;double* curve=nullptr;
  DevicePacket() {cudaMalloc(&device,sizeof(Packet));cudaMalloc(&curve,92*sizeof(double));}
  ~DevicePacket(){cudaFree(curve);cudaFree(device);}
  void Prepare(Packet& p) {
    const auto& c=p.material.curve;
    ASSERT_EQ(c.count,46u);
    ASSERT_EQ(cudaMemcpy(curve,c.plastic_strain,46*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(curve+46,c.yield_stress_pa,46*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
    p.material.curve={curve,curve+46,46};
  }
  bool Run(Packet& p,bool initial) {
    if(cudaMemcpy(device,&p,sizeof(p),cudaMemcpyHostToDevice)!=cudaSuccess)return false;
    Evaluate<<<1,1>>>(device,initial);
    return cudaGetLastError()==cudaSuccess&&cudaDeviceSynchronize()==cudaSuccess&&
      cudaMemcpy(&p,device,sizeof(p),cudaMemcpyDeviceToHost)==cudaSuccess;
  }
};
class Beam18ForceCuda:public ::testing::Test {void SetUp() override {int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);}};
TEST_F(Beam18ForceCuda, CarriedHistoryAndLateRejectedScratchRetryMatchIndependentNative) {
  Packet packet{};packet.ref=Reference();const auto material=Material(packet.ref);packet.material=material;
  DevicePacket gpu;gpu.Prepare(packet);ASSERT_TRUE(gpu.Run(packet,true));ASSERT_EQ(packet.status,b::Status::Success);
  b::PrescribedInterval virgin;for(auto& v:virgin.velocity_midpoint_m_s)v={11.123,-.37,.129};
  auto native=Native(packet.ref,material,{},virgin,true);Compare(packet.trial,native);
  for(unsigned step=0;step<16;++step) {
    packet.accepted=packet.trial;packet.interval=Motion(packet.ref,packet.accepted.proposed_history);
    packet.interval.velocity_midpoint_m_s[1]={step<9?1000.:-300.,50.,-25.};
    packet.interval.angular_velocity_midpoint_rad_s[1]={2,-3,5};
    const auto valid=packet.interval;
    const auto saved=packet.trial;
    packet.interval.angular_velocity_midpoint_rad_s[1].z=std::numeric_limits<double>::max();
    ASSERT_TRUE(gpu.Run(packet,false));EXPECT_NE(packet.status,b::Status::Success);
    EXPECT_EQ(std::memcmp(&saved,&packet.trial,sizeof(saved)),0);
    packet.interval=valid;ASSERT_TRUE(gpu.Run(packet,false));ASSERT_EQ(packet.status,b::Status::Success);
    native=Native(packet.ref,material,native.next,valid);Compare(packet.trial,native);ASSERT_FALSE(HasFailure());
  }
}
#ifdef BEAM18_FORCE_ORIGINAL
TEST_F(Beam18ForceCuda, All142OriginalSourceNativeConstructorsAndFirstIntervals) {
  DevicePacket gpu;
  for(unsigned i=0;i<142;++i) {
    Packet p{};const auto input=beam18_test::original::Input(i);SCOPED_TRACE(input.source_element_id);
    ASSERT_EQ(b::InitializeReference(input,p.ref),b::Status::Success);
    const auto material=Material(p.ref);p.material=material;gpu.Prepare(p);
    ASSERT_TRUE(gpu.Run(p,true));ASSERT_EQ(p.status,b::Status::Success);
    b::PrescribedInterval initial;for(auto& v:initial.velocity_midpoint_m_s)v={11.123,-.37,.129};
    auto native=Native(p.ref,material,{},initial,true);Compare(p.trial,native);ASSERT_FALSE(HasFailure());
    p.accepted=p.trial;p.interval=Motion(p.ref,p.accepted.proposed_history);
    p.interval.velocity_midpoint_m_s[1]={4.1,-3.2,2.3};p.interval.angular_velocity_midpoint_rad_s[1]={1.2,-2.3,3.4};
    ASSERT_TRUE(gpu.Run(p,false));ASSERT_EQ(p.status,b::Status::Success);
    Compare(p.trial,Native(p.ref,material,native.next,p.interval));ASSERT_FALSE(HasFailure());
  }
}
#endif
} // namespace beam18_force_test
