// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "EvaluationValues.h"
#include "lib_src/elements/type25/Type25Math.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <cstring>
#include <limits>

namespace type25_test {
namespace {
struct Packet {
  spring::SourceUnits units{1000,.001,1};spring::Property property{};spring::Reference reference{};
  spring::History accepted{};spring::EndpointKinematics nodes[2]{};double dt=1e-7;
  spring::Evaluation output{};spring::Status status=spring::Status::InvalidInput;
};
static_assert(sizeof(Packet)<4096);
__global__ void RunValue(Packet* p) {
  p->status=spring::Evaluate(p->units,p->property,p->reference,p->accepted,p->nodes,p->dt,p->output);
}
struct DevicePacket {
  Packet* pointer=nullptr;
  ~DevicePacket(){if(pointer)EXPECT_EQ(cudaFree(pointer),cudaSuccess);}
  cudaError_t Allocate(){return cudaMalloc(reinterpret_cast<void**>(&pointer),sizeof(Packet));}
  cudaError_t Run(Packet& p) {
    auto error=cudaMemcpy(pointer,&p,sizeof(p),cudaMemcpyHostToDevice);if(error!=cudaSuccess)return error;
    RunValue<<<1,1>>>(pointer);error=cudaGetLastError();if(error!=cudaSuccess)return error;
    error=cudaDeviceSynchronize();if(error!=cudaSuccess)return error;
    return cudaMemcpy(&p,pointer,sizeof(p),cudaMemcpyDeviceToHost);
  }
};
Packet Input() {
  Fixture f;Packet p;p.property=Property();const auto& c=f.connections[2];
  EXPECT_EQ(spring::InitializeReference(p.units,c.position,c.seed,p.reference),spring::Status::Success);
  p.accepted.transverse_axis=p.reference.transverse_axis;
  p.nodes[0]={c.position[0],{1,2,3},{.4,.5,.6}};
  p.nodes[1]={c.position[1],{2,3,4},{.7,.8,.9}};
  return p;
}
void Compare(const spring::Evaluation& a,const spring::Evaluation& b) {
  const auto av=EvaluationValues(a),bv=EvaluationValues(b);
  for(std::size_t i=0;i<av.size();++i)EXPECT_NEAR(av[i],bv[i],2e-12*std::max({std::abs(av[i]),std::abs(bv[i]),1e-30}))<<i;
  EXPECT_EQ(a.history.active,b.history.active);
}
TEST(Type25Cuda,PureMathMatchesAllHostFieldsAcrossAcceptedHistory) {
  DevicePacket device;ASSERT_EQ(device.Allocate(),cudaSuccess);auto p=Input();auto host=p;
  for(unsigned step=0;step<16;++step) {
    for(unsigned n=0;n<2;++n) {
      p.nodes[n].position.x+=p.dt*p.nodes[n].velocity.x;
      p.nodes[n].position.y+=p.dt*p.nodes[n].velocity.y;
      p.nodes[n].position.z+=p.dt*p.nodes[n].velocity.z;
      host.nodes[n]=p.nodes[n];
    }
    ASSERT_EQ(spring::Evaluate(host.units,host.property,host.reference,host.accepted,host.nodes,host.dt,host.output),spring::Status::Success);
    ASSERT_EQ(device.Run(p),cudaSuccess);ASSERT_EQ(p.status,spring::Status::Success);Compare(p.output,host.output);
    p.accepted=p.output.history;host.accepted=host.output.history;
  }
  RecordProperty("device_bytes",static_cast<int>(sizeof(Packet)));RecordProperty("kernel_threads",1);
}
TEST(Type25Cuda,LateInvalidEndpointPreservesOutputAndRetry) {
  DevicePacket device;ASSERT_EQ(device.Allocate(),cudaSuccess);auto p=Input();
  ASSERT_EQ(device.Run(p),cudaSuccess);ASSERT_EQ(p.status,spring::Status::Success);
  const auto before=p; p.nodes[1].angular_velocity.z=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(device.Run(p),cudaSuccess);EXPECT_EQ(p.status,spring::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(&p.output,&before.output,sizeof(p.output)),0);
  p.nodes[1]=before.nodes[1];ASSERT_EQ(device.Run(p),cudaSuccess);ASSERT_EQ(p.status,spring::Status::Success);
  Compare(p.output,before.output);
}
} // namespace
} // namespace type25_test
