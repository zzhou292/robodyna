#include "Fixture.h"
#include <cuda_runtime.h>

namespace rigid_contact_test {
struct Packet { sc::RigidContactBody body;Vec3 point,normal;sc::RigidNormalResponse response;sc::Status status; };
__global__ void Evaluate(Packet* packets) {
  const auto i=threadIdx.x;
  if(i<4) {
    auto& p=packets[i];p.status=sc::EvaluateRigidNormalResponse(p.body,p.point,p.normal,p.response);
  }
}
TEST(RigidContactResponseCuda, SameBodyResponseAndRejectedTailPreserveOutput) {
  Packet input[4];
  for(auto& p:input)p={Body(),{.4,2,-3},{.6,.8,0},{17,18,true},sc::Status::kInvalidArgument};
  input[1].point=input[1].body.center;
  input[2].body.current_frame.inertia.z=0;
  input[3].normal={0,0,0};
  Packet* device=nullptr;ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device),sizeof(input)),cudaSuccess);
  struct Cleanup { Packet* p;~Cleanup(){cudaFree(p);} } cleanup{device};
  ASSERT_EQ(cudaMemcpy(device,input,sizeof(input),cudaMemcpyHostToDevice),cudaSuccess);
  Evaluate<<<1,4>>>(device);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Packet output[4];ASSERT_EQ(cudaMemcpy(output,device,sizeof(output),cudaMemcpyDeviceToHost),cudaSuccess);
  for(unsigned i=0;i<4;++i) {
    auto expected=input[i].response;
    const auto status=sc::EvaluateRigidNormalResponse(input[i].body,input[i].point,input[i].normal,expected);
    EXPECT_EQ(output[i].status,status);
    EXPECT_EQ(std::memcmp(&output[i].response.inverse_effective_mass,&expected.inverse_effective_mass,sizeof(double)),0);
    EXPECT_EQ(std::memcmp(&output[i].response.inverse_upper,&expected.inverse_upper,sizeof(double)),0);
    EXPECT_EQ(output[i].response.valid,expected.valid);
  }
}
} // namespace rigid_contact_test
