#include "Fixture.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
namespace type13_recurrence_test {
__global__ void Packet(t::Property property,t::Reference reference,t::NativeHistory history,
    t::NativeEndpointKinematics a,t::NativeEndpointKinematics b,double dt,t::Evaluation* output,t::Status* status) {
  const t::NativeEndpointKinematics nodes[2]={a,b};*status=t::Evaluate(property,reference,history,nodes,dt,*output);
}
TEST(Type13H1Cuda,SixModesRetainDeviceHistoryAndMatchHostIncludingFailure) {
  t::Evaluation* device=nullptr;t::Status* code=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(*device)),cudaSuccess);ASSERT_EQ(cudaMalloc(&code,sizeof(*code)),cudaSuccess);
  for(unsigned mode=0;mode<6;++mode) {
    Case c(false,.125);t::Evaluation host;
    ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,host),t::Status::Success);
    auto device_history=host.native_history;double previous=0;
    for(double x:{.02,.08,.12,.10,-.12,.25,.30,.31}) {
      Mode(c,mode,x,previous,.01);t::Evaluation actual;t::Status status;
      Packet<<<1,1>>>(c.property,c.reference,device_history,c.nodes[0],c.nodes[1],.01,device,code);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&status,code,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);ASSERT_EQ(status,t::Status::Success);
      ASSERT_EQ(t::Evaluate(c.property,c.reference,host.native_history,c.nodes,.01,host),t::Status::Success);
      const auto a=Values(actual),b=Values(host);
      for(unsigned i=0;i<a.size();++i)EXPECT_NEAR(a[i],b[i],2e-12*std::fmax(1e-10,std::fmax(std::fabs(a[i]),std::fabs(b[i]))));
      device_history=actual.native_history;previous=x;
    }
  }
  EXPECT_EQ(cudaFree(code),cudaSuccess);EXPECT_EQ(cudaFree(device),cudaSuccess);
}
TEST(Type13H1Cuda,LateArithmeticFailurePreservesDeviceOutputAndRetries) {
  Case c;t::Evaluation initial;ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,initial),t::Status::Success);
  Mode(c,2,.06,0,.01);auto bad=initial.native_history;bad.channels[5].accumulated_plastic_deformation=1.7976931348623157e308;
  t::Evaluation* device=nullptr;t::Status* code=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(*device)),cudaSuccess);ASSERT_EQ(cudaMalloc(&code,sizeof(*code)),cudaSuccess);
  Packet<<<1,1>>>(c.property,c.reference,initial.native_history,c.nodes[0],c.nodes[1],.01,device,code);
  t::Evaluation clean_device;
  t::Status clean_status;
  ASSERT_EQ(cudaMemcpy(&clean_device,device,sizeof(clean_device),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&clean_status,code,sizeof(clean_status),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(clean_status,t::Status::Success);
  ASSERT_EQ(cudaMemcpy(device,&initial,sizeof(initial),cudaMemcpyHostToDevice),cudaSuccess);
  Packet<<<1,1>>>(c.property,c.reference,bad,c.nodes[0],c.nodes[1],.01,device,code);
  t::Evaluation result;t::Status status;
  ASSERT_EQ(cudaMemcpy(&result,device,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&status,code,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(status,t::Status::NonfiniteResult);EXPECT_EQ(Values(result),Values(initial));
  Packet<<<1,1>>>(c.property,c.reference,initial.native_history,c.nodes[0],c.nodes[1],.01,device,code);
  ASSERT_EQ(cudaMemcpy(&result,device,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&status,code,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);EXPECT_EQ(status,t::Status::Success);
  EXPECT_EQ(Values(result),Values(clean_device));
  EXPECT_EQ(cudaFree(code),cudaSuccess);EXPECT_EQ(cudaFree(device),cudaSuccess);
}
} // namespace type13_recurrence_test
