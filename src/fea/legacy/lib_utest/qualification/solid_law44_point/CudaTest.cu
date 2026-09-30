#include "NativeOracle.h"
#include <cuda_runtime.h>
#include <limits>

namespace law44_solid_test {
namespace {
struct DeviceScope {
  double *x=nullptr,*y=nullptr;
  law::Parameters* parameters=nullptr;
  law::Result* results=nullptr;
  law::Status* status=nullptr;
  ~DeviceScope() {
    cudaFree(status); cudaFree(results); cudaFree(parameters); cudaFree(y); cudaFree(x);
  }
  bool Initialize(const law::Parameters (&p)[4]) {
    if(cudaMalloc(&x,sizeof(X))!=cudaSuccess || cudaMalloc(&y,sizeof(Y))!=cudaSuccess ||
       cudaMalloc(&parameters,sizeof(p))!=cudaSuccess ||
       cudaMalloc(&results,8*sizeof(law::Result))!=cudaSuccess ||
       cudaMalloc(&status,8*sizeof(law::Status))!=cudaSuccess) return false;
    auto prepared=std::array<law::Parameters,4>{p[0],p[1],p[2],p[3]};
    for(auto& v:prepared) v.curve={x,y,Count};
    return cudaMemcpy(x,X,sizeof(X),cudaMemcpyHostToDevice)==cudaSuccess &&
           cudaMemcpy(y,Y,sizeof(Y),cudaMemcpyHostToDevice)==cudaSuccess &&
           cudaMemcpy(parameters,prepared.data(),sizeof(p),cudaMemcpyHostToDevice)==cudaSuccess;
  }
};
__global__ void Advance(const law::Parameters* p,law::Result* results,law::Status* status) {
  const unsigned i=threadIdx.x;
  if(i>=4) return;
  law::History history{};
  law::Result result{};
  status[i]=law::Status::Ok;
  for(unsigned step=0;step<320;++step) {
    status[i]=law::Update(p[i],history,Motion(step),result);
    if(status[i]!=law::Status::Ok) return;
    history=result.history;
  }
  results[i]=result;
}
__global__ void RejectAndRetry(const law::Parameters* p,law::Result* results,law::Status* status,
                              double invalid) {
  const unsigned i=threadIdx.x;
  if(i>=4) return;
  const auto history=results[i].history;
  auto input=Motion(320);
  input.engineering_rate_per_s[5]=invalid;
  status[i]=law::Update(p[i],history,input,results[i]);
  status[i+4]=law::Update(p[i],history,Motion(320),results[i+4]);
}
void Materials(law::Parameters (&p)[4]) {
  p[0]=Parameters(); p[1]=Parameters(true);
  p[2]=Parameters(false,law::WorkingUnits::SI); p[3]=Parameters(true,law::WorkingUnits::SI);
}
}
TEST(SolidLaw44Cuda, IndependentNativeAndDeviceAcceptedHistories) {
  law::Parameters p[4]; Materials(p);
  DeviceScope device;
  ASSERT_TRUE(device.Initialize(p));
  Advance<<<1,4>>>(device.parameters,device.results,device.status);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  law::Result result[4]{};
  law::Status status[4]{};
  ASSERT_EQ(cudaMemcpy(result,device.results,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(status,device.status,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);
  for(unsigned i=0;i<4;++i) {
    ASSERT_EQ(status[i],law::Status::Ok);
    law::History native{},previous{};
    NativeResult n{};
    for(unsigned step=0;step<320;++step) {
      previous=native;
      n=Native(p[i],native,Motion(step));
      native=n.result.history;
    }
    Compare(result[i],n.result,p[i],previous,Motion(319));
  }
}
TEST(SolidLaw44Cuda, LatePointFailureKeepsOutputAndRetryUsesAcceptedHistory) {
  law::Parameters p[4]; Materials(p);
  DeviceScope device;
  ASSERT_TRUE(device.Initialize(p));
  Advance<<<1,4>>>(device.parameters,device.results,device.status);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  law::Result before[4]{},after[8]{};
  law::Status status[8]{};
  ASSERT_EQ(cudaMemcpy(before,device.results,sizeof(before),cudaMemcpyDeviceToHost),cudaSuccess);
  RejectAndRetry<<<1,4>>>(device.parameters,device.results,device.status,
                        std::numeric_limits<double>::infinity());
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(after,device.results,sizeof(after),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(status,device.status,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);
  for(unsigned i=0;i<4;++i) {
    EXPECT_EQ(status[i],law::Status::InvalidInput);
    ASSERT_EQ(status[i+4],law::Status::Ok);
    Same(before[i],after[i]);
    Compare(after[i+4],Native(p[i],before[i].history,Motion(320)).result,
            p[i],before[i].history,Motion(320));
  }
}
}  // namespace law44_solid_test
