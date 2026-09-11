#include "ZeroCNativeSupport.h"
#include <cuda_runtime.h>
#include <limits>

namespace zero_c_test {
namespace {
struct Packet {
  Parameters parameters;
  double x[3]{},y[3]{};
  History history;
  Result result;
  Status status=Status::InvalidParameters;
};
struct Device { Packet* p=nullptr; ~Device(){if(p) cudaFree(p);} };
__global__ void Configure(Packet* packet,bool analytic) {
  auto& p=*packet;
  const mat::TabulatedShellPlasticityRate rate{true,0,1,10000,Policy::FilteredZeroC};
  p.status=analytic?mat::PrepareLinearLaw44ShellPlasticity(70e9,.22,2500,{30e6,1e9},rate,p.parameters):
      mat::PrepareTabulatedShellPlasticity(70e9,.22,2500,{p.x,p.y,3},rate,p.parameters);
}
__global__ void AdvancePoint(Packet* packet,Input in) {
  auto& p=*packet;
  p.status=mat::UpdateLaw44ShellPlasticity(p.parameters,p.history,in,p.result);
  if(p.status==Status::Ok) p.history=p.result.history;
}
}
TEST(FilteredZeroCCuda, DeviceOwnedFilterAndMaterialHistoryMatchNativeIncludingOffAndLateRetry) {
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  Device device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.p),sizeof(Packet)),cudaSuccess);
  for (bool analytic:{false,true}) {
    SCOPED_TRACE(analytic);
    Packet packet;
    std::copy_n(X,3,packet.x); std::copy_n(Y,3,packet.y);
    History native_history;
    native_history.filtered_rate_per_s=packet.history.filtered_rate_per_s=23.;
    ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
    Configure<<<1,1>>>(device.p,analytic);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.status,Status::Ok);
    double time=0;
    for (unsigned step=0;step<14;++step) {
      SCOPED_TRACE(step);
      const auto in=Increment(step);
      time+=in.dt;
      const auto expected=NativeAdvance(analytic,native_history,in,time);
      AdvancePoint<<<1,1>>>(device.p,in);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(packet.status,Status::Ok);
      Compare(packet.result,expected,native_history);
      native_history=NativeHistory(expected);
    }
    const auto before=Bytes(packet.history);
    const auto result=Bytes(packet.result);
    auto bad=Increment(12);
    bad.strain_increment[4]=std::numeric_limits<double>::max();
    AdvancePoint<<<1,1>>>(device.p,bad);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_EQ(packet.status,Status::NonfiniteResult);
    EXPECT_EQ(Bytes(packet.history),before);
    EXPECT_EQ(Bytes(packet.result),result);
    const auto retry=Increment(12);
    const auto expected=NativeAdvance(analytic,native_history,retry,time+retry.dt);
    AdvancePoint<<<1,1>>>(device.p,retry);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.status,Status::Ok);
    Compare(packet.result,expected,native_history);
  }
}
} // namespace zero_c_test
