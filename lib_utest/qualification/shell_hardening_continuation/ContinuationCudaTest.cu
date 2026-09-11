#include "ContinuationNativeSupport.h"
#include <cuda_runtime.h>
#include <limits>

namespace continuation_test {
namespace {
struct Packet {
  Parameters parameters;
  double x[17]{},y[17]{};
  History history;
  Result result;
  Status status=Status::InvalidParameters;
};
struct Device { Packet* p=nullptr; ~Device(){if(p)cudaFree(p);} };
__global__ void Configure(Packet* packet,unsigned count) {
  auto& p=*packet;
  p.status=mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{p.x,p.y,count},
      p.parameters.rate,Policy::NativeLastSegment,p.parameters);
}
__global__ void Advance(Packet* packet,Input input) {
  auto& p=*packet;
  p.status=mat::UpdateLaw44ShellPlasticity(p.parameters,p.history,input,p.result);
  if(p.status==Status::Ok) p.history=p.result.history;
}
}
TEST(HardeningContinuationCuda, DevicePreparedOriginalBoundariesCrossingUnloadingAndRejectedRetry) {
  int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  Device device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.p),sizeof(Packet)),cudaSuccess);
  for(const auto c:Curves) for(bool rate:{false,true}) {
    SCOPED_TRACE(c.id);
    SCOPED_TRACE(rate);
    const auto p=Prepare(c,rate);
    for(double start:{std::nextafter(c.x[c.count-1],0.),c.x[c.count-1],
                      std::nextafter(c.x[c.count-1],1.)}) {
      Packet packet;
      packet.parameters=p;
      std::copy_n(c.x,c.count,packet.x); std::copy_n(c.y,c.count,packet.y);
      packet.history.plastic_strain=start;
      auto oracle=NativeInput(p,packet.history,Increment(p,0));
      ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
      Configure<<<1,1>>>(device.p,c.count);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(packet.status,Status::Ok);
      for(unsigned step=0;step<14;++step) {
        const auto in=Increment(p,step==0?0.:step<10?.004:-1.e-5);
        std::copy_n(in.strain_increment,5,oracle.strain_increment.begin());
        native::Result expected;
        ASSERT_TRUE(native::Evaluate(oracle,expected));
        Advance<<<1,1>>>(device.p,in);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
        ASSERT_EQ(packet.status,Status::Ok);
        ComparePoint(packet.result,expected);
        Accept(oracle,expected);
      }
      const auto history=Bytes(packet.history);
      const auto result=Bytes(packet.result);
      auto bad=Increment(p); bad.strain_increment[4]=std::numeric_limits<double>::max();
      Advance<<<1,1>>>(device.p,bad);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
      EXPECT_EQ(packet.status,Status::NonfiniteResult);
      EXPECT_EQ(Bytes(packet.history),history); EXPECT_EQ(Bytes(packet.result),result);
      const auto retry=Increment(p,0);
      std::copy_n(retry.strain_increment,5,oracle.strain_increment.begin());
      native::Result expected;
      ASSERT_TRUE(native::Evaluate(oracle,expected));
      Advance<<<1,1>>>(device.p,retry);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(packet.status,Status::Ok); ComparePoint(packet.result,expected);
    }
  }
}
} // namespace continuation_test
