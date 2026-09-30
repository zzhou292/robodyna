#include "Tab1NativeSupport.h"
#include <cuda_runtime.h>
#include <limits>
namespace tab1_test {
namespace {
struct Packet {
  sec::PointParameters parameters;
  sec::ShellLayeredTab1Parameters failure;
  sec::ShellLayeredTab1History history;
  sec::ShellLayeredTab1Result section;
  Work work;
  PointStatus status=PointStatus::InvalidParameters;
};
struct Device { Packet* p=nullptr; ~Device(){if(p)cudaFree(p);} };
__global__ void Configure(Packet* packet) {
  auto& p=*packet;
  p.status=mat::PrepareLinearLaw44ShellPlasticity(70e9,.22,2500,{30e6,1e9},
      {true,0,1,10000,mat::ShellPlasticityRatePolicy::FilteredZeroC},p.parameters);
}
__global__ void AdvanceSection(Packet* packet,sec::ShellLayeredJ2Input in,double time,double viscosity) {
  auto& p=*packet;
  sec::ShellLayeredTab1Result next;
  auto work=p.work;
  p.status=sec::UpdateShellLayeredTab1(p.parameters,p.failure,p.history,in,time,next);
  if(p.status!=PointStatus::Ok)return;
  if(!sec::ApplyLayeredTab1Work(next,in.strain_curvature_increment,in.reference_thickness,.001,
      viscosity,work)) {p.status=PointStatus::NonfiniteResult;return;}
  p.history=next.history;
  p.section=next;
  p.work=work;
}
}
TEST(Tab1Cuda, DeviceOwnedGlassAnyPointHistoryWorkAndLateFailureRetryMatchNative) {
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  Device device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.p),sizeof(Packet)),cudaSuccess);
  for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(mask);
    Packet packet;
    packet.failure.table=Table();
    packet.history=Seed(mask);
    auto native=NativeSeed(packet.history);
    ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
    Configure<<<1,1>>>(device.p);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.status,PointStatus::Ok);
    for(unsigned step=0;step<24;++step) {
      SCOPED_TRACE(step);
      const auto in=Increment(step,packet.work.thickness);
      NativeTrace trace;
      NativeStep(in,(step+1)*in.dt,.001,.02,native,trace);
      AdvanceSection<<<1,1>>>(device.p,in,(step+1)*in.dt,trace.diagnostics[8]);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(packet.status,PointStatus::Ok);
      Compare(packet.section,packet.work,native,trace,in.reference_thickness,.001);
    }
    const auto h=Bytes(packet.history);
    const auto section=Bytes(packet.section);
    const auto work=Bytes(packet.work);
    auto bad=Increment(24,packet.work.thickness);
    bad.strain_curvature_increment[4]=std::numeric_limits<double>::max();
    AdvanceSection<<<1,1>>>(device.p,bad,25*bad.dt,0.);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_EQ(packet.status,PointStatus::NonfiniteResult);
    EXPECT_EQ(Bytes(packet.history),h); EXPECT_EQ(Bytes(packet.section),section); EXPECT_EQ(Bytes(packet.work),work);
    const auto retry=Increment(24,packet.work.thickness);
    NativeTrace trace;
    NativeStep(retry,25*retry.dt,.001,.02,native,trace);
    AdvanceSection<<<1,1>>>(device.p,retry,25*retry.dt,trace.diagnostics[8]);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.status,PointStatus::Ok);
    Compare(packet.section,packet.work,native,trace,retry.reference_thickness,.001);
  }
}
} // namespace tab1_test
