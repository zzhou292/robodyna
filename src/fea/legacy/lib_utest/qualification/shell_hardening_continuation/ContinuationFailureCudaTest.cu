#include "ContinuationFailureNative.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>

namespace continuation_test {
namespace {
struct Packet {
  Parameters parameters;
  double x[17]{},y[17]{},viscosity[MaximumFailureSteps]{};
  sec::ShellLayeredJ2FailureHistory history;
  sec::ShellLayeredJ2Input input;
  FailureRecord records[MaximumFailureSteps];
  unsigned count=0;
  Status preparation=Status::InvalidParameters;
};
struct Device { Packet* p=nullptr; ~Device(){if(p)cudaFree(p);} };
__global__ void RejectLatePoint(Packet* packet,unsigned count) {
  auto& p=*packet;
  p.preparation=mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{p.x,p.y,count},
      p.parameters.rate,Policy::NativeLastSegment,p.parameters);
  if(p.preparation!=Status::Ok) return;
  auto invalid=p.history;
  invalid.saved.point[2].plastic_strain=static_cast<double>(1e20f);
  p.records[0].status=sec::UpdateShellLayeredJ2Failure(p.parameters,{1.},invalid,p.input,
      2*p.input.dt,p.records[0].section);
}
__global__ void AdvanceFailureTrajectory(Packet* packet,unsigned count) {
  auto& p=*packet;
  p.preparation=mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{p.x,p.y,count},
      p.parameters.rate,Policy::NativeLastSegment,p.parameters);
  if(p.preparation!=Status::Ok) return;
  failure::WorkHistory work;
  unsigned post=0;
  for(unsigned step=0;step<MaximumFailureSteps;++step) {
    auto& record=p.records[step];
    record.reference_thickness=p.input.reference_thickness;
    record.status=sec::UpdateShellLayeredJ2Failure(p.parameters,{1.},p.history,p.input,
        (step+2)*p.input.dt,record.section);
    if(record.status!=Status::Ok) return;
    record.work_valid=sec::ApplyLayeredJ2FailureWork(record.section,p.input.strain_curvature_increment,
        p.input.reference_thickness,.01,p.viscosity[step],work);
    if(!record.work_valid) return;
    record.work=work;
    p.history=record.section.history;
    p.input.reference_thickness=p.input.reported_thickness=record.section.current.reported_thickness;
    ++p.count;
    if(!p.history.element_active && !record.section.removed_now) ++post;
    if(post==2) break;
  }
}
}
TEST(HardeningContinuationCuda, OwnDeviceNip3FailureHistoryMatchesNativeEveryMaskAndPostRemoval) {
  int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  Device device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.p),sizeof(Packet)),cudaSuccess);
  for(const auto c:Curves) for(bool rate:{false,true}) for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(c.id);
    SCOPED_TRACE(rate);
    SCOPED_TRACE(mask);
    const bool virgin=mask==0;
    const auto expected=NativeFailureTrajectory(c,rate,mask,virgin);
    ASSERT_LE(expected.size(),MaximumFailureSteps);
    Packet packet;
    packet.parameters=Prepare(c,rate);
    std::copy_n(c.x,c.count,packet.x); std::copy_n(c.y,c.count,packet.y);
    packet.input=FailureInput(packet.parameters);
    packet.history=FailureSeed(c,mask,virgin);
    // Work-operation input is the independently returned native DM coefficient.
    // This qualifies failure/work ordering, not a second DM formula.
    for(std::size_t i=0;i<expected.size();++i) packet.viscosity[i]=expected[i].trace.diagnostics[8];
    const auto old_history=Bytes(packet.history);
    const auto old_section=Bytes(packet.records[0].section);
    ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
    RejectLatePoint<<<1,1>>>(device.p,c.count);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.preparation,Status::Ok);
    EXPECT_EQ(packet.records[0].status,Status::CurveDomainExceeded);
    EXPECT_EQ(Bytes(packet.history),old_history);
    EXPECT_EQ(Bytes(packet.records[0].section),old_section);
    EXPECT_EQ(packet.count,0u);
    // Retry on the original device history; no host replacement or reseeding.
    AdvanceFailureTrajectory<<<1,1>>>(device.p,c.count);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.preparation,Status::Ok);
    ASSERT_EQ(packet.count,expected.size());
    const std::vector<FailureRecord> actual(packet.records,packet.records+packet.count);
    CompareFailureTrajectory(c,actual,expected,mask!=7);
  }
}
} // namespace continuation_test
