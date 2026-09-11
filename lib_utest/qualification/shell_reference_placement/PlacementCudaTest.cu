#include "PlacementNativeSupport.h"
#include <cuda_runtime.h>
#include <limits>
namespace placement_test {
namespace {
struct Packet {
  sec::PointParameters material;
  sec::ShellLayeredTab1Parameters failure;
  sec::ShellLayeredTab1History history;
  sec::ShellLayeredTab1Result section;
  Work work;
  PointStatus status=PointStatus::InvalidParameters;
};
struct Device {
  Packet* data=nullptr;
  ~Device(){if(data)cudaFree(data);}
};
void SameAccepted(const Packet& a,const Packet& b) {
  SameNativeState(NativeSeed(a.history),NativeSeed(b.history));
  for(unsigned p=0;p<3;++p)
    EXPECT_EQ(Bytes(a.history.current_force_point[p].stress),Bytes(b.history.current_force_point[p].stress));
  EXPECT_EQ(Bytes(a.work.stress),Bytes(b.work.stress));
  EXPECT_EQ(Bytes(a.work.material_stress),Bytes(b.work.material_stress));
  EXPECT_EQ(Bytes(a.work.bending_stress),Bytes(b.work.bending_stress));
  EXPECT_EQ(Bytes(a.work.strain_curvature),Bytes(b.work.strain_curvature));
  EXPECT_EQ(Bytes(a.work.internal_work),Bytes(b.work.internal_work));
  EXPECT_EQ(Bytes(a.work.thickness),Bytes(b.work.thickness));
  EXPECT_EQ(Bytes(a.section.constitutive_increment),Bytes(b.section.constitutive_increment));
  EXPECT_EQ(Bytes(a.section.caller_failure_increment),Bytes(b.section.caller_failure_increment));
  EXPECT_EQ(a.section.removed_now,b.section.removed_now);
}
__global__ void Configure(Packet* packet) {
  packet->status=mat::PrepareLinearLaw44ShellPlasticity(70e9,.22,2500,{30e6,1e9},
      {true,0,1,10000,mat::ShellPlasticityRatePolicy::FilteredZeroC},packet->material);
}
__global__ void Advance(Packet* packet,sec::ShellLayeredJ2Input input,double time,
    double viscosity,bool late_failure) {
  auto& p=*packet;
  auto base=p.history;
  if(late_failure) base.saved.point[2].filtered_rate_per_s=__longlong_as_double(0x7ff8000000000000ULL);
  sec::ShellLayeredTab1Result candidate;
  auto work=p.work;
  p.status=sec::UpdateShellLayeredTab1(p.material,p.failure,base,input,time,candidate);
  if(p.status!=PointStatus::Ok) return;
  if(!sec::ApplyLayeredTab1Work(candidate,input.strain_curvature_increment,
      input.reference_thickness,.001,viscosity,work,input.placement)) {
    p.status=PointStatus::NonfiniteResult;
    return;
  }
  p.history=candidate.history;
  p.section=candidate;
  p.work=work;
}
}
TEST(ShellPlacementCuda, DeviceOwnedAllPlanesAndMasksMatchNativeThroughRemovalAndRetry) {
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  Device device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.data),2*sizeof(Packet)),cudaSuccess);
  for(auto plane:Planes) for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(NativeIpos(plane));
    SCOPED_TRACE(mask);
    Packet packet;
    packet.failure.table=Table();packet.history=Seed(mask);packet.work.thickness=.00228;
    auto native=NativeSeed(packet.history);native.thickness=packet.work.thickness;
    ASSERT_EQ(cudaMemcpy(device.data,&packet,sizeof packet,cudaMemcpyHostToDevice),cudaSuccess);
    Configure<<<1,1>>>(device.data);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet,device.data,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(packet.status,PointStatus::Ok);
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32;++step) {
      const auto input=Input(step,packet.work.thickness,plane);
      NativeTrace trace;
      NativePlacementStep(input,(step+1)*input.dt,.001,.02,native,trace);
      const auto accepted=packet;
      if(step==2) {
        ASSERT_EQ(cudaMemcpy(device.data+1,&accepted,sizeof accepted,cudaMemcpyHostToDevice),cudaSuccess);
        Advance<<<1,1>>>(device.data+1,input,(step+1)*input.dt,trace.diagnostics[8],false);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        Advance<<<1,1>>>(device.data,input,(step+1)*input.dt,trace.diagnostics[8],true);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        ASSERT_EQ(cudaMemcpy(&packet,device.data,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
        EXPECT_NE(packet.status,PointStatus::Ok);
        EXPECT_EQ(Bytes(packet.history),Bytes(accepted.history));
        EXPECT_EQ(Bytes(packet.section),Bytes(accepted.section));
        EXPECT_EQ(Bytes(packet.work),Bytes(accepted.work));
      }
      Advance<<<1,1>>>(device.data,input,(step+1)*input.dt,trace.diagnostics[8],false);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device.data,sizeof packet,cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(packet.status,PointStatus::Ok);
      if(step==2) {
        Packet control;
        ASSERT_EQ(cudaMemcpy(&control,device.data+1,sizeof control,cudaMemcpyDeviceToHost),cudaSuccess);
        ASSERT_EQ(control.status,PointStatus::Ok);
        SameAccepted(packet,control);
      }
      Compare(packet.section,packet.work,native,trace,input.reference_thickness,.001);
      for(unsigned p=0;p<3;++p)
        Close(packet.section.caller_failure_increment[p],native.points[7*p+5]-accepted.history.saved.point[p].plastic_strain);
      removed+=packet.section.removed_now;post+=!accepted.history.element_active;
    }
    EXPECT_EQ(removed,1u);EXPECT_GE(post,2u);
  }
}
} // namespace placement_test
