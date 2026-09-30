#include "native/NativePacket.h"
#include "../shell_tab1_force/NativeAgreement.h"
#include <cuda_runtime.h>

namespace placement_force_test {
namespace sec=tl::fea::sections;
using base::Diagnostics;
template<class F> struct DevicePacket {
  typename F::Reference reference;
  typename F::History accepted;
  typename F::Trial output;
  sec::PointParameters material;
  sec::ShellLayeredTab1Parameters failure;
  typename F::Status status=F::Status::kInvalidInput;
};
template<class F> __global__ void AdvanceDevice(DevicePacket<F>* packet,unsigned step,unsigned fault) {
  auto& p=*packet;
  auto base=p.accepted;
  auto reference=p.reference;
  if(fault==1) base.section.saved.point[2].plastic_strain=-1.;
  if(fault==2) {
    auto input=reference.input;
    input.placement=input.placement==Placement::TopReferencePlane?
        Placement::BottomReferencePlane:Placement::TopReferencePlane;
    if(InitializeReference(input,reference)!=F::Status::kSuccess) return;
  }
  p.status=EvaluateLayeredTab1Force(reference,p.material,p.failure,base,
      Interval(p.reference,step),p.output);
  if(p.status==F::Status::kSuccess)
    p.accepted={p.output.force.proposed_history,p.output.section.history};
}
template<class F> struct DeviceBuffer {
  DevicePacket<F>* p=nullptr;
  ~DeviceBuffer() {if(p) cudaFree(p);}
};
template<class F> void DeviceRecurrence() {
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  DeviceBuffer<F> device,control;
  ASSERT_EQ(cudaMalloc(&device.p,sizeof(*device.p)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&control.p,sizeof(*control.p)),cudaSuccess);
  for(auto plane:Planes) for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(static_cast<unsigned>(plane));
    SCOPED_TRACE(mask);
    Fixture<F> f(plane,mask);
    auto native_packet=native::Seed(f.accepted);
    DevicePacket<F> packet;
    packet.reference=f.reference;
    packet.accepted=f.accepted;
    packet.material=f.material;
    packet.failure=f.failure;
    ASSERT_EQ(cudaMemcpy(device.p,&packet,sizeof(packet),cudaMemcpyHostToDevice),cudaSuccess);
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32&&post<2;++step) {
      SCOPED_TRACE(step);
      const bool was_active=packet.accepted.section.element_active;
      const double thickness=packet.accepted.shell.data().thickness;
      if(step==1) {
        ASSERT_EQ(cudaMemcpy(control.p,device.p,sizeof(packet),cudaMemcpyDeviceToDevice),cudaSuccess);
        const auto accepted=tab1_test::Bytes(packet.accepted);
        const auto output=tab1_test::Bytes(packet.output);
        for(unsigned fault:{1u,2u}) {
          AdvanceDevice<<<1,1>>>(device.p,step,fault);
          ASSERT_EQ(cudaGetLastError(),cudaSuccess);
          ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof(packet),cudaMemcpyDeviceToHost),cudaSuccess);
          EXPECT_NE(packet.status,F::Status::kSuccess);
          EXPECT_EQ(tab1_test::Bytes(packet.accepted),accepted);
          EXPECT_EQ(tab1_test::Bytes(packet.output),output);
        }
      }
      AdvanceDevice<<<1,1>>>(device.p,step,0u);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof(packet),cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(packet.status,F::Status::kSuccess);
      if(step==1) {
        AdvanceDevice<<<1,1>>>(control.p,step,0u);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        DevicePacket<F> clean;
        ASSERT_EQ(cudaMemcpy(&clean,control.p,sizeof(clean),cudaMemcpyDeviceToHost),cudaSuccess);
        ASSERT_EQ(clean.status,F::Status::kSuccess);
        Exact(ForceValues(packet.output.force),ForceValues(clean.output.force));
        Exact(SectionValues(packet.output.section),SectionValues(clean.output.section));
        Exact(Diagnostics(packet.output.force.diagnostics),Diagnostics(clean.output.force.diagnostics));
      }
      native::Advance(f.reference,Interval(f.reference,step),f.material,f.failure,native_packet);
      base::Agreement<F>(packet.output,native_packet,Interval(f.reference,step),thickness);
      removed+=packet.output.section.removed_now;
      if(!was_active) ++post;
    }
    EXPECT_EQ(removed,1u);
    EXPECT_EQ(post,2u);
  }
}
TEST(PlacementForceCuda,QephDeviceOwnedBothSignsNativeRemovalAndLateFailureRetry) {DeviceRecurrence<Q>();}
TEST(PlacementForceCuda,T3DeviceOwnedBothSignsNativeRemovalAndLateFailureRetry) {DeviceRecurrence<T>();}
} // namespace placement_force_test
