#include "NativeAgreement.h"
#include <cuda_runtime.h>

namespace tab1_force_test {
template<class F> struct DevicePacket {
  typename F::Reference reference;
  typename F::History accepted;
  typename F::Trial output;
  sec::PointParameters material;
  sec::ShellLayeredTab1Parameters failure;
  typename F::Status status=F::Status::kInvalidInput;
};
template<class F> __global__ void AdvanceDevice(DevicePacket<F>* packet,unsigned step,bool bad_last) {
  auto& p=*packet;
  auto base=p.accepted;
  if(bad_last) base.section.saved.point[2].plastic_strain=-1.;
  p.status=EvaluateLayeredTab1Force(p.reference,p.material,p.failure,base,
      Interval(p.reference,step),p.output);
  if(p.status==F::Status::kSuccess)
    p.accepted={p.output.force.proposed_history,p.output.section.history};
}
template<class F> struct DeviceBuffer {
  DevicePacket<F>* p=nullptr;
  ~DeviceBuffer() {if(p) cudaFree(p);}
};
template<class F> void DeviceRecurrence() {
  using Reference=std::conditional_t<std::is_same_v<F,Q>,native::nq::Reference,native::nt::Reference>;
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  DeviceBuffer<F> device,control;
  ASSERT_EQ(cudaMalloc(&device.p,sizeof(*device.p)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&control.p,sizeof(*control.p)),cudaSuccess);
  for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(mask);
    Fixture<F> f(mask);
    const auto reference=native::Reference<Reference>(f.reference.input);
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
        AdvanceDevice<<<1,1>>>(device.p,step,true);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof(packet),cudaMemcpyDeviceToHost),cudaSuccess);
        EXPECT_NE(packet.status,F::Status::kSuccess);
        EXPECT_EQ(tab1_test::Bytes(packet.accepted),accepted);
        EXPECT_EQ(tab1_test::Bytes(packet.output),output);
      }
      AdvanceDevice<<<1,1>>>(device.p,step,false);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device.p,sizeof(packet),cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(packet.status,F::Status::kSuccess);
      if(step==1) {
        AdvanceDevice<<<1,1>>>(control.p,step,false);
        ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        DevicePacket<F> clean;
        ASSERT_EQ(cudaMemcpy(&clean,control.p,sizeof(clean),cudaMemcpyDeviceToHost),cudaSuccess);
        ASSERT_EQ(clean.status,F::Status::kSuccess);
        Exact(ForceValues(packet.output.force),ForceValues(clean.output.force));
        Exact(SectionValues(packet.output.section),SectionValues(clean.output.section));
        Exact(Diagnostics(packet.output.force.diagnostics),Diagnostics(clean.output.force.diagnostics));
      }
      native::Advance(reference,Interval(f.reference,step),f.material,f.failure,native_packet);
      Agreement<F>(packet.output,native_packet,Interval(f.reference,step),thickness);
      removed+=packet.output.section.removed_now;
      if(!was_active) ++post;
    }
    EXPECT_EQ(removed,1u);
    EXPECT_EQ(post,2u);
  }
}
TEST(Tab1ForceCuda,QephDeviceOwnedOriginalGlassRemovalAndAtomicLateFailureRetry) {DeviceRecurrence<Q>();}
TEST(Tab1ForceCuda,T3DeviceOwnedOriginalGlassRemovalAndAtomicLateFailureRetry) {DeviceRecurrence<T>();}
} // namespace tab1_force_test
