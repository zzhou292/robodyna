#include "NativeAgreement.h"
#include <cuda_runtime.h>

namespace failure_force_test {
template<class F> struct DevicePacket {
  typename F::Reference reference;typename F::History accepted;typename F::Trial output;
  sec::PointParameters material;sec::ConstantFailureParameters failure;
  double strains[5]{},yields[5]{};typename F::Status status=F::Status::kInvalidInput;
};
template<class F> __global__ void Advance(DevicePacket<F>* packet,unsigned step,bool bad_last) {
  auto& p=*packet;auto base=p.accepted;
  if(p.material.hardening==tl::material::ShellPlasticityHardeningKind::Tabulated) {
    p.material.curve.plastic_strain=p.strains;p.material.curve.yield_stress_pa=p.yields;
  }
  if(bad_last)base.section.saved.point[2].plastic_strain=-1.;
  p.status=EvaluateLayeredJ2FailureForce(p.reference,p.material,p.failure,base,Interval(p.reference,step),p.output);
  if(p.status==F::Status::kSuccess)p.accepted={p.output.force.proposed_history,p.output.section.history};
}
template<class F> void DeviceRecurrence() {
  using N=std::conditional_t<std::is_same_v<F,Q>,native::nq::Reference,native::nt::Reference>;
  DevicePacket<F>* device=nullptr;DevicePacket<F>* control=nullptr;ASSERT_EQ(cudaMalloc(&device,sizeof(*device)),cudaSuccess);
  struct Free {DevicePacket<F>* value;~Free(){cudaFree(value);}} release{device};
  ASSERT_EQ(cudaMalloc(&control,sizeof(*control)),cudaSuccess);Free release_control{control};
  for(bool rate:{false,true})for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(rate);
    SCOPED_TRACE(mask);
    Fixture<F> f(mask);f.material=layered_failure_test::Parameters(false,rate);
    DevicePacket<F> packet;packet.reference=f.reference;packet.accepted=f.accepted;packet.material=f.material;packet.failure=f.failure;
    std::copy_n(f.material.curve.plastic_strain,5,packet.strains);std::copy_n(f.material.curve.yield_stress_pa,5,packet.yields);
    auto n=native::Seed(f.accepted);const auto r=native::Reference<N>(f.reference.input);
    ASSERT_EQ(cudaMemcpy(device,&packet,sizeof(packet),cudaMemcpyHostToDevice),cudaSuccess);
    unsigned inactive_steps=0;
    for(unsigned step=0;step<32&&inactive_steps<2;++step) {
      SCOPED_TRACE(step);const auto before=packet;const double thickness=packet.accepted.shell.data().thickness;
      if(step==1) {
        ASSERT_EQ(cudaMemcpy(control,device,sizeof(packet),cudaMemcpyDeviceToDevice),cudaSuccess);
        std::array<unsigned char,sizeof(packet.accepted)> accepted_bytes;
        std::array<unsigned char,sizeof(packet.output)> output_bytes;
        std::memcpy(accepted_bytes.data(),&packet.accepted,sizeof(packet.accepted));
        std::memcpy(output_bytes.data(),&packet.output,sizeof(packet.output));
        Advance<<<1,1>>>(device,step,true);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
        ASSERT_EQ(cudaMemcpy(&packet,device,sizeof(packet),cudaMemcpyDeviceToHost),cudaSuccess);
        EXPECT_NE(packet.status,F::Status::kSuccess);
        EXPECT_EQ(std::memcmp(&packet.accepted,accepted_bytes.data(),sizeof(packet.accepted)),0);
        EXPECT_EQ(std::memcmp(&packet.output,output_bytes.data(),sizeof(packet.output)),0);
      }
      Advance<<<1,1>>>(device,step,false);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&packet,device,sizeof(packet),cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(packet.status,F::Status::kSuccess);
      EXPECT_TRUE(packet.accepted.shell.matches_reference(packet.reference));
      if(step==1) {
        Advance<<<1,1>>>(control,step,false);ASSERT_EQ(cudaGetLastError(),cudaSuccess);DevicePacket<F> clean;
        ASSERT_EQ(cudaMemcpy(&clean,control,sizeof(clean),cudaMemcpyDeviceToHost),cudaSuccess);
        ASSERT_EQ(clean.status,F::Status::kSuccess);
        Exact(ForceValues(packet.output.force),ForceValues(clean.output.force));
        Exact(Values(packet.output.section),Values(clean.output.section));
        Exact(Diagnostics(packet.output.force.diagnostics),Diagnostics(clean.output.force.diagnostics));
      }
      native::Advance(r,Interval(f.reference,step),f.material,f.failure.failure_strain,n);
      Agreement<F>(packet.output,n,Interval(f.reference,step),thickness);
      if(!before.accepted.section.element_active)++inactive_steps;
      f.accepted=packet.accepted;
    }
    EXPECT_EQ(inactive_steps,2);
  }
}
TEST(ShellFailureForceCuda,QephDeviceOwnedRemovalHistoryAndLateFailureRetry) {DeviceRecurrence<Q>();}
TEST(ShellFailureForceCuda,T3DeviceOwnedRemovalHistoryAndLateFailureRetry) {DeviceRecurrence<T>();}
} // namespace failure_force_test
