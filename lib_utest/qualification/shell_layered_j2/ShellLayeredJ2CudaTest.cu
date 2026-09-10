#include "LayeredJ2Fixture.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <array>
#include <cstring>

namespace layered_j2_test {
namespace {
struct Packet {
  FamilyInputs input;
  q::LayeredJ2ForceTrial quad;
  t::LayeredJ2ForceTrial triangle;
  q::Status quad_status=q::Status::kInvalidInput;
  t::Status triangle_status=t::Status::kInvalidInput;
};
static_assert(sizeof(Packet)<16384,"Bounded one-thread device qualification packet");
__global__ void Evaluate(Packet* packet) {
  const double strain[]{0,.002,.01,.1,.4},yield[]{220e6,250e6,300e6,400e6,500e6};
  sec::PointParameters p;
  if(tl::material::PrepareTabulatedShellPlasticity(200e9,.3,7890,{strain,yield,5},p)!=sec::PointStatus::Ok) return;
  const auto& in=packet->input;
  packet->quad_status=q::EvaluateLayeredJ2Force(in.qr,p,in.qh,in.qi,packet->quad);
  packet->triangle_status=t::EvaluateLayeredJ2Force(in.tr,p,in.th,in.ti,packet->triangle);
}
class DevicePacket {
 public:
  ~DevicePacket() { if(value) EXPECT_EQ(cudaFree(value),cudaSuccess); }
  cudaError_t Allocate() { return cudaMalloc(reinterpret_cast<void**>(&value),sizeof(Packet)); }
  cudaError_t Run(Packet& packet) {
    auto status=cudaMemcpy(value,&packet,sizeof(Packet),cudaMemcpyHostToDevice);
    if(status!=cudaSuccess) return status;
    Evaluate<<<1,1>>>(value); status=cudaGetLastError();
    if(status!=cudaSuccess) return status;
    status=cudaDeviceSynchronize(); if(status!=cudaSuccess) return status;
    return cudaMemcpy(&packet,value,sizeof(Packet),cudaMemcpyDeviceToHost);
  }
 private:
  Packet* value=nullptr;
};
void Near(double a,double b) { EXPECT_NEAR(a,b,1e-15+3e-11*std::max(std::abs(a),std::abs(b))); }
template<class Trial> void Agreement(const Trial& a,const Trial& b,unsigned nodes) {
  for(unsigned n=0;n<nodes;++n) {
    Near(a.force.internal_force[n].x,b.force.internal_force[n].x);
    Near(a.force.internal_force[n].y,b.force.internal_force[n].y);
    Near(a.force.internal_force[n].z,b.force.internal_force[n].z);
    Near(a.force.internal_couple[n].x,b.force.internal_couple[n].x);
    Near(a.force.internal_couple[n].y,b.force.internal_couple[n].y);
    Near(a.force.internal_couple[n].z,b.force.internal_couple[n].z);
  }
  const auto& x=a.force.proposed_history.data(); const auto& y=b.force.proposed_history.data();
  for(unsigned c=0;c<5;++c) { Near(x.stress[c],y.stress[c]); Near(x.material_stress[c],y.material_stress[c]); }
  for(unsigned c=0;c<3;++c) Near(x.bending_stress[c],y.bending_stress[c]);
  for(unsigned c=0;c<8;++c) Near(x.strain_curvature[c],y.strain_curvature[c]);
  for(unsigned c=0;c<2;++c) Near(x.internal_work[c],y.internal_work[c]);
  Near(x.thickness,y.thickness);
  for(unsigned p=0;p<3;++p) {
    for(unsigned c=0;c<5;++c) Near(a.proposed_section.point[p].stress[c],b.proposed_section.point[p].stress[c]);
    Near(a.proposed_section.point[p].plastic_strain,b.proposed_section.point[p].plastic_strain);
  }
  Near(a.section_diagnostics.plastic_work_density_increment,b.section_diagnostics.plastic_work_density_increment);
  Near(a.section_diagnostics.maximum_plastic_strain,b.section_diagnostics.maximum_plastic_strain);
  Near(a.section_diagnostics.mean_plastic_strain,b.section_diagnostics.mean_plastic_strain);
  Near(a.section_diagnostics.minimum_tangent_ratio,b.section_diagnostics.minimum_tangent_ratio);
}
template<class T> auto Bytes(const T& x) {
  std::array<unsigned char,sizeof(T)> result{}; std::memcpy(result.data(),&x,sizeof(T)); return result;
}
TEST(ShellLayeredJ2Cuda, BothAdaptersPersistentPlasticityAndLateFailureAreAtomic) {
  DevicePacket device; ASSERT_EQ(device.Allocate(),cudaSuccess);
  const auto p=Parameters(); Packet packet; packet.input=Families(p);
  for(unsigned step=0;step<2;++step) {
    q::LayeredJ2ForceTrial quad; t::LayeredJ2ForceTrial triangle;
    auto& in=packet.input;
    ASSERT_EQ(q::EvaluateLayeredJ2Force(in.qr,p,in.qh,in.qi,quad),q::Status::kSuccess);
    ASSERT_EQ(t::EvaluateLayeredJ2Force(in.tr,p,in.th,in.ti,triangle),t::Status::kSuccess);
    ASSERT_EQ(device.Run(packet),cudaSuccess);
    ASSERT_EQ(packet.quad_status,q::Status::kSuccess); ASSERT_EQ(packet.triangle_status,t::Status::kSuccess);
    Agreement(packet.quad,quad,4); Agreement(packet.triangle,triangle,3);
    const auto qb=Bytes(packet.quad);
    const auto tb=Bytes(packet.triangle);
    const auto q_saved=in.qh.section.point[2].plastic_strain,t_saved=in.th.section.point[2].plastic_strain;
    in.qh.section.point[2].plastic_strain=in.th.section.point[2].plastic_strain=.401;
    ASSERT_EQ(device.Run(packet),cudaSuccess);
    EXPECT_NE(packet.quad_status,q::Status::kSuccess); EXPECT_NE(packet.triangle_status,t::Status::kSuccess);
    EXPECT_EQ(Bytes(packet.quad),qb); EXPECT_EQ(Bytes(packet.triangle),tb);
    in.qh.section.point[2].plastic_strain=q_saved; in.th.section.point[2].plastic_strain=t_saved;
    ASSERT_EQ(device.Run(packet),cudaSuccess);
    ASSERT_EQ(packet.quad_status,q::Status::kSuccess); ASSERT_EQ(packet.triangle_status,t::Status::kSuccess);
    Agreement(packet.quad,quad,4); Agreement(packet.triangle,triangle,3);
    in.qh={packet.quad.force.proposed_history,packet.quad.proposed_section};
    in.th={packet.triangle.force.proposed_history,packet.triangle.proposed_section};
    in.qi.base_time=in.qh.shell.stamp().time; in.qi.sample_index=in.qh.shell.stamp().sample_index+1;
    in.ti.base_time=in.th.shell.stamp().time; in.ti.sample_index=in.th.shell.stamp().sample_index+1;
  }
  RecordProperty("kernel_threads",1); RecordProperty("owned_device_bytes",static_cast<int>(sizeof(Packet)));
}
} // namespace
} // namespace layered_j2_test
