#include "Law1TestSupport.h"
#include "../native/law1/NativeReference.h"
#include <cuda_runtime_api.h>
namespace layered_law1_test {
struct Packet {double e=200e9,nu=.3,rho=7890;History history;Input input;Result result;bool prepared=false,ok=false;};
__global__ void Evaluate(Packet* p) {
  Parameters material;p->prepared=mat::PrepareShellElasticLaw1Point(p->e,p->nu,p->rho,material);
  p->ok=p->prepared&&sec::UpdateShellLayeredLaw1(material,p->history,p->input,p->result);
}
struct Device {Packet* p=nullptr;~Device(){if(p)cudaFree(p);}};
TEST(LayeredLaw1Cuda, ActualNativeIndependentHistoryAndLateRollback) {
  Device device;ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.p),sizeof(Packet)),cudaSuccess);
  Packet p;const auto material=Material();double h=.002,nh=.002;
  tl::qualification::law1::SectionInput native;native.young=p.e;native.nu=p.nu;native.rho=p.rho;native.gs=material.elastic.g*5./6.;
  for(unsigned i=0;i<160;++i) {
    p.input=Step(i,h,native.gs);native.reference_thickness=native.reported_thickness=nh;
    std::copy_n(p.input.strain_curvature_increment,8,native.increment.begin());
    tl::qualification::law1::SectionResult n;ASSERT_TRUE(tl::qualification::law1::Evaluate(native,&n));
    ASSERT_EQ(cudaMemcpy(device.p,&p,sizeof p,cudaMemcpyHostToDevice),cudaSuccess);
    Evaluate<<<1,1>>>(device.p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&p,device.p,sizeof p,cudaMemcpyDeviceToHost),cudaSuccess);ASSERT_TRUE(p.prepared);ASSERT_TRUE(p.ok);
    for(unsigned k=0;k<3;++k)for(unsigned c=0;c<5;++c)NativeClose(p.result.history.point[k].stress[c],n.stress[5*k+c]);
    for(unsigned c=0;c<5;++c)NativeClose(p.result.material_stress[c],n.force[c]);
    for(unsigned c=0;c<3;++c)NativeClose(p.result.bending_stress[c],n.moment[c]);
    ThicknessClose(p.result.reported_thickness,n.thickness);
    p.history=p.result.history;h=p.result.reported_thickness;native.stress=n.stress;nh=n.thickness;
  }
  p.history.point[2].stress[2]=std::numeric_limits<double>::max();
  for(double& x:p.input.strain_curvature_increment)x=0;p.input.strain_curvature_increment[2]=1.e297;
  const auto before=Bytes(p.result);const auto history=Bytes(p.history);
  ASSERT_EQ(cudaMemcpy(device.p,&p,sizeof p,cudaMemcpyHostToDevice),cudaSuccess);
  Evaluate<<<1,1>>>(device.p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&p,device.p,sizeof p,cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_FALSE(p.ok);EXPECT_EQ(Bytes(p.result),before);EXPECT_EQ(Bytes(p.history),history);
}
} // namespace layered_law1_test
