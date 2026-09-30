#include "../native/law44/NativeAnalyticTestSupport.h"
#include <cuda_runtime.h>
#include <limits>

namespace analytic_test {
namespace {
struct Packet { Parameters parameters; History accepted; Input input; Result result; Status status{}; };
struct Device { Packet* value=nullptr; ~Device(){if(value)cudaFree(value);} };
__global__ void Configure(Packet* p,double e,double nu,double rho,double a,double etan) {
  p->status=mat::PrepareLinearLaw44ShellPlasticity(e,nu,rho,{a,etan},{true,8000,8,10000},p->parameters);
}
__global__ void Advance(Packet* p) {
  p->status=mat::UpdateLaw44ShellPlasticity(p->parameters,p->accepted,p->input,p->result);
}
}
TEST(Law44AnalyticCuda, DevicePreparedSourceParametersAndIndependentNativeHistory) {
  int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  Device device; ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.value),sizeof(Packet)),cudaSuccess);
  for(const auto source:Sources) {
    Packet host;
    ASSERT_EQ(cudaMemcpy(device.value,&host,sizeof host,cudaMemcpyHostToDevice),cudaSuccess);
    Configure<<<1,1>>>(device.value,source.e*1e6,.3,source.rho,source.sigy*1e6,source.etan*1e6);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&host,device.value,sizeof host,cudaMemcpyDeviceToHost),cudaSuccess); ASSERT_EQ(host.status,Status::Ok);
    auto oracle=Native(source);
    double actual_thickness=.002,native_thickness=.002;
    unsigned yielded=0;
    for(unsigned step=0;step<384;++step) {
      SCOPED_TRACE(source.mid);
      SCOPED_TRACE(step);
      const double sign=step<128?1.:step<160?0.:-1.;
      oracle.point.strain_increment={sign*.0005,sign*-.0001,sign*.0002,sign*.00001,sign*-.00002};
      std::array<double,8> dx{}; std::copy(oracle.point.strain_increment.begin(),oracle.point.strain_increment.end(),dx.begin());
      oracle.point.rate.total_shell_rate_per_s=native::NativeShellRate(dx,native_thickness,Dt);
      native::AnalyticResult expected;
      ASSERT_TRUE(native::EvaluateAnalytic(oracle,.25*native_thickness,native_thickness,expected));
      EXPECT_DOUBLE_EQ(host.parameters.plastic_hardening_pa,expected.native_plastic_hardening);
      std::copy(oracle.point.strain_increment.begin(),oracle.point.strain_increment.end(),host.input.strain_increment);
      host.input.transverse_shear_modulus=oracle.point.transverse_shear_modulus;
      host.input.dt=Dt; host.input.total_strain_rate_per_s=oracle.point.rate.total_shell_rate_per_s;
      ASSERT_EQ(cudaMemcpy(device.value,&host,sizeof host,cudaMemcpyHostToDevice),cudaSuccess);
      Advance<<<1,1>>>(device.value); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&host,device.value,sizeof host,cudaMemcpyDeviceToHost),cudaSuccess); ASSERT_EQ(host.status,Status::Ok);
      Compare(host.result,expected);
      const auto layer=.25*actual_thickness;
      actual_thickness+=host.result.elastic_thickness_strain*layer;
      actual_thickness+=host.result.plastic_thickness_strain*layer;
      Close(actual_thickness,expected.reported_thickness_m,2e-14);
      yielded+=host.result.plastic_increment>0; host.accepted=host.result.history;
      Accept(oracle,expected); native_thickness=expected.reported_thickness_m;
    }
    EXPECT_GT(yielded,50u);
    const auto saved=Bytes(host.result); const auto state=Bytes(host.accepted);
    host.input.strain_increment[4]=std::numeric_limits<double>::max();
    ASSERT_EQ(cudaMemcpy(device.value,&host,sizeof host,cudaMemcpyHostToDevice),cudaSuccess);
    Advance<<<1,1>>>(device.value); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&host,device.value,sizeof host,cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_EQ(host.status,Status::NonfiniteResult); EXPECT_EQ(Bytes(host.result),saved); EXPECT_EQ(Bytes(host.accepted),state);
  }
}
} // namespace analytic_test
