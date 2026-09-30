#include "PlacementNativeSupport.h"
#include <cuda_runtime.h>
namespace placement_test {
namespace {
struct Coefficients {
  double values[8]{};
  bool valid=false;
};
struct DeviceCoefficients {
  Coefficients* data=nullptr;
  ~DeviceCoefficients(){if(data)cudaFree(data);}
};
__global__ void EvaluateCoefficients(Coefficients* output,Placement plane,double active) {
  namespace q=tl::fea::qeph;
  namespace t=tl::fea::t3;
  constexpr double area=.003,thickness=.00228,length=.01;
  q::ReferenceInput qi;qi.thickness=thickness;qi.density=2500;qi.young_modulus=70e9;qi.poisson_ratio=.22;
  t::ReferenceInput ti;ti.thickness=thickness;ti.density=2500;ti.young_modulus=70e9;ti.poisson_ratio=.22;
  q::detail::MaterialWork qm;t::detail::MaterialWork tm;
  Coefficients candidate;
  if(!q::detail::PrepareMaterial(qi,area,1.e-7,qm,plane)||
      !t::detail::PrepareMaterial(ti,area,tm,plane)) {output->valid=false;return;}
  q::detail::GeometryWork qg;qg.values.area=area;qg.values.characteristic_length=length;
  t::detail::GeometryWork tg;tg.kinematics.area=area;tg.kinematics.area_scale=1;
  tg.kinematics.characteristic_length=length;
  q::ForceDiagnostics qd;t::ForceDiagnostics td;
  q::detail::StiffnessDiagnostics(qg,qm,qd,active);
  t::detail::StiffnessDiagnostics(tg,tm,td,active);
  candidate.values[0]=tl::fea::NativeQephPlacementInertia(.1,area,thickness,plane);
  candidate.values[1]=tl::fea::NativeT3PlacementInertia(.1,area,thickness);
  candidate.values[2]=qm.offset;
  candidate.values[3]=tl::fea::NativeShellOffsetStiffnessFactor(qm.offset,thickness);
  candidate.values[4]=qd.translational_stiffness;candidate.values[5]=qd.rotational_stiffness;
  candidate.values[6]=td.translational_stiffness;candidate.values[7]=td.rotational_stiffness;
  candidate.valid=true;
  *output=candidate;
}
}
TEST(ShellPlacementCuda, NativeFamilyOffsetMassAndStiffnessExecuteOnDevice) {
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0);
  DeviceCoefficients device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.data),sizeof(Coefficients)),cudaSuccess);
  for(auto plane:Planes) for(double active:{1.,0.}) {
    double shift,native[10];
    placement_native_shift(NativeIpos(plane),&shift);
    placement_native_coefficients(shift,.1,.003,.00228,70e9/(1-.22*.22),
        70e9/(2*(1+.22)),5./6.,.01,.015,std::sqrt(70e9/2500.),active,native);
    EvaluateCoefficients<<<1,1>>>(device.data,plane,active);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Coefficients result;
    ASSERT_EQ(cudaMemcpy(&result,device.data,sizeof result,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_TRUE(result.valid);
    for(unsigned i=0;i<8;++i) Close(result.values[i],native[i]);
  }
  EvaluateCoefficients<<<1,1>>>(device.data,static_cast<Placement>(255),1.);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Coefficients rejected;
  ASSERT_EQ(cudaMemcpy(&rejected,device.data,sizeof rejected,cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_FALSE(rejected.valid);
}
} // namespace placement_test
