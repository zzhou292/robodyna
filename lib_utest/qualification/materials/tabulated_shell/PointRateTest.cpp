#include "lib_src/elements/sections/ShellLayeredJ2.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
using namespace tl::material;
using Status=TabulatedShellPlasticityStatus;
constexpr double X[]{0,.1,.3},Y[]{270e6,340e6,362e6};
constexpr TabulatedShellPlasticityRate SourceRate{true,8000,8,10000};
TabulatedShellPlasticityParameters Parameters() {
  TabulatedShellPlasticityParameters p;
  EXPECT_EQ(PrepareTabulatedShellPlasticity(200e9,.3,7890,{X,Y,3},SourceRate,p),Status::Ok);
  return p;
}
void Near(double a,long double b) {
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b));
  EXPECT_LE(std::abs(static_cast<long double>(a)-b),
      64*std::numeric_limits<double>::epsilon()*std::max(1.L,std::abs(b)));
}
TEST(TabulatedShellRate, ExplicitDeclarationAndHistoryFailuresAreAtomic) {
  const auto valid=Parameters(); auto p=valid;
  const double nan=std::numeric_limits<double>::quiet_NaN();
  for(auto rate:{TabulatedShellPlasticityRate{false,8000,8,10000},
      TabulatedShellPlasticityRate{true,0,8,10000},TabulatedShellPlasticityRate{true,8000,-8,10000},
      TabulatedShellPlasticityRate{true,8000,8,nan},
      TabulatedShellPlasticityRate{true,std::numeric_limits<double>::denorm_min(),8,10000}}) {
    EXPECT_EQ(PrepareTabulatedShellPlasticity(200e9,.3,7890,{X,Y,3},rate,p),Status::InvalidParameters);
    EXPECT_EQ(std::memcmp(&p,&valid,sizeof p),0);
  }
  TabulatedShellPlasticityHistory base; base.filtered_rate_per_s=12;
  TabulatedShellPlasticityInput input; input.dt=0x1p-24;
  input.total_strain_rate_per_s=100; input.transverse_shear_modulus=p.shear_modulus;
  TabulatedShellPlasticityResult clean;
  ASSERT_EQ(UpdateTabulatedShellPlasticity(p,base,input,clean),Status::Ok);
  auto output=clean;
  for(double bad:{0.,-1.,nan}) {
    auto invalid=input; invalid.dt=bad;
    EXPECT_EQ(UpdateTabulatedShellPlasticity(p,base,invalid,output),Status::InvalidIncrement);
    EXPECT_EQ(std::memcmp(&output,&clean,sizeof output),0);
  }
  auto invalid=base; invalid.filtered_rate_per_s=-1;
  EXPECT_EQ(UpdateTabulatedShellPlasticity(p,invalid,input,output),Status::InvalidHistory);
  EXPECT_EQ(std::memcmp(&output,&clean,sizeof output),0);
  auto bad=input; bad.total_strain_rate_per_s=nan;
  EXPECT_EQ(UpdateTabulatedShellPlasticity(p,base,bad,output),Status::InvalidIncrement);
  EXPECT_EQ(std::memcmp(&output,&clean,sizeof output),0);
  ASSERT_EQ(UpdateTabulatedShellPlasticity(p,base,input,output),Status::Ok);
  EXPECT_EQ(std::memcmp(&output,&clean,sizeof output),0);
  EXPECT_EQ(base.filtered_rate_per_s,12);
}
TEST(TabulatedShellRate, SourceFilterRiseDecayAndSaturationFollowIndependentRecurrence) {
  const auto p=Parameters(); TabulatedShellPlasticityHistory base;
  long double oracle=0;
  for(unsigned step=0;step<64;++step) {
    TabulatedShellPlasticityInput input; input.dt=step==63?1e-3:0x1p-24;
    input.total_strain_rate_per_s=step<32?100.:0.;
    input.transverse_shear_modulus=p.shear_modulus;
    const long double alpha=std::min(1.L,2*std::acos(-1.L)*10000*input.dt);
    oracle=alpha*input.total_strain_rate_per_s+(1-alpha)*oracle;
    TabulatedShellPlasticityResult next;
    ASSERT_EQ(UpdateTabulatedShellPlasticity(p,base,input,next),Status::Ok);
    Near(next.history.filtered_rate_per_s,oracle);
    Near(next.yield_before_pa,270e6L*(1+std::pow(oracle/8000.L,1.L/8)));
    EXPECT_EQ(next.history.plastic_strain,0); EXPECT_EQ(next.plastic_increment,0);
    for(double value:next.history.stress) EXPECT_EQ(value,0);
    base=next.history;
  }
  EXPECT_EQ(base.filtered_rate_per_s,0);
}
TEST(TabulatedShellRate, SectionSharesNativeTotalRateAndPreservesDistinctPointFilters) {
  namespace sec=tl::fea::sections;
  const auto p=Parameters(); sec::ShellLayeredJ2History base;
  for(unsigned i=0;i<3;++i) base.point[i].filtered_rate_per_s=10.*i;
  sec::ShellLayeredJ2Input input;
  input.dt=0x1p-24; input.reference_thickness=.001648;
  input.reported_thickness=.0015; input.transverse_shear_modulus=p.shear_modulus*5./6.;
  auto& d=input.strain_curvature_increment;
  d[0]=2e-7; d[1]=-1e-7; d[2]=3e-8; d[5]=.0002; d[6]=-.0001; d[7]=.0003;
  const long double ex=d[0],ey=d[1],xy=d[2],kx=d[5],ky=d[6],kxy=d[7],t=input.reported_thickness;
  const long double total=std::sqrt((4.L/3)*(ex*ex+ey*ey+ex*ey+.25L*xy*xy)+
      (kx*kx+ky*ky+kx*ky+.25L*kxy*kxy)*(1.L/9)*t*t)/input.dt;
  Near(sec::LayeredJ2TotalStrainRate(input),total);
  const long double alpha=2*std::acos(-1.L)*10000*input.dt;
  sec::ShellLayeredJ2Result result;
  ASSERT_EQ(sec::UpdateShellLayeredJ2(p,base,input,result),Status::Ok);
  long double mean_yield=0;
  for(unsigned i=0;i<3;++i) {
    const long double rate=alpha*total+(1-alpha)*base.point[i].filtered_rate_per_s;
    Near(result.history.point[i].filtered_rate_per_s,rate);
    mean_yield+=(i==1?.5L:.25L)*270e6L*(1+std::pow(rate/8000.L,1.L/8));
  }
  EXPECT_EQ(result.diagnostics.minimum_tangent_ratio,1);
  EXPECT_EQ(result.diagnostics.mean_tangent_ratio,1);
  Near(result.diagnostics.mean_yield_before_pa,mean_yield);
  Near(result.diagnostics.last_point_yield_before_pa,
      270e6L*(1+std::pow(static_cast<long double>(result.history.point[2].filtered_rate_per_s)/8000,1.L/8)));
  const auto saved=result; auto invalid=base;
  invalid.point[2].filtered_rate_per_s=-1;
  EXPECT_EQ(sec::UpdateShellLayeredJ2(p,invalid,input,result),Status::InvalidHistory);
  EXPECT_EQ(std::memcmp(&result,&saved,sizeof result),0);
}
} // namespace
