#pragma once
#include "NativeAnalytic.h"
#include "../../law44_analytic/AnalyticTestSupport.h"
#include "NativeRateTestSupport.h"

namespace analytic_test {
namespace native=tl::qualification::law44;
using native::rate_test::Close;
inline native::AnalyticInput Native(Source s=Sources[0]) {
  native::AnalyticInput in;
  in.initial_yield=s.sigy*1e6; in.tangent_modulus=s.etan*1e6;
  in.point.young=s.e*1e6; in.point.poisson=.3; in.point.density=s.rho;
  in.point.transverse_shear_modulus=(5./6.)*(s.e*1e6/2./1.3);
  in.point.rate={true,8000,8,0,native::NativeFilterCoefficient(10000,Dt),0};
  return in;
}
inline void Compare(const Result& actual,const native::AnalyticResult& expected) {
  for(unsigned c=0;c<5;++c) Close(actual.history.stress[c],expected.stress[c],1e-8);
  Close(actual.history.plastic_strain,expected.plastic_strain,2e-14);
  Close(actual.history.filtered_rate_per_s,expected.filtered_rate_per_s,2e-11);
  Close(actual.plastic_increment,expected.plastic_increment,2e-14);
  Close(actual.tangent_ratio,expected.tangent_ratio,2e-14);
  Close(actual.yield_before_pa,expected.yield_before,1e-8);
  Close(actual.equivalent_stress_pa,expected.equivalent_stress,1e-8);
  Close(actual.plastic_work_density,expected.plastic_work_density,1e-8);
}
inline void Accept(native::AnalyticInput& in,const native::AnalyticResult& next) {
  in.point.accepted_stress=next.stress; in.point.accepted_plastic_strain=next.plastic_strain;
  in.point.rate.accepted_filtered_rate_per_s=next.filtered_rate_per_s;
}
} // namespace analytic_test
