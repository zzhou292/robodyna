#pragma once
#include "ContinuationFixture.h"
#include "lib_utest/qualification/native/law44/NativePoint.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

namespace continuation_test {
namespace native=tl::qualification::law44;
inline native::Input NativeInput(const Parameters& p, const History& base, const Input& in) {
  native::Input n;
  n.young=p.young_pa; n.poisson=p.poisson_ratio; n.density=p.density_kg_m3;
  n.transverse_shear_modulus=in.transverse_shear_modulus;
  n.plastic_strain=p.curve.plastic_strain; n.yield_stress=p.curve.yield_stress_pa;
  n.point_count=p.curve.count;
  std::copy_n(base.stress,5,n.accepted_stress.begin());
  std::copy_n(in.strain_increment,5,n.strain_increment.begin());
  n.accepted_plastic_strain=base.plastic_strain;
  n.continuation=native::CurveContinuation::NativeLastSegment;
  if(p.rate.enabled) n.rate={true,p.rate.cowper_symonds_c_per_s,p.rate.cowper_symonds_p,
      in.total_strain_rate_per_s,native::NativeFilterCoefficient(p.rate.cutoff_hz,in.dt),
      base.filtered_rate_per_s};
  return n;
}
inline void Close(double actual,double expected,double absolute) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  EXPECT_NEAR(actual,expected,absolute+2.e-11*std::max(std::abs(actual),std::abs(expected)));
}
inline void ComparePoint(const Result& actual,const native::Result& expected) {
  for(unsigned i=0;i<5;++i) Close(actual.history.stress[i],expected.stress[i],1.e-8);
  Close(actual.history.plastic_strain,expected.plastic_strain,2.e-14);
  Close(actual.history.filtered_rate_per_s,expected.filtered_rate_per_s,1.e-10);
  Close(actual.plastic_increment,expected.plastic_increment,2.e-14);
  Close(actual.tangent_ratio,expected.tangent_ratio,2.e-14);
  Close(actual.elastic_thickness_strain+actual.plastic_thickness_strain,expected.total_thickness_strain,2.e-14);
  Close(actual.yield_before_pa,expected.yield_before,1.e-8);
  Close(actual.equivalent_stress_pa,expected.equivalent_stress,1.e-8);
  Close(actual.plastic_work_density,expected.plastic_work_density,1.e-8);
}
inline void Accept(native::Input& n,const native::Result& result) {
  n.accepted_stress=result.stress;
  n.accepted_plastic_strain=result.plastic_strain;
  n.rate.accepted_filtered_rate_per_s=result.filtered_rate_per_s;
}
} // namespace continuation_test
