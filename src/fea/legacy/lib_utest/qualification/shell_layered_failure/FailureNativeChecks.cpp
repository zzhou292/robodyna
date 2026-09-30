#include "FailureNativeFixture.h"
#include <gtest/gtest.h>
#include <cmath>

namespace layered_failure_test {
namespace {
void Close(double actual,double expected,double absolute) {
  ASSERT_TRUE(std::isfinite(actual));ASSERT_TRUE(std::isfinite(expected));
  EXPECT_NEAR(actual,expected,absolute+2.e-11*std::max(std::abs(actual),std::abs(expected)));
}
}
void Compare(const sec::ShellLayeredJ2FailureResult& actual,const WorkHistory& work,
    const NativeState& native,const NativeTrace& trace,double thickness,double area) {
  EXPECT_EQ(actual.history.element_active,native.parent==1);
  EXPECT_EQ(actual.removed_now,trace.removed==1);
  double mean_tangent=0,mean_yield=0,min_tangent=1;
  for(unsigned p=0;p<3;++p) {
    SCOPED_TRACE(p);
    for(unsigned c=0;c<5;++c) {
      Close(actual.history.saved.point[p].stress[c],native.points[7*p+c],1.e-8);
      Close(actual.history.current_force_point[p].stress[c],trace.point_values[13*p+c],1.e-8);
      Close(actual.current.history.point[p].stress[c],trace.point_values[13*p+c],1.e-8);
    }
    Close(actual.history.saved.point[p].plastic_strain,native.points[7*p+5],2.e-14);
    Close(actual.history.saved.point[p].filtered_rate_per_s,native.points[7*p+6],1.e-10);
    Close(actual.constitutive_increment[p],trace.point_values[13*p+6],2.e-14);
    Close(actual.history.failure[p].damage,native.failures[3*p],2.e-14);
    EXPECT_EQ(actual.history.failure[p].failure_time_s,native.failures[3*p+1]);
    EXPECT_EQ(actual.history.failure[p].point_active,native.failures[3*p+2]==1);
    const double weight=p==1?.5:.25;
    mean_tangent+=weight*trace.point_values[13*p+7];
    min_tangent=std::min(min_tangent,trace.point_values[13*p+7]);
    mean_yield+=weight*trace.point_values[13*p+9];
  }
  for(unsigned c=0;c<5;++c) {
    Close(actual.current.material_stress[c],native.material[c],1.e-8);
    Close(work.stress[c],native.stress[c],1.e-8);
  }
  for(unsigned c=0;c<3;++c) Close(actual.current.bending_stress[c],native.moment[c],1.e-8);
  Close(actual.current.reported_thickness,native.thickness,2.e-14);
  for(unsigned c=0;c<2;++c) Close(work.internal_work[c],native.work[c],1.e-12);
  const auto& d=actual.current.diagnostics;
  Close(d.plastic_work_density_increment*thickness*area,trace.diagnostics[0],1.e-12);
  Close(d.mean_plastic_strain,trace.diagnostics[1],2.e-14);
  Close(d.maximum_plastic_strain,trace.diagnostics[2],2.e-14);
  Close(d.mean_tangent_ratio,trace.diagnostics[3],2.e-14);
  Close(d.minimum_tangent_ratio,trace.diagnostics[4],2.e-14);
  Close(d.mean_yield_before_pa,trace.diagnostics[5],1.e-8);
  Close(d.last_point_yield_before_pa,trace.diagnostics[6],1.e-8);
  Close(d.mean_tangent_ratio,mean_tangent,2.e-14);
  Close(d.minimum_tangent_ratio,min_tangent,2.e-14);
  Close(d.mean_yield_before_pa,mean_yield,1.e-8);
}
} // namespace layered_failure_test
