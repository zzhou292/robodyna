#pragma once
#include "Tab1Fixture.h"
#include "lib_utest/qualification/shell_layered_failure/FailureNativeFixture.h"
#include <algorithm>
#include <cmath>
namespace tab1_test {
using NativeTrace=layered_failure_test::NativeTrace;
struct NativeState {
  std::array<double,21> points{};
  std::array<double,15> failures{};
  std::array<double,5> material{},stress{};
  std::array<double,3> moment{};
  std::array<double,2> work{};
  double parent=1.,thickness=.003;
};
extern "C" void tab1_native_defaults(double*,double*);
extern "C" void tab1_native_table(const double*,double,double,int,double*,double*,int*);
extern "C" void tab1_native_point(const double*,double,const double*,int,double,double,const double*,double*);
inline std::array<double,5> NativeHistory(const fail::Tab1ConstantFailureHistory& h) {
  return {h.damage,h.failure_time_s,h.point_active?1.:0.,h.maximum_damage,static_cast<double>(h.table_segment)};
}
inline NativeState NativeSeed(const sec::ShellLayeredTab1History& h) {
  NativeState n;
  n.parent=h.element_active?1.:0.;
  for(unsigned p=0;p<3;++p) {
    std::copy_n(h.saved.point[p].stress,5,n.points.begin()+7*p);
    n.points[7*p+5]=h.saved.point[p].plastic_strain;
    n.points[7*p+6]=h.saved.point[p].filtered_rate_per_s;
    const auto f=NativeHistory(h.failure[p]);
    std::copy(f.begin(),f.end(),n.failures.begin()+5*p);
  }
  return n;
}
void NativeStep(const sec::ShellLayeredJ2Input&,double,double,double,NativeState&,NativeTrace&);
inline void Close(double a,double b,double absolute=2.e-14) {
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b));
  EXPECT_NEAR(a,b,absolute+2.e-11*std::max(std::abs(a),std::abs(b)));
}
void Compare(const sec::ShellLayeredTab1Result&,const Work&,const NativeState&,
    const NativeTrace&,double,double);
} // namespace tab1_test
