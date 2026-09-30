#include "ZeroCNativeSupport.h"

namespace zero_c_test {
TEST(FilteredZeroCNative, AnalyticAndTableLoadingUnloadingFilterSaturationAndInactiveParent) {
  for (bool analytic:{false,true}) {
    SCOPED_TRACE(analytic);
    const auto p=Prepare(analytic);
    History actual,expected;
    actual.filtered_rate_per_s=expected.filtered_rate_per_s=23.;
    double time=0;
    for (unsigned step=0;step<14;++step) {
      SCOPED_TRACE(step);
      const auto in=Increment(step);
      time+=in.dt;
      const auto native=NativeAdvance(analytic,expected,in,time);
      Result result;
      ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,actual,in,result),Status::Ok);
      Compare(result,native,expected);
      EXPECT_EQ(native.values[11],in.element_active?1.:0.);
      if (analytic) Near(p.plastic_hardening_pa,native.hardening);
      if (step==3) EXPECT_DOUBLE_EQ(native.values[12],in.total_strain_rate_per_s);
      actual=result.history;
      expected=NativeHistory(native);
    }
  }
}
TEST(FilteredZeroCNative, StrengtheningIsExactlyIndependentOfFiniteFilteredHistory) {
  for (bool analytic:{false,true}) {
    auto in=Increment(0);
    History before;
    const auto low=NativeAdvance(analytic,before,in,in.dt);
    before.filtered_rate_per_s=1.e200;
    in.total_strain_rate_per_s=1.e200;
    const auto high=NativeAdvance(analytic,before,in,in.dt);
    ASSERT_GT(high.values[12],1.e199);
    for (unsigned i=0;i<12;++i) EXPECT_DOUBLE_EQ(low.values[i],high.values[i]);
    auto p=Prepare(analytic);
    Result value;
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,before,in,value),Status::Ok);
    Compare(value,high,before);
  }
}
} // namespace zero_c_test
