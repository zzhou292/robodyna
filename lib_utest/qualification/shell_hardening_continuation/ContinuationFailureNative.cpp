#include "ContinuationFailureNative.h"
#include <gtest/gtest.h>

namespace continuation_test {
std::vector<NativeFailureRecord> NativeFailureTrajectory(OriginalCurve c,bool rate,unsigned mask,bool virgin) {
  const auto p=Prepare(c,rate);
  auto input=FailureInput(p);
  auto native=failure::NativeSeed(FailureSeed(c,mask,virgin));
  std::vector<NativeFailureRecord> result;
  unsigned post=0;
  for(unsigned step=0;step<MaximumFailureSteps;++step) {
    NativeFailureRecord record;
    record.reference_thickness=native.thickness;
    failure::NativeStep(p,1.,input,(step+2)*input.dt,.01,.015,native,record.trace);
    record.state=native;
    result.push_back(record);
    if(native.parent==0 && !record.trace.removed) ++post;
    if(post==2) break;
  }
  return result;
}
void CompareFailureTrajectory(OriginalCurve c,const std::vector<FailureRecord>& actual,
    const std::vector<NativeFailureRecord>& expected,bool initially_active) {
  ASSERT_EQ(actual.size(),expected.size());
  unsigned removed=0,post=0,crossed=0;
  for(std::size_t step=0;step<actual.size();++step) {
    SCOPED_TRACE(c.id);
    SCOPED_TRACE(step);
    const auto& a=actual[step]; const auto& e=expected[step];
    ASSERT_EQ(a.status,Status::Ok); ASSERT_TRUE(a.work_valid);
    EXPECT_NEAR(a.reference_thickness,e.reference_thickness,2.e-14);
    failure::Compare(a.section,a.work,e.state,e.trace,a.reference_thickness,.01);
    crossed+=a.section.current.diagnostics.maximum_plastic_strain>c.x[c.count-1];
    if(a.section.removed_now) ++removed;
    else if(!a.section.history.element_active) ++post;
  }
  EXPECT_EQ(removed,initially_active?1u:0u);
  EXPECT_EQ(post,2u); EXPECT_GT(crossed,0u);
}
} // namespace continuation_test
