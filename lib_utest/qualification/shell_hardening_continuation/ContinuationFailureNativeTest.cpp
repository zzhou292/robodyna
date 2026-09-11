#include "ContinuationFailureNative.h"
#include <gtest/gtest.h>

namespace continuation_test {
TEST(HardeningContinuationNative, CompleteFailOneCallerCrossesOriginalTablesAndAllPointMasks) {
  for(const auto c:Curves) for(bool rate:{false,true}) for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(c.id);
    SCOPED_TRACE(rate);
    SCOPED_TRACE(mask);
    const bool virgin=mask==0;
    const auto expected=NativeFailureTrajectory(c,rate,mask,virgin);
    const auto p=Prepare(c,rate);
    auto input=FailureInput(p);
    auto history=FailureSeed(c,mask,virgin);
    failure::WorkHistory work;
    std::vector<FailureRecord> actual;
    unsigned post=0;
    for(unsigned step=0;step<MaximumFailureSteps;++step) {
      FailureRecord record;
      record.reference_thickness=input.reference_thickness;
      ASSERT_LT(step,expected.size());
      record.status=sec::UpdateShellLayeredJ2Failure(p,{1.},history,input,(step+2)*input.dt,record.section);
      ASSERT_EQ(record.status,Status::Ok);
      record.work_valid=sec::ApplyLayeredJ2FailureWork(record.section,input.strain_curvature_increment,
          input.reference_thickness,.01,expected[step].trace.diagnostics[8],work);
      record.work=work;
      actual.push_back(record);
      history=record.section.history;
      input.reference_thickness=input.reported_thickness=record.section.current.reported_thickness;
      if(!history.element_active && !record.section.removed_now) ++post;
      if(post==2) break;
    }
    CompareFailureTrajectory(c,actual,expected,mask!=7);
  }
}
} // namespace continuation_test
