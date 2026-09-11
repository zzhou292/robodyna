#include "ContinuationFailureFixture.h"
#include <gtest/gtest.h>

namespace continuation_test {
TEST(HardeningContinuation, OriginalFailOnePassesEndpointsPartialRemovalAndInactiveRecurrence) {
  for(const auto c:Curves) for(bool rate:{false,true}) {
    SCOPED_TRACE(c.id);
    SCOPED_TRACE(rate);
    auto p=Prepare(c,rate); auto input=FailureInput(p);
    auto state=FailureSeed(c,0,true);
    failure::WorkHistory work;
    unsigned removed=0,post=0,partial=0,crossed=0;
    for(unsigned step=0;step<MaximumFailureSteps;++step) {
      sec::ShellLayeredJ2FailureResult result;
      ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.},state,input,(step+2)*input.dt,result),Status::Ok);
      ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(result,input.strain_curvature_increment,
          input.reference_thickness,.01,0,work));
      crossed+=result.current.diagnostics.maximum_plastic_strain>c.x[c.count-1];
      if(result.removed_now) ++removed;
      else if(!result.history.element_active) ++post;
      else if(sec::ShellNip3FailedThickness(result.history.failure)>0) ++partial;
      state=result.history;
      input.reference_thickness=input.reported_thickness=result.current.reported_thickness;
      if(post==2) break;
    }
    EXPECT_EQ(removed,1u); EXPECT_EQ(post,2u);
    EXPECT_GT(crossed,1u); EXPECT_GT(partial,0u);
  }
}
} // namespace continuation_test
