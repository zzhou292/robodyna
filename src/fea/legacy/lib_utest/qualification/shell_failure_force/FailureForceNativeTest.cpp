#include "NativeAgreement.h"
#include "../shell_hardening_continuation/ContinuationFailureFixture.h"

namespace failure_force_test {
template<class F> void Recurrence(bool analytic,bool rate,const continuation_test::OriginalCurve* curve=nullptr) {
  using N=std::conditional_t<std::is_same_v<F,Q>,native::nq::Reference,native::nt::Reference>;
  bool saw_partial=false,saw_removal=false,saw_continued_history=false;
  for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(mask);Fixture<F> f(mask);f.material=layered_failure_test::Parameters(analytic,rate);
    if(curve) {
      f.material=continuation_test::Prepare(*curve,rate);f.failure.failure_strain=1;
      for(unsigned p=0;p<3;++p) {
        f.accepted.section.saved.point[p].plastic_strain=curve->x[curve->count-1]+.01;
        if(f.accepted.section.failure[p].point_active)f.accepted.section.failure[p].damage=.9995;
      }
    }
    const auto reference=native::Reference<N>(f.reference.input);auto n=native::Seed(f.accepted);
    unsigned post_removal=0;
    for(unsigned step=0;step<32&&post_removal<2;++step) {
      SCOPED_TRACE(step);const auto before=f.accepted;const auto native_before=n;
      typename F::Trial current;ASSERT_EQ(f.Evaluate(step,current),F::Status::kSuccess);
      native::Advance(reference,Interval(f.reference,step),f.material,f.failure.failure_strain,n);
      Agreement<F>(current,n,Interval(f.reference,step),before.shell.data().thickness);
      EXPECT_TRUE(current.force.proposed_history.matches_reference(f.reference));
      for(unsigned p=0;p<3;++p) {
        EXPECT_DOUBLE_EQ(current.section.caller_failure_increment[p],
            current.section.history.saved.point[p].plastic_strain-before.section.saved.point[p].plastic_strain);
        if(!before.section.failure[p].point_active)EXPECT_EQ(current.section.history.failure[p].failure_time_s,before.section.failure[p].failure_time_s);
        else if(!current.section.history.failure[p].point_active)EXPECT_EQ(current.section.history.failure[p].failure_time_s,(step+1)*Dt);
        qeph_force_port_test::Independent(current.section.caller_failure_increment[p],n.points[7*p+5]-native_before.points[7*p+5],2.e-14);
      }
      unsigned inactive=0;for(const auto& point:current.section.history.failure)inactive+=!point.point_active;
      saw_partial|=(mask>0&&mask<7);saw_removal|=current.section.removed_now;
      if(!before.section.element_active) {
        ++post_removal;EXPECT_FALSE(current.section.removed_now);
        Exact({before.shell.data().internal_work[0],before.shell.data().internal_work[1]},
              {current.force.proposed_history.data().internal_work[0],current.force.proposed_history.data().internal_work[1]});
        if constexpr(std::is_same_v<F,Q>) {
          saw_continued_history|=before.shell.data().stabilization[0]!=current.force.proposed_history.data().stabilization[0];
          EXPECT_EQ(before.shell.data().hourglass_viscous_work,current.force.proposed_history.data().hourglass_viscous_work);
        } else saw_continued_history|=current.force.proposed_history.data().strain_curvature[0]!=before.shell.data().strain_curvature[0];
      }
      EXPECT_EQ(current.section.history.element_active,inactive<3);f.Accept(current);
    }
    EXPECT_EQ(post_removal,2);
  }
  EXPECT_TRUE(saw_partial);EXPECT_TRUE(saw_removal);EXPECT_TRUE(saw_continued_history);
}
TEST(ShellFailureForceNative,QephTabulatedMasksRemovalAndTwoPostRemovalIntervals) {Recurrence<Q>(false,false);Recurrence<Q>(false,true);}
TEST(ShellFailureForceNative,QephAnalyticMasksRemovalAndTwoPostRemovalIntervals) {Recurrence<Q>(true,true);}
TEST(ShellFailureForceNative,T3TabulatedMasksRemovalAndTwoPostRemovalIntervals) {Recurrence<T>(false,false);Recurrence<T>(false,true);}
TEST(ShellFailureForceNative,T3AnalyticMasksRemovalAndTwoPostRemovalIntervals) {Recurrence<T>(true,true);}
TEST(ShellFailureForceNative,QephOriginalFailureCurvesRetainNativeContinuation) {
  for(const auto& curve:continuation_test::Curves)for(bool rate:{false,true})Recurrence<Q>(false,rate,&curve);
}
TEST(ShellFailureForceNative,T3OriginalFailureCurvesRetainNativeContinuation) {
  for(const auto& curve:continuation_test::Curves)for(bool rate:{false,true})Recurrence<T>(false,rate,&curve);
}
} // namespace failure_force_test
