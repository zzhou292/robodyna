#include "NativeAgreement.h"

namespace tab1_force_test {
template<class F> void NativeRecurrence() {
  using Reference=std::conditional_t<std::is_same_v<F,Q>,native::nq::Reference,native::nt::Reference>;
  for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(mask);
    Fixture<F> f(mask);
    const auto reference=native::Reference<Reference>(f.reference.input);
    auto packet=native::Seed(f.accepted);
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32&&post<2;++step) {
      SCOPED_TRACE(step);
      const auto before=f.accepted;
      const auto old_native=packet;
      const auto interval=Interval(f.reference,step);
      typename F::Trial trial;
      ASSERT_EQ(f.Evaluate(step,trial),F::Status::kSuccess);
      native::Advance(reference,interval,f.material,f.failure,packet);
      Agreement<F>(trial,packet,interval,before.shell.data().thickness);
      removed+=trial.section.removed_now;
      for(unsigned p=0;p<3;++p) {
        EXPECT_DOUBLE_EQ(trial.section.caller_failure_increment[p],
            trial.section.history.saved.point[p].plastic_strain-before.section.saved.point[p].plastic_strain);
        qeph_force_port_test::Independent(trial.section.caller_failure_increment[p],
            packet.points[7*p+5]-old_native.points[7*p+5],2.e-14);
        if(before.section.failure[p].point_active&&!trial.section.history.failure[p].point_active) {
          EXPECT_EQ(trial.section.history.failure[p].failure_time_s,interval.base_time+interval.dt);
          EXPECT_GE(trial.section.history.failure[p].damage,1.);
          EXPECT_EQ(trial.section.history.failure[p].maximum_damage,1.);
        }
      }
      if(!before.section.element_active) {
        ++post;
        EXPECT_FALSE(trial.section.removed_now);
        for(unsigned p=0;p<3;++p)
          tab1_test::SameFailureHistory(before.section.failure[p],trial.section.history.failure[p]);
      }
      f.Accept(trial);
    }
    EXPECT_EQ(removed,1u);
    EXPECT_EQ(post,2u);
  }
}
TEST(Tab1ForceNative,QephOriginalGlassAllMasksNativeFullForceRemovalAndTwoLaterIntervals) {NativeRecurrence<Q>();}
TEST(Tab1ForceNative,T3OriginalGlassAllMasksNativeFullForceRemovalAndTwoLaterIntervals) {NativeRecurrence<T>();}
} // namespace tab1_force_test
