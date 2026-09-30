#include "Tab1ForceFixture.h"
#include <limits>

namespace tab1_force_test {
template<class F> void Removal() {
  for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(mask);
    Fixture<F> f(mask);
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32&&post<2;++step) {
      const auto before=f.accepted;
      typename F::Trial trial;
      ASSERT_EQ(f.Evaluate(step,trial),F::Status::kSuccess);
      const auto& h=trial.force.proposed_history;
      EXPECT_TRUE(h.matches_reference(f.reference));
      EXPECT_EQ(h.stamp().sample_index,step+1);
      EXPECT_EQ(h.stamp().time,(step+1)*Dt);
      if(trial.section.removed_now) {
        ++removed;
        if(mask) EXPECT_EQ(step,0u);
      }
      if(!before.section.element_active) {
        ++post;
        EXPECT_EQ(h.data().active,0);
        EXPECT_EQ(trial.force.diagnostics.translational_stiffness,0);
        EXPECT_EQ(trial.force.diagnostics.rotational_stiffness,0);
        EXPECT_GT(trial.force.diagnostics.unscaled_element_dt,0);
        for(unsigned i=0;i<3;++i) {
          tab1_test::SameFailureHistory(before.section.failure[i],trial.section.history.failure[i]);
          EXPECT_NE(before.section.saved.point[i].filtered_rate_per_s,
              trial.section.history.saved.point[i].filtered_rate_per_s);
        }
        std::vector<double> loads;
        Append(loads,trial.force.internal_force);
        Append(loads,trial.force.internal_couple);
        for(double value:loads) EXPECT_EQ(value,0);
      }
      f.Accept(trial);
    }
    EXPECT_EQ(removed,1u);
    EXPECT_EQ(post,2u);
  }
}
TEST(Tab1ForceValues,QephOriginalMaterialAnyPointRemovalAndPostRemovalHistory) {Removal<Q>();}
TEST(Tab1ForceValues,T3OriginalMaterialAnyPointRemovalAndPostRemovalHistory) {Removal<T>();}

template<class F> void Rejection() {
  Fixture<F> f;
  typename F::History startup;
  const auto initial=tab1_test::Bytes(startup);
  auto wrong=f.material;
  ASSERT_EQ(tl::material::PrepareLinearLaw44ShellPlasticity(70e9,.22,2500,{30e6,1e9},
      {true,8000,8,10000},wrong),sec::PointStatus::Ok);
  EXPECT_EQ(InitializeLayeredTab1History(f.reference,wrong,f.failure,{},startup),F::Status::kInvalidInput);
  EXPECT_EQ(tab1_test::Bytes(startup),initial);
  typename F::Trial output;
  ASSERT_EQ(f.Evaluate(0,output),F::Status::kSuccess);
  f.Accept(output);
  const auto bytes=tab1_test::Bytes(output);
  const auto accepted=tab1_test::Bytes(f.accepted);
  auto bad=f.accepted;
  bad.section.saved.point[2].filtered_rate_per_s=std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(EvaluateLayeredTab1Force(f.reference,f.material,f.failure,bad,
      Interval(f.reference,1),output),F::Status::kSuccess);
  EXPECT_EQ(tab1_test::Bytes(output),bytes);
  EXPECT_EQ(tab1_test::Bytes(f.accepted),accepted);
  typename F::Trial control;
  ASSERT_EQ(f.Evaluate(1,control),F::Status::kSuccess);
  ASSERT_EQ(f.Evaluate(1,output),F::Status::kSuccess);
  Exact(ForceValues(output.force),ForceValues(control.force));
  Exact(SectionValues(output.section),SectionValues(control.section));
  Fixture<F> removed(7);
  ASSERT_EQ(removed.Evaluate(0,output),F::Status::kSuccess);
  removed.Accept(output);
  auto interval=Interval(removed.reference,1);
  if constexpr(std::is_same_v<F,Q>) for(auto& x:interval.position_endpoint) x={};
  else for(auto& x:interval.position) x={};
  const auto saved=tab1_test::Bytes(output);
  EXPECT_NE(EvaluateLayeredTab1Force(removed.reference,removed.material,removed.failure,
      removed.accepted,interval,output),F::Status::kSuccess);
  EXPECT_EQ(tab1_test::Bytes(output),saved);
}
TEST(Tab1ForceValues,QephWrongCompositionLateFailureAndDegenerateInactiveGeometryRejectAtomically) {Rejection<Q>();}
TEST(Tab1ForceValues,T3WrongCompositionLateFailureAndDegenerateInactiveGeometryRejectAtomically) {Rejection<T>();}
} // namespace tab1_force_test
