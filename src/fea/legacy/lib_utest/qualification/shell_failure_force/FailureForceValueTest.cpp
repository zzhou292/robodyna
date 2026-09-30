#include "FailureForceFields.h"
#include <limits>

namespace failure_force_test {
template<class F> void LegacyAndInactive() {
  Fixture<F> f;f.failure.failure_strain=100;
  typename F::LegacyHistory legacy;
  ASSERT_EQ(InitializeLayeredJ2History(f.reference,f.material,{},legacy),F::Status::kSuccess);
  for(unsigned step=0;step<5;++step) {
    typename F::Trial current;typename F::LegacyTrial prior;
    ASSERT_EQ(f.Evaluate(step,current),F::Status::kSuccess);
    ASSERT_EQ(EvaluateLayeredJ2Force(f.reference,f.material,legacy,Interval(f.reference,step),prior),F::Status::kSuccess);
    Exact(ForceValues(current.force),ForceValues(prior.force));
    std::vector<double> a,b;Append(a,current.force.internal_force);Append(a,current.force.internal_couple);
    Append(b,prior.force.internal_force);Append(b,prior.force.internal_couple);Exact(a,b);
    f.Accept(current);legacy={prior.force.proposed_history,prior.proposed_section};
  }
  Fixture<F> removed(7);typename F::Trial out;
  for(unsigned step=0;step<2;++step) {
    ASSERT_EQ(removed.Evaluate(step,out),F::Status::kSuccess);const auto& h=out.force.proposed_history.data();
    EXPECT_EQ(h.active,0);EXPECT_EQ(out.force.diagnostics.translational_stiffness,0);
    EXPECT_EQ(out.force.diagnostics.rotational_stiffness,0);EXPECT_GT(out.force.diagnostics.unscaled_element_dt,0);
    EXPECT_NE(h.strain_curvature[0],removed.accepted.shell.data().strain_curvature[0]);
    EXPECT_NE(out.section.history.current_force_point[0].stress[0],0);
    EXPECT_EQ(out.section.history.saved.point[0].stress[0],0);
    EXPECT_EQ(PreparePrescribedHistory(removed.reference,h,out.force.proposed_history.stamp(),removed.accepted.shell),F::Status::kInvalidInput);
    typename F::LegacyHistory forbidden{out.force.proposed_history,out.section.history.saved};typename F::LegacyTrial rejected;
    EXPECT_EQ(EvaluateLayeredJ2Force(removed.reference,removed.material,forbidden,Interval(removed.reference,step+1),rejected),F::Status::kInvalidInput);
    removed.Accept(out);
  }
}
TEST(ShellFailureForceValues,QephLegacyBitsAndInactiveNativeChannels) {LegacyAndInactive<Q>();}
TEST(ShellFailureForceValues,T3LegacyBitsAndInactiveNativeChannels) {LegacyAndInactive<T>();}

template<class F> void Rejection() {
  Fixture<F> f;typename F::Trial output;
  ASSERT_EQ(f.Evaluate(0,output),F::Status::kSuccess);f.Accept(output);
  const auto base=Values(f.accepted.shell.data());
  // Snapshot the same target object, so unspecified padding is not an oracle.
  std::vector<unsigned char> bytes(sizeof(output));std::memcpy(bytes.data(),&output,sizeof(output));
  auto bad=f.accepted;bad.section.saved.point[2].plastic_strain=3;
  EXPECT_NE(EvaluateLayeredJ2FailureForce(f.reference,f.material,f.failure,bad,Interval(f.reference,1),output),F::Status::kSuccess);
  EXPECT_EQ(std::memcmp(bytes.data(),&output,sizeof(output)),0);Exact(base,Values(f.accepted.shell.data()));
  typename F::Trial expected;ASSERT_EQ(f.Evaluate(1,expected),F::Status::kSuccess);
  ASSERT_EQ(f.Evaluate(1,output),F::Status::kSuccess);
  Exact(ForceValues(expected.force),ForceValues(output.force));
  Exact(Values(expected.section),Values(output.section));
  auto values=f.accepted.shell.data();values.active=.8;
  EXPECT_EQ(PrepareFailurePrescribedHistory(f.reference,values,f.accepted.shell.stamp(),f.accepted.shell),F::Status::kInvalidInput);
  Exact(base,Values(f.accepted.shell.data()));
  Fixture<F> inactive(7);auto interval=Interval(inactive.reference,0);
  if constexpr(std::is_same_v<F,Q>) for(auto& x:interval.position_endpoint)x={};
  else for(auto& x:interval.position)x={};
  EXPECT_NE(EvaluateLayeredJ2FailureForce(inactive.reference,inactive.material,inactive.failure,inactive.accepted,interval,output),F::Status::kSuccess);
}
TEST(ShellFailureForceValues,QephLateFailureRollbackRetryAndGeometryDomain) {Rejection<Q>();}
TEST(ShellFailureForceValues,T3LateFailureRollbackRetryAndGeometryDomain) {Rejection<T>();}
} // namespace failure_force_test
