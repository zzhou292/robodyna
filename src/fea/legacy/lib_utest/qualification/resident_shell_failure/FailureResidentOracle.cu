#include "FailureResidentFixture.h"
namespace resident_failure_test {
namespace {
qe::PrescribedInterval QuadInterval(const Rig& rig,const Prepared& prepared,unsigned parent) {
  auto interval=mixed::QephInterval(rig,prepared);
  for(unsigned i=0;i<4;++i) {
    const auto n=rig.binding.qeph_nodes(parent)[i];
    interval.position_endpoint[i]={prepared.endpoint.x[3*n],prepared.endpoint.x[3*n+1],prepared.endpoint.x[3*n+2]};
    interval.velocity_midpoint[i]={prepared.endpoint.v[3*n],prepared.endpoint.v[3*n+1],prepared.endpoint.v[3*n+2]};
    interval.omega_midpoint[i]={prepared.endpoint.omega[3*n],prepared.endpoint.omega[3*n+1],prepared.endpoint.omega[3*n+2]};
  }
  return interval;
}
tr::PrescribedInterval TriangleInterval(const Rig& rig,const Prepared& prepared,unsigned parent) {
  auto interval=mixed::T3Interval(rig,prepared);
  for(unsigned i=0;i<3;++i) {
    const auto n=rig.binding.t3_nodes(parent)[i];
    interval.position[i]={prepared.endpoint.x[3*n],prepared.endpoint.x[3*n+1],prepared.endpoint.x[3*n+2]};
    interval.velocity[i]={prepared.endpoint.v[3*n],prepared.endpoint.v[3*n+1],prepared.endpoint.v[3*n+2]};
    interval.angular_velocity[i]={prepared.endpoint.omega[3*n],prepared.endpoint.omega[3*n+1],prepared.endpoint.omega[3*n+2]};
  }
  return interval;
}
void Agreement(const std::vector<double>& actual,const std::vector<double>& expected) {
  ASSERT_EQ(actual.size(),expected.size());
  for(unsigned i=0;i<actual.size();++i) {
    ASSERT_TRUE(std::isfinite(actual[i]));
    ASSERT_TRUE(std::isfinite(expected[i]));
    EXPECT_LE(std::abs(actual[i]-expected[i]),2e-12*std::max({1.,std::abs(actual[i]),std::abs(expected[i])}))<<i;
  }
}
template<class Trial,class OldForce> void Check(const Trial& expected,const OldForce& actual,
    const fe::ShellBatchSectionState& old,const fe::ShellBatchSectionState& next,
    const fe::ShellBatchFailureState& failure,double thickness) {
  Agreement(failure_force_test::ForceValues(actual),failure_force_test::ForceValues(expected.force));
  Agreement(failure_force_test::Diagnostics(actual.diagnostics),failure_force_test::Diagnostics(expected.force.diagnostics));
  std::vector<double> values,oracle;
  failure_force_test::Append(values,next.history);
  failure_force_test::Append(oracle,expected.section.history.saved);
  failure_force_test::Append(values,next.diagnostics);
  failure_force_test::Append(oracle,expected.section.current.diagnostics);
  Agreement(values,oracle);
  const auto work=old.cumulative_plastic_work_J+
    expected.section.current.diagnostics.plastic_work_density_increment*thickness*expected.force.kinematics.area;
  Agreement({next.cumulative_plastic_work_J},{work});
  EXPECT_EQ(failure.active,expected.section.history.element_active);
  for(unsigned p=0;p<3;++p) {
    EXPECT_EQ(failure.constant_points()[p].point_active,expected.section.history.failure[p].point_active);
    Agreement({failure.constant_points()[p].damage,failure.constant_points()[p].failure_time_s},
      {expected.section.history.failure[p].damage,expected.section.history.failure[p].failure_time_s});
    values.clear();oracle.clear();
    failure_force_test::Append(values,failure.current_force_point[p].stress);
    failure_force_test::Append(oracle,expected.section.history.current_force_point[p].stress);
    Agreement(values,oracle);
  }
}
}
void CheckForceAdapters(const Rig& rig,const fe::ShellBatchPlasticityBinding& catalog,
    const Prepared& prepared,const Frame& old,const Frame& actual) {
  fe::sections::PointParameters qp,tp;
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qeph,1,&qp));
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::T3,1,&tp));
  const auto& qs=*old.material.qsection[1].plastic();
  const auto& ts=*old.material.tsection[1].plastic();
  qe::LayeredJ2FailureForceTrial qtrial;
  tr::LayeredJ2FailureForceTrial ttrial;
  // Call qualified value contracts directly, never the resident dispatcher.
  ASSERT_EQ(qe::EvaluateLayeredJ2FailureForce(rig.binding.qeph_reference(1),qp,{1e-6},
    {old.material.qforce[1].proposed_history,storage::FailureHistory(qs,old.qfailure[1])},
    QuadInterval(rig,prepared,1),qtrial),qe::Status::kSuccess);
  ASSERT_EQ(tr::EvaluateLayeredJ2FailureForce(rig.binding.t3_reference(1),tp,{1e-6},
    {old.material.tforce[1].proposed_history,storage::FailureHistory(ts,old.tfailure[1])},
    TriangleInterval(rig,prepared,1),ttrial),tr::Status::kSuccess);
  Check(qtrial,actual.material.qforce[1],qs,*actual.material.qsection[1].plastic(),actual.qfailure[1],old.material.qforce[1].proposed_history.data().thickness);
  Check(ttrial,actual.material.tforce[1],ts,*actual.material.tsection[1].plastic(),actual.tfailure[1],old.material.tforce[1].proposed_history.data().thickness);
}
} // namespace resident_failure_test
