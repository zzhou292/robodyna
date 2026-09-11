#pragma once
#include "native/NativePacket.h"
#include "../shell_layered_failure/FailureNativeFixture.h"
#include "../qeph/QephForceFixture.h"
#include "../t3/T3ForcePortFixture.h"
#include "../native/qeph/NativeQephBridge.h"
#include "../native/t3/T3EngineContext.h"
#include "FailureForceFields.h"

namespace failure_force_test {
inline std::vector<double> Diagnostics(const q::ForceDiagnostics& d) {
  return {d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,d.stabilization_viscosity,
    d.translational_stiffness,d.rotational_stiffness,d.unscaled_element_dt,
    d.internal_work_increment[0],d.internal_work_increment[1],d.hourglass_viscous_work_increment};
}
inline std::vector<double> Diagnostics(const t::ForceDiagnostics& d) {
  const auto a=t3_force_port_test::Diagnostics(d);return {a.begin(),a.end()};
}
// Existing independent native force comparison tolerance; no failure-specific
// widening. Exact OFF/zero-stiffness and source TT checks are separate below.
inline void ArrayAgreement(const std::vector<double>& values,const double* expected,double absolute=2.e-11) {
  for(unsigned i=0;i<values.size();++i) {SCOPED_TRACE(i);qeph_force_port_test::Independent(values[i],expected[i],absolute);}
}
inline void Geometry(const q::ForceTrial& force,const native::Packet& n,const q::PrescribedInterval& in) {
  native::nq::Kinematics k;
  ASSERT_EQ(native::nq::detail::UnpackKinematics(n.force.data(),n.planar,qeph_kinematics_test::NativeInterval(in),k),native::nq::Status::kSuccess);
  qeph_kinematics_test::Agreement(force.kinematics,k,in,true);
}
inline void Geometry(const t::ForceTrial& force,const native::Packet& n,const t::PrescribedInterval& in) {
  native::nt::Kinematics k;std::array<double,38> values;std::copy_n(n.force.begin(),38,values.begin());
  ASSERT_EQ(native::nt::detail::UnpackKinematics(values,t3_port_test::Native(in),k),native::nt::Status::kSuccess);
  t3_port_test::RatesAgreement(force.kinematics,k,in);
}
template<class F,class I> void Agreement(const typename F::Trial& trial,const native::Packet& n,const I& interval,double thickness) {
  constexpr bool quad=std::is_same_v<F,Q>;
  const auto& force=trial.force;const auto& h=force.proposed_history.data();
  Geometry(force,n,interval);
  EXPECT_TRUE(force.proposed_history.prepared());
  EXPECT_EQ(force.proposed_history.stamp().time,interval.base_time+interval.dt);
  EXPECT_EQ(force.proposed_history.stamp().sample_index,interval.sample_index);
  const auto values=native::History(h);ArrayAgreement(values,n.history.data());
  std::vector<double> loads;Append(loads,force.internal_force);Append(loads,force.internal_couple);
  ArrayAgreement(loads,n.force.data()+(quad?118:64));
  ArrayAgreement(Diagnostics(force.diagnostics),n.force.data()+(quad?142:82));
  layered_failure_test::NativeState state;state.points=n.points;state.failures=n.failures;
  std::copy_n(n.history.data(),5,state.stress.begin());std::copy_n(n.history.data()+5,5,state.material.begin());
  std::copy_n(n.history.data()+10,3,state.moment.begin());state.thickness=n.history[quad?33:21];
  std::copy_n(n.history.data()+(quad?34:22),2,state.work.begin());state.parent=n.history[quad?37:25];
  layered_failure_test::NativeTrace trace{n.point_values,n.diagnostics,n.removed};
  layered_failure_test::WorkHistory work;
  std::copy_n(h.stress,5,work.stress);std::copy_n(h.internal_work,2,work.internal_work);
  layered_failure_test::Compare(trial.section,work,state,trace,thickness,force.kinematics.area);
  if(!trial.section.history.element_active) {
    EXPECT_EQ(h.active,0);EXPECT_EQ(force.diagnostics.translational_stiffness,0);
    EXPECT_EQ(force.diagnostics.rotational_stiffness,0);EXPECT_GT(force.diagnostics.unscaled_element_dt,0);
    for(unsigned i=0;i<5;++i) {EXPECT_EQ(h.stress[i],0);EXPECT_EQ(std::signbit(h.stress[i]),std::signbit(n.history[i]));}
    for(double value:loads)EXPECT_EQ(value,0);
  }
}
} // namespace failure_force_test
