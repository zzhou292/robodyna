#pragma once
#include "native/NativePacket.h"
#include "../shell_failure_force/NativeAgreement.h"
#include "../shell_tab1_glass/Tab1NativeSupport.h"

namespace tab1_force_test {
using failure_force_test::Diagnostics;
using failure_force_test::ArrayAgreement;
template<class F,class IntervalType>
void Agreement(const typename F::Trial& trial,const native::Packet& n,
    const IntervalType& interval,double thickness) {
  constexpr bool quad=std::is_same_v<F,Q>;
  const auto& force=trial.force;
  const auto& h=force.proposed_history.data();
  // Reuse the independent native geometry unpack/comparison. Its old failure
  // state is not read; TAB1 has a separate complete five-field point packet.
  failure_force_test::native::Packet geometry;
  geometry.force=n.force;
  geometry.planar=n.planar;
  failure_force_test::Geometry(force,geometry,interval);
  EXPECT_TRUE(force.proposed_history.prepared());
  EXPECT_EQ(force.proposed_history.stamp().time,interval.base_time+interval.dt);
  EXPECT_EQ(force.proposed_history.stamp().sample_index,interval.sample_index);
  ArrayAgreement(native::History(h),n.history.data());
  std::vector<double> loads;
  Append(loads,force.internal_force);
  Append(loads,force.internal_couple);
  ArrayAgreement(loads,n.force.data()+(quad?118:64));
  ArrayAgreement(Diagnostics(force.diagnostics),n.force.data()+(quad?142:82));

  tab1_test::NativeState state;
  state.points=n.points;
  state.failures=n.failures;
  std::copy_n(n.history.data(),5,state.stress.begin());
  std::copy_n(n.history.data()+5,5,state.material.begin());
  std::copy_n(n.history.data()+10,3,state.moment.begin());
  state.thickness=n.history[quad?33:21];
  std::copy_n(n.history.data()+(quad?34:22),2,state.work.begin());
  state.parent=n.history[quad?37:25];
  tab1_test::NativeTrace trace{n.point_values,n.diagnostics,n.removed};
  tab1_test::Work work;
  std::copy_n(h.stress,5,work.stress);
  std::copy_n(h.material_stress,5,work.material_stress);
  std::copy_n(h.bending_stress,3,work.bending_stress);
  std::copy_n(h.internal_work,2,work.internal_work);
  work.thickness=h.thickness;
  tab1_test::Compare(trial.section,work,state,trace,thickness,force.kinematics.area);

  // Actual native family coefficient, independently returned by its complete
  // CNCOEF3B/C3COEF3 path, must feed the section viscosity even after removal.
  const unsigned index=quad?142:82;
  const double sound_speed=n.force[index+1],dm=n.force[index+2];
  EXPECT_DOUBLE_EQ(sound_speed,std::sqrt(70e9/2500.));
  const double onep414=((1.+4./10.)+1./100.)+4./1000.;
  const double dtinv=interval.dt/std::max(interval.dt*interval.dt,1./1e20);
  const double viscosity=(onep414*dm)*sound_speed*std::sqrt(force.kinematics.area)*dtinv*2500.;
  qeph_force_port_test::Independent(n.diagnostics[8],viscosity,2.e-11);
  if(!trial.section.history.element_active) {
    EXPECT_EQ(h.active,0);
    EXPECT_EQ(force.diagnostics.translational_stiffness,0);
    EXPECT_EQ(force.diagnostics.rotational_stiffness,0);
    EXPECT_GT(force.diagnostics.unscaled_element_dt,0);
    for(unsigned i=0;i<5;++i) {
      EXPECT_EQ(h.stress[i],0);
      EXPECT_EQ(std::signbit(h.stress[i]),std::signbit(n.history[i]));
    }
    for(double value:loads) EXPECT_EQ(value,0);
  }
}
} // namespace tab1_force_test
