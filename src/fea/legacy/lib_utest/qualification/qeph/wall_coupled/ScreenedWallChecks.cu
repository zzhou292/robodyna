#include "ScreenedWallFixture.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace qeph_wall_test {
namespace {
using Wide=long double;
constexpr Wide Roundoff=256*std::numeric_limits<double>::epsilon();
}
void CheckScreenedRigid(const WallRig& w,const Trial& t,const Staged& s,const LedgerScales& scales,
                        double& regular_maximum,double& hourglass_maximum) {
  const auto& r=w.shell;
  for(unsigned e=0;e<r.count;++e) {
    const auto& trial=s.elements[e]; const auto& input=r.element[e].reference.input;
    const auto interval=Interval(r,e,t.nodal.state,t.nodal.view.base_time,t.nodal.view.kinematics.base_epoch);
    const double length=qeph_kinematics_test::LengthScale(interval);
    for(unsigned i=0;i<8;++i) {
      const double budget=2e-12*screened_wall::ImpactSpeed/(i<5?Side:Side*Side);
      const double ratio=std::abs(trial.kinematics.regular_rate[i])/budget;
      EXPECT_LE(ratio,1); regular_maximum=std::max(regular_maximum,ratio);
    }
    for(unsigned i=0;i<6;++i) {
      const double budget=2e-12*screened_wall::ImpactSpeed/(i==2||i==3?Side:1.);
      const double ratio=std::abs(trial.kinematics.hourglass_rate[i])/budget;
      EXPECT_LE(ratio,1); hourglass_maximum=std::max(hourglass_maximum,ratio);
    }
    auto zero=qeph_force_port_test::Seed(input,false);
    zero.active=trial.proposed_history.data().active; // Native parity checks this flag separately.
    qeph_force_port_test::HistoryAgreement(trial.proposed_history.data(),zero,input,length);
    for(unsigned local=0;local<4;++local) {
      qeph_kinematics_test::Field(trial.internal_force[local],q::Vec3{},input.young_modulus*input.thickness*length,2e-12,"rigid_internal_force",local);
      qeph_kinematics_test::Field(trial.internal_couple[local],q::Vec3{},input.young_modulus*input.thickness*length*length,2e-12,"rigid_internal_couple",local);
    }
    const auto& h=trial.proposed_history.data();
    for(double value:{h.internal_work[0],h.internal_work[1],h.hourglass_viscous_work})
      EXPECT_LE(std::abs(static_cast<Wide>(value)),Roundoff*std::abs(static_cast<Wide>(value))+1e-12L*scales.energy);
  }
  for(unsigned n=0;n<r.n;++n) {
    EXPECT_NEAR(t.nodal.state.x[3*n],t.nodal.state.x[0],2e-12*Side);
    EXPECT_NEAR(t.nodal.state.v[3*n],t.nodal.state.v[0],VelocityBudget());
    for(unsigned a=1;a<3;++a) {
      EXPECT_NEAR(t.nodal.state.x[3*n+a],r.x[3*n+a],2e-12*Side);
      EXPECT_NEAR(t.nodal.state.v[3*n+a],0,VelocityBudget());
    }
    for(unsigned a=0;a<3;++a) EXPECT_NEAR(t.nodal.state.omega[3*n+a],0,SpinBudget());
  }
}
void CheckScreenedInitial(const WallRig& w,const LedgerScales& scales,const Snapshot& state,
                          const PortResults& cache,const q::BatchDiagnostics& d) {
  const auto& r=w.shell;
  EXPECT_EQ(state.stamp.epoch,0u); EXPECT_EQ(state.stamp.time,0);
  EXPECT_EQ(d.qualification_id,w.experiment.qualification); EXPECT_TRUE(d.kinetic_available);
  EXPECT_FALSE(d.has_completed_interval); EXPECT_EQ(d.kick_dt,0); EXPECT_EQ(d.velocity_time,0);
  const Wide budget=Roundoff*scales.energy+1e-12L*scales.energy;
  EXPECT_LE(std::abs(static_cast<Wide>(d.kinetic_translation)-scales.energy),budget);
  EXPECT_GT(d.kinetic_translation,0); EXPECT_EQ(d.kinetic_rotation,0);
  EXPECT_EQ(d.kinetic_physical_isotropic,0); EXPECT_EQ(d.kinetic_added_isotropic,0);
  for(unsigned n=0;n<r.n;++n) {
    EXPECT_EQ(state.x[3*n],-screened_wall::InitialGap); EXPECT_EQ(state.v[3*n],screened_wall::ImpactSpeed);
    for(unsigned a=0;a<3;++a) EXPECT_EQ(state.omega[3*n+a],0);
  }
  for(unsigned e=0;e<r.count;++e) {
    const auto& trial=cache[e]; const auto& h=trial.proposed_history.data();
    EXPECT_TRUE(trial.proposed_history.prepared()); EXPECT_EQ(trial.proposed_history.stamp().sample_index,0u);
    for(double x:h.stress) EXPECT_EQ(x,0);
    for(double x:h.material_stress) EXPECT_EQ(x,0);
    for(double x:h.bending_stress) EXPECT_EQ(x,0);
    for(double x:h.stabilization) EXPECT_EQ(x,0);
    for(double x:h.strain_curvature) EXPECT_EQ(x,0);
    for(double x:h.internal_work) EXPECT_EQ(x,0);
    EXPECT_EQ(h.hourglass_viscous_work,0);
    for(unsigned i=0;i<4;++i) {
      EXPECT_EQ(qeph_startup_test::Length(trial.internal_force[i]),0);
      EXPECT_EQ(qeph_startup_test::Length(trial.internal_couple[i]),0);
    }
  }
  const auto contact=w.Host(state,0,1);
  EXPECT_EQ(contact.potential.upper,0); EXPECT_EQ(contact.resultant.upper,0);
}
} // namespace qeph_wall_test
