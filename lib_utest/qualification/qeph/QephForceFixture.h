#pragma once
#include "QephKinematicsFixture.h"
#include "lib_src/elements/qeph/QephForce.h"
#include "lib_utest/qualification/native/qeph/QephForceReference.h"

namespace qeph_force_port_test {
using namespace qeph_kinematics_test;
constexpr double kIndependentRelative=2e-10,kIndependentAbsolute=2e-11,kWorkAbsolute=2e-22;
inline void Independent(double a,double b,double absolute=kIndependentAbsolute) {
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b));
  EXPECT_LE(std::abs(a-b),absolute+kIndependentRelative*std::max(std::abs(a),std::abs(b)))
      <<std::setprecision(17)<<a<<" versus "<<b;
}
inline port::HistoryValues Seed(const port::ReferenceInput& input,bool nonzero) {
  port::HistoryValues h; h.thickness=input.thickness;
  if(!nonzero) return h;
  const double e=input.young_modulus,l=Scale(input),t=input.thickness;
  for(unsigned i=0;i<5;++i) { h.stress[i]=e*1e-5*(i+1); h.material_stress[i]=-e*2e-6*(i+1); }
  for(unsigned i=0;i<3;++i) h.bending_stress[i]=e*t/l*1e-6*(i+1);
  for(unsigned i=0;i<12;++i) h.stabilization[i]=e*1e-6*(i+1)*(i%2?-1.:1.)/
      ((i==2||i==3||i==8||i==9)?l:1.);
  for(unsigned i=0;i<8;++i) h.strain_curvature[i]=1e-5*(i+1)/(i<5?1.:l);
  h.internal_work[0]=-e*t*l*l*1e-6; h.internal_work[1]=e*t*t*t*1e-6;
  h.hourglass_viscous_work=e*t*l*l*1e-7;
  return h;
}
inline native::HistoryValues NativeValues(const port::HistoryValues& h) {
  native::HistoryValues n;
  for(unsigned i=0;i<5;++i) { n.stress[i]=h.stress[i]; n.material_stress[i]=h.material_stress[i]; }
  for(unsigned i=0;i<3;++i) n.bending_stress[i]=h.bending_stress[i];
  for(unsigned i=0;i<12;++i) n.stabilization[i]=h.stabilization[i];
  for(unsigned i=0;i<8;++i) n.strain_curvature[i]=h.strain_curvature[i];
  n.thickness=h.thickness; n.internal_work={h.internal_work[0],h.internal_work[1]};
  n.hourglass_viscous_work=h.hourglass_viscous_work; n.active=h.active; return n;
}
template<class Expected>
inline void HistoryAgreement(const port::HistoryValues& a,const Expected& b,
    const port::ReferenceInput& reference,double length,double coefficient=kRoundoff) {
  const double e=reference.young_modulus,t=reference.thickness,l=length;
  for(unsigned i=0;i<5;++i) {
    Field(a.stress[i],b.stress[i],e,coefficient,"stress",i);
    Field(a.material_stress[i],b.material_stress[i],e,coefficient,"material_stress",i);
  }
  for(unsigned i=0;i<3;++i) Field(a.bending_stress[i],b.bending_stress[i],e*t/l,coefficient,"bending_stress",i);
  for(unsigned i=0;i<12;++i)
    Field(a.stabilization[i],b.stabilization[i],(i==2||i==3||i==8||i==9)?e/l:e,coefficient,"stabilization",i);
  for(unsigned i=0;i<8;++i)
    Field(a.strain_curvature[i],b.strain_curvature[i],i<5?1.:1./l,coefficient,"strain_curvature",i);
  Field(a.thickness,b.thickness,t,coefficient,"reported_thickness");
  Field(a.internal_work[0],b.internal_work[0],e*t*l*l,coefficient,"internal_work",0);
  Field(a.internal_work[1],b.internal_work[1],e*t*t*t,coefficient,"internal_work",1);
  Field(a.hourglass_viscous_work,b.hourglass_viscous_work,e*t*l*l,coefficient,"hourglass_viscous_work");
  EXPECT_EQ(a.active,b.active);
}
template<class Expected>
inline void ForceAgreement(const port::ForceTrial& a,const Expected& b,
    const port::ReferenceInput& reference,const port::PrescribedInterval& interval,double coefficient=kRoundoff) {
  qeph_kinematics_test::Agreement(a.kinematics,b.kinematics,interval,true,coefficient);
  const double e=reference.young_modulus,t=reference.thickness,l=LengthScale(interval);
  const double c=std::sqrt(e/std::max(reference.density,1e-20));
  HistoryAgreement(a.proposed_history.data(),b.proposed_history.data(),reference,l,coefficient);
  EXPECT_EQ(a.proposed_history.stamp().time,b.proposed_history.stamp().time);
  EXPECT_EQ(a.proposed_history.stamp().sample_index,b.proposed_history.stamp().sample_index);
  EXPECT_TRUE(a.proposed_history.prepared());
  for(unsigned n=0;n<4;++n) {
    Field(a.internal_force[n],b.internal_force[n],e*t*l,coefficient,"internal_force",n);
    Field(a.internal_couple[n],b.internal_couple[n],e*t*l*l,coefficient,"internal_couple",n);
  }
  const auto& x=a.diagnostics; const auto& y=b.diagnostics;
  Field(x.effective_thickness,y.effective_thickness,t,coefficient,"effective_thickness");
  Field(x.native_sound_speed,y.native_sound_speed,c,coefficient,"sound_speed");
  Field(x.membrane_viscosity,y.membrane_viscosity,1.,coefficient,"membrane_viscosity");
  Field(x.stabilization_viscosity,y.stabilization_viscosity,1.,coefficient,"stabilization_viscosity");
  Field(x.translational_stiffness,y.translational_stiffness,e*t,coefficient,"translational_stiffness");
  Field(x.rotational_stiffness,y.rotational_stiffness,e*t*l*l,coefficient,"rotational_stiffness");
  Field(x.unscaled_element_dt,y.unscaled_element_dt,l/c,coefficient,"unscaled_element_dt");
  Field(x.internal_work_increment[0],y.internal_work_increment[0],e*t*l*l,coefficient,"work_increment",0);
  Field(x.internal_work_increment[1],y.internal_work_increment[1],e*t*t*t,coefficient,"work_increment",1);
  Field(x.hourglass_viscous_work_increment,y.hourglass_viscous_work_increment,e*t*l*l,coefficient,"viscous_work_increment");
}
inline port::PrescribedInterval Next(const port::ReferenceInput& input,const port::History& h,double dt=1e-6) {
  auto p=Stationary(input,dt); p.base_time=h.stamp().time; p.sample_index=h.stamp().sample_index+1; return p;
}
inline void ApplyMode(port::PrescribedInterval& p,unsigned mode,double rate) {
  // Independent continuous affine fields; material order XX YY XY YZ ZX KXX KYY KXY.
  for(unsigned n=0;n<4;++n) {
    const auto x=p.position_endpoint[n];
    if(mode==0) p.velocity_midpoint[n].x=rate*x.x;
    if(mode==1) p.velocity_midpoint[n].y=rate*x.y;
    if(mode==2) p.velocity_midpoint[n].x=rate*x.y;
    if(mode==3) p.omega_midpoint[n].x=-rate;
    if(mode==4) p.omega_midpoint[n].y=rate;
    if(mode==5) p.omega_midpoint[n].y=rate*x.x;
    if(mode==6) p.omega_midpoint[n].x=-rate*x.y;
    if(mode==7) p.omega_midpoint[n]={-.5*rate*x.x,.5*rate*x.y,0};
  }
}
inline void ModeStressTruth(const port::ForceTrial& result,const port::ReferenceInput& input,
                             unsigned mode,double rate,double dt) {
  // Long-double independent plane-stress matrix; shear correction 5/6.
  const long double e=input.young_modulus,nu=input.poisson_ratio,t=input.thickness;
  const long double a=e/(1-nu*nu),b=nu*a,g=e/(2*(1+nu)),increment=rate*dt;
  long double d[8]{}; d[mode]=increment;
  // Native midpoint correction for vx=rate*y: eyy=-dt^2*rate^2/2.
  if(mode==2) d[1]=-.5L*dt*dt*rate*rate;
  const long double s[]{a*d[0]+b*d[1],b*d[0]+a*d[1],g*d[2],g*(5.L/6)*d[3],g*(5.L/6)*d[4]};
  const long double moment[]{t*t*t/12*(a*d[5]+b*d[6]),t*t*t/12*(b*d[5]+a*d[6]),t*t*t/12*g*d[7]};
  const auto& h=result.proposed_history.data();
  for(unsigned i=0;i<5;++i) Independent(h.material_stress[i],static_cast<double>(s[i]));
  for(unsigned i=0;i<3;++i)
    Independent(input.thickness*input.thickness*h.bending_stress[i],static_cast<double>(moment[i]));
  const long double thk=t*(1-nu*(d[0]+d[1])/(1-nu));
  Independent(h.thickness,static_cast<double>(thk));
  EXPECT_EQ(result.diagnostics.effective_thickness,input.thickness);
}
inline void Balance(const port::ForceTrial& trial,const port::PrescribedInterval& p) {
  Vec3 total{},moment{}; double fs=0,ms=0;
  for(unsigned n=0;n<4;++n) {
    const auto f=trial.internal_force[n],c=trial.internal_couple[n];
    const auto arm=Cross(Difference(p.position_endpoint[n],p.position_endpoint[0]),f);
    total={total.x+f.x,total.y+f.y,total.z+f.z};
    moment={moment.x+arm.x+c.x,moment.y+arm.y+c.y,moment.z+arm.z+c.z};
    fs+=Length(f); ms+=Length(arm)+Length(c);
  }
  EXPECT_LE(Length(total),kIndependentAbsolute+kIndependentRelative*fs);
  EXPECT_LE(Length(moment),kIndependentAbsolute+kIndependentRelative*ms);
}
} // namespace qeph_force_port_test
