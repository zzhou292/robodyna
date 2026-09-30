#pragma once
#include "Fixture.h"
namespace global_law1_test {
// Independent dimensional conversion. Time remains seconds. Numerical native
// reference inputs use length L metres and mass M kilograms per source unit.
struct Units {
  double length=1,mass=1;
  double stress() const {return mass/length;}
  double force() const {return mass*length;}
  double work() const {return mass*length*length;}
  tl::math::Vec3 ToNative(tl::math::Vec3 x) const {return {x.x/length,x.y/length,x.z/length};}
  template<class R> R Reference(R r) const {
    for(auto& x:r.position)x=ToNative(x);
    r.thickness/=length;r.young_modulus/=stress();r.density*=length*length*length/mass;return r;
  }
  nq::PrescribedInterval Interval(const q::PrescribedInterval& x) const {
    auto n=qeph_kinematics_test::NativeInterval(x);
    for(auto& v:n.position_endpoint)v=ToNative(v);for(auto& v:n.velocity_midpoint)v=ToNative(v);return n;
  }
  nt::PrescribedInterval Interval(const t::PrescribedInterval& x) const {
    auto n=t3_port_test::Native(x);
    for(auto& v:n.position)v=ToNative(v);for(auto& v:n.velocity)v=ToNative(v);return n;
  }
};
inline void UnitClose(double a,double b,double scale) {
  qeph_kinematics_test::Field(a,b,scale,path::cv::Tolerance,"unit_conversion");
}
template<class Port,class Native> void CommonHistory(const Port& a,const Native& b,const Units& u,double young,double thickness) {
  for(unsigned i=0;i<5;++i) {
    UnitClose(a.stress[i],b.stress[i]*u.stress(),young);
    UnitClose(a.material_stress[i],b.material_stress[i]*u.stress(),young);
  }
  for(unsigned i=0;i<3;++i)UnitClose(a.bending_stress[i],b.bending_stress[i]*u.stress(),young);
  for(unsigned i=0;i<8;++i)UnitClose(a.strain_curvature[i],b.strain_curvature[i]*(i<5?1.:1./u.length),i<5?1.:100.);
  UnitClose(a.thickness,b.thickness*u.length,thickness);
  for(unsigned i=0;i<2;++i)UnitClose(a.internal_work[i],b.internal_work[i]*u.work(),young*thickness*.0004);
  EXPECT_EQ(a.active,b.active);
}
template<unsigned N,class Port,class Native> void CommonForce(const Port& a,const Native& b,const Units& u,double young,double thickness) {
  CommonHistory(a.proposed_history.data(),b.proposed_history.data(),u,young,thickness);
  EXPECT_EQ(a.proposed_history.stamp().sample_index,b.proposed_history.stamp().sample_index);
  EXPECT_EQ(a.proposed_history.stamp().time,b.proposed_history.stamp().time);
  for(unsigned i=0;i<N;++i)for(unsigned c=0;c<3;++c) {
    UnitClose(t3_port_test::Component(a.internal_force[i],c),t3_port_test::Component(b.internal_force[i],c)*u.force(),young*thickness*.02);
    UnitClose(t3_port_test::Component(a.internal_couple[i],c),t3_port_test::Component(b.internal_couple[i],c)*u.work(),young*thickness*.0004);
  }
  const auto& x=a.diagnostics;const auto& y=b.diagnostics;
  UnitClose(x.effective_thickness,y.effective_thickness*u.length,thickness);
  UnitClose(x.native_sound_speed,y.native_sound_speed*u.length,1e4);
  EXPECT_EQ(x.membrane_viscosity,y.membrane_viscosity);
  UnitClose(x.translational_stiffness,y.translational_stiffness*u.mass,young*thickness);
  UnitClose(x.rotational_stiffness,y.rotational_stiffness*u.work(),young*thickness*.0004);
  UnitClose(x.unscaled_element_dt,y.unscaled_element_dt,1e-5);
  for(unsigned i=0;i<2;++i)UnitClose(x.internal_work_increment[i],y.internal_work_increment[i]*u.work(),young*thickness*.0004);
}
} // namespace global_law1_test
