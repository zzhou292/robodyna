#pragma once
#include "../shell_layered_law1_force/Law1ForceTestSupport.h"
#include "native/NativeReference.h"
namespace global_law1_test {
using namespace layered_law1_force_test;
namespace global=tl::qualification::global_law1_native;
using Profile=tl::fea::ShellGlobalLaw1Profile;
using Thickness=tl::fea::ShellLaw1Thickness;
inline Profile Accepted(double length=1.) {return {Thickness::Accepted,length};}
inline q::ReferenceInput Quad() {auto x=qeph_startup_test::Case(5);Material(x);return x;}
inline t::ReferenceInput Triangle() {auto x=t3_port_test::Triangle(.02,1);Material(x,10e9);return x;}
inline q::History QHistory(const q::ReferenceData& r,double thickness=0) {
  q::HistoryValues h;h.thickness=thickness?thickness:r.input.thickness;q::History out;
  if(q::PreparePrescribedHistory(r,h,{},out)!=q::Status::kSuccess)throw std::runtime_error("QEPH history fixture");return out;
}
inline t::History THistory(const t::ReferenceData& r,double thickness=0) {
  t::HistoryValues h;h.thickness=thickness?thickness:r.input.thickness;
  return t3_force_port_test::History(r,h);
}
inline nq::History QNativeHistory(const nq::Reference& r,const q::History& h) {
  nq::History out;if(nq::PreparePrescribedHistory(r,qeph_force_port_test::NativeValues(h.data()),
      {h.stamp().time,h.stamp().sample_index},out)!=nq::Status::kSuccess)throw std::runtime_error("Native QEPH history fixture");return out;
}
inline void DoubleBits(double a,double b) {
  std::uint64_t x=0,y=0;std::memcpy(&x,&a,sizeof x);std::memcpy(&y,&b,sizeof y);EXPECT_EQ(x,y);
}
template<class A,class B> void ArrayBits(const A& a,const B& b,unsigned count) {
  for(unsigned i=0;i<count;++i)DoubleBits(a[i],b[i]);
}
template<class Trial> void OrdinaryBits(const Trial& a,const Trial& b,unsigned nodes) {
  const auto& x=a.proposed_history.data();const auto& y=b.proposed_history.data();
  ArrayBits(x.stress,y.stress,5);ArrayBits(x.material_stress,y.material_stress,5);
  ArrayBits(x.bending_stress,y.bending_stress,3);ArrayBits(x.strain_curvature,y.strain_curvature,8);
  ArrayBits(x.internal_work,y.internal_work,2);DoubleBits(x.thickness,y.thickness);DoubleBits(x.active,y.active);
  for(unsigned n=0;n<nodes;++n)for(unsigned c=0;c<3;++c) {
    DoubleBits(t3_port_test::Component(a.internal_force[n],c),t3_port_test::Component(b.internal_force[n],c));
    DoubleBits(t3_port_test::Component(a.internal_couple[n],c),t3_port_test::Component(b.internal_couple[n],c));
  }
  const auto& p=a.diagnostics;const auto& q=b.diagnostics;
  DoubleBits(p.effective_thickness,q.effective_thickness);DoubleBits(p.native_sound_speed,q.native_sound_speed);
  DoubleBits(p.membrane_viscosity,q.membrane_viscosity);DoubleBits(p.translational_stiffness,q.translational_stiffness);
  DoubleBits(p.rotational_stiffness,q.rotational_stiffness);DoubleBits(p.unscaled_element_dt,q.unscaled_element_dt);
  ArrayBits(p.internal_work_increment,q.internal_work_increment,2);
}
inline void ForceBits(const q::ForceTrial& a,const q::ForceTrial& b) {
  OrdinaryBits(a,b,4);ArrayBits(a.proposed_history.data().stabilization,b.proposed_history.data().stabilization,12);
  DoubleBits(a.proposed_history.data().hourglass_viscous_work,b.proposed_history.data().hourglass_viscous_work);
  DoubleBits(a.diagnostics.hourglass_viscous_work_increment,b.diagnostics.hourglass_viscous_work_increment);
  DoubleBits(a.diagnostics.stabilization_viscosity,b.diagnostics.stabilization_viscosity);
}
inline void ForceBits(const t::ForceTrial& a,const t::ForceTrial& b) {
  OrdinaryBits(a,b,3);DoubleBits(a.proposed_history.data().equivalent_strain_rate,b.proposed_history.data().equivalent_strain_rate);
  DoubleBits(a.diagnostics.shear_factor,b.diagnostics.shear_factor);
  DoubleBits(a.diagnostics.transverse_shear_modulus,b.diagnostics.transverse_shear_modulus);
}
} // namespace global_law1_test
