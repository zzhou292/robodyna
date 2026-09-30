#pragma once
// Test-only native adapters and independent oracles. No production equations.
#include "T3PortFixture.h"
#include "lib_src/elements/t3/T3Force.h"
#include "../native/t3/T3ForceTestOracle.h"
#include <iomanip>

namespace t3_force_port_test {
using namespace t3_port_test;
using t3_port_test::Native;
namespace oracle=native::force_test;
inline native::HistoryValues Native(const port::HistoryValues& h) {
  native::HistoryValues n;
  for(unsigned i=0;i<5;++i) { n.stress[i]=h.stress[i]; n.material_stress[i]=h.material_stress[i]; }
  for(unsigned i=0;i<3;++i) n.bending_stress[i]=h.bending_stress[i];
  for(unsigned i=0;i<8;++i) n.strain_curvature[i]=h.strain_curvature[i];
  n.thickness=h.thickness; n.internal_work={h.internal_work[0],h.internal_work[1]};
  n.equivalent_strain_rate=h.equivalent_strain_rate; n.active=h.active; return n;
}
inline port::HistoryValues Values(const native::HistoryValues& n) {
  port::HistoryValues h;
  for(unsigned i=0;i<5;++i) { h.stress[i]=n.stress[i]; h.material_stress[i]=n.material_stress[i]; }
  for(unsigned i=0;i<3;++i) h.bending_stress[i]=n.bending_stress[i];
  for(unsigned i=0;i<8;++i) h.strain_curvature[i]=n.strain_curvature[i];
  h.thickness=n.thickness; h.internal_work[0]=n.internal_work[0]; h.internal_work[1]=n.internal_work[1];
  h.equivalent_strain_rate=n.equivalent_strain_rate; h.active=n.active; return h;
}
inline port::History History(const port::ReferenceData& r,port::HistoryValues values={},port::HistoryStamp stamp={}) {
  if(values.thickness==0) values.thickness=r.input.thickness;
  port::History out;
  if(port::PreparePrescribedHistory(r,values,stamp,out)!=port::Status::kSuccess)
    throw std::runtime_error("T3 force port history fixture rejected");
  return out;
}
inline native::History Native(const native::Reference& r,const port::History& h) {
  return oracle::MakeHistory(r,Native(h.data()),{h.stamp().time,h.stamp().sample_index});
}
inline std::array<double,10> Diagnostics(const port::ForceDiagnostics& d) {
  return {d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,d.shear_factor,d.transverse_shear_modulus,
    d.translational_stiffness,d.rotational_stiffness,d.unscaled_element_dt,d.internal_work_increment[0],d.internal_work_increment[1]};
}
inline native::ForceTrial Native(const native::Reference& r,const port::ForceTrial& p) {
  native::ForceTrial out; out.proposed_history=Native(r,p.proposed_history); out.kinematics=Native(p.kinematics);
  for(unsigned n=0;n<3;++n) { out.internal_force[n]=p.internal_force[n]; out.internal_couple[n]=p.internal_couple[n]; }
  const auto& a=p.diagnostics; auto& b=out.diagnostics;
  b.effective_thickness=a.effective_thickness; b.native_sound_speed=a.native_sound_speed;
  b.membrane_viscosity=a.membrane_viscosity; b.shear_factor=a.shear_factor; b.transverse_shear_modulus=a.transverse_shear_modulus;
  b.translational_stiffness=a.translational_stiffness; b.rotational_stiffness=a.rotational_stiffness;
  b.unscaled_element_dt=a.unscaled_element_dt; b.internal_work_increment={a.internal_work_increment[0],a.internal_work_increment[1]};
  return out;
}
inline void Field(double actual,double expected,double dimension,double coefficient=2e-12) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  EXPECT_LE(std::abs(actual-expected),coefficient*(dimension+std::abs(expected)))
    <<std::setprecision(18)<<actual<<" expected "<<expected<<" dimension "<<dimension;
}
inline double Length(const port::PrescribedInterval& in) {
  const auto a=native::test::Difference(in.position[1],in.position[0]);
  const auto b=native::test::Difference(in.position[2],in.position[0]);
  return double(std::max(std::sqrt(native::test::Dot(a,a)),std::sqrt(native::test::Dot(b,b))));
}
inline void Agreement(const port::ReferenceData& r,const port::PrescribedInterval& in,
                      const port::ForceTrial& a,const native::ForceTrial& b,double coefficient=2e-12) {
  RatesAgreement(a.kinematics,b.kinematics,in);
  const double e=r.input.young_modulus,t=r.input.thickness,l=Length(in);
  const double sound=std::sqrt(e/std::max(r.input.density,1e-20));
  const auto h=Native(a.proposed_history.data()); const auto& n=b.proposed_history.data();
  for(unsigned i=0;i<5;++i) { Field(h.stress[i],n.stress[i],e,coefficient); Field(h.material_stress[i],n.material_stress[i],e,coefficient); }
  for(unsigned i=0;i<3;++i) Field(h.bending_stress[i],n.bending_stress[i],e*t/l,coefficient);
  for(unsigned i=0;i<8;++i) Field(h.strain_curvature[i],n.strain_curvature[i],i<5?1.:1./l,coefficient);
  Field(h.thickness,n.thickness,t,coefficient);
  Field(h.internal_work[0],n.internal_work[0],e*t*l*l,coefficient);
  Field(h.internal_work[1],n.internal_work[1],e*t*t*t,coefficient);
  Field(h.equivalent_strain_rate,n.equivalent_strain_rate,1/in.dt,coefficient);
  EXPECT_EQ(h.active,n.active); EXPECT_TRUE(a.proposed_history.prepared());
  EXPECT_TRUE(a.proposed_history.matches_reference(r));
  EXPECT_EQ(a.proposed_history.stamp().time,b.proposed_history.stamp().time);
  EXPECT_EQ(a.proposed_history.stamp().sample_index,b.proposed_history.stamp().sample_index);
  for(unsigned i=0;i<3;++i) for(unsigned axis=0;axis<3;++axis) {
    Field(Component(a.internal_force[i],axis),Component(b.internal_force[i],axis),e*t*l,coefficient);
    Field(Component(a.internal_couple[i],axis),Component(b.internal_couple[i],axis),e*t*l*l,coefficient);
  }
  const auto da=Diagnostics(a.diagnostics); const auto& d=b.diagnostics;
  const double db[]{d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,d.shear_factor,d.transverse_shear_modulus,
      d.translational_stiffness,d.rotational_stiffness,d.unscaled_element_dt,d.internal_work_increment[0],d.internal_work_increment[1]};
  const double scale[]{t,sound,1,1,e,e*t,e*t*l*l,l/sound,e*t*l*l,e*t*t*t};
  for(unsigned i=0;i<10;++i) { SCOPED_TRACE(i); Field(da[i],db[i],scale[i],coefficient); }
}
inline void Check(const port::ReferenceData& r,const port::History& h,const port::PrescribedInterval& in,
                  const port::ForceTrial& out,bool independent=false) {
  const auto reference=kt::MakeReference(Native(r.input)); const auto base=Native(reference,h);
  native::ForceTrial expected;
  ASSERT_EQ(native::EvaluateForce(reference,base,Native(in),expected),native::Status::kSuccess);
  Agreement(r,in,out,expected);
  if(independent) oracle::Check(reference,base.data(),Native(in),Native(reference,out));
}
inline void Power(const port::ReferenceData& r,const port::PrescribedInterval& in,const port::ForceTrial& out) {
  // Supply actual current coordinates to the retained independent rate-map
  // oracle. This local copy is not reference reinitialization or force setup.
  const auto original=kt::MakeReference(Native(r.input)); auto current=Native(r.input);
  for(unsigned n=0;n<3;++n) current.position[n]=in.position[n];
  oracle::CheckFixedResultantPower(current,Native(original,out));
}
inline void Balance(const port::PrescribedInterval& in,const port::ForceTrial& out) {
  kt::Wide force{},moment{};
  for(unsigned n=0;n<3;++n) {
    force=kt::Add(force,kt::Widen(out.internal_force[n]));
    moment=kt::Add(moment,kt::Add(kt::Cross(native::test::Difference(in.position[n],in.position[0]),
      kt::Widen(out.internal_force[n])),kt::Widen(out.internal_couple[n])));
  }
  for(unsigned axis=0;axis<3;++axis) { oracle::Near(double(force[axis]),0); oracle::Near(double(moment[axis]),0); }
}
inline void Mode(port::PrescribedInterval& in,unsigned mode,double rate) {
  auto n=Native(in); oracle::Mode(n,mode,rate); in=Input(n);
}
inline void Exact(const port::ForceTrial& a,const port::ForceTrial& b) {
  ExactRates(a.kinematics,b.kinematics);
  std::array<double,26> x{},y{}; native::detail::PackHistory(Native(a.proposed_history.data()),x);
  native::detail::PackHistory(Native(b.proposed_history.data()),y); EXPECT_EQ(x,y);
  EXPECT_EQ(a.proposed_history.prepared(),b.proposed_history.prepared());
  EXPECT_EQ(a.proposed_history.stamp().time,b.proposed_history.stamp().time);
  EXPECT_EQ(a.proposed_history.stamp().sample_index,b.proposed_history.stamp().sample_index);
  for(unsigned n=0;n<3;++n) for(unsigned axis=0;axis<3;++axis) {
    EXPECT_EQ(Component(a.internal_force[n],axis),Component(b.internal_force[n],axis));
    EXPECT_EQ(Component(a.internal_couple[n],axis),Component(b.internal_couple[n],axis));
  }
  EXPECT_EQ(Diagnostics(a.diagnostics),Diagnostics(b.diagnostics));
}
} // namespace t3_force_port_test
