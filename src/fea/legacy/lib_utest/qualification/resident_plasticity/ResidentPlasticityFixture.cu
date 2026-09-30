#include "ResidentPlasticityFixture.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"
#include <algorithm>

namespace resident_plasticity_test {
namespace {
bool InitializeImpl(Rig& r,bool mismatch_curve,bool triangle_elastic,
    tl::material::TabulatedShellPlasticityRate rate,bool mismatch_rate) {
  if(!r.PrepareReference()) return false;
  const auto state=r.initial.Initialize(r.owner);
  EXPECT_EQ(state.status,fe::NodalStatus::Ok); if(state.status!=fe::NodalStatus::Ok) return false;
  // Caller arrays are deliberately temporary. Every later step exercises the
  // resident deep copy, including host-side joined declaration comparisons.
  double x[]{0,.1,.3},y[]{2700,3400,3620};
  fe::ShellBatchPlasticityConfig material{37,47,{x,y,3}};
  material.rate=rate;
  q::QephBatchConfig qc; qc.owner=r.owner.accepted(); qc.element_count=1;
  qc.configuration_id=Configuration; qc.qualification_id=Qualification; qc.usage=q::BatchUsage::PrescribedFields;
  const auto qr=r.qeph.InitializeJoined(qc,r.binding,material);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message; if(qr.status!=q::BatchStatus::Success) return false;
  if(mismatch_curve) y[1]+=1; // Same caller pointer, different immutable values.
  if(mismatch_rate) material.rate.cutoff_hz+=1;
  t::T3BatchConfig tc; tc.owner=r.owner.accepted(); tc.element_count=1;
  tc.configuration_id=Configuration; tc.qualification_id=Qualification; tc.usage=t::BatchUsage::PrescribedFields;
  const auto tr=triangle_elastic?r.t3.InitializeJoined(tc,r.binding):r.t3.InitializeJoined(tc,r.binding,material);
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  for(double& value:x) value=-1;
  for(double& value:y) value=-1;
  material.rate={false,-1,-1,-1}; // No borrowed declaration survives initialization.
  return tr.status==t::BatchStatus::Success;
}
}
bool Initialize(Rig& r,bool mismatch_curve,bool triangle_elastic) {
  return InitializeImpl(r,mismatch_curve,triangle_elastic,{},false);
}
bool InitializeRate(Rig& r,tl::material::TabulatedShellPlasticityRate rate,bool mismatch_rate) {
  return InitializeImpl(r,false,false,rate,mismatch_rate);
}
Loads PlasticSchedule(const Rig& r,unsigned interval) {
  auto loads=Schedule(r,interval);
  for(double& x:loads.force) x*=4000;
  for(double& x:loads.couple) x*=4000;
  return loads;
}
bool Sections(Rig& r,SectionPair& output,const Staged* prepared) {
  SectionPair next; q::BatchDiagnostics qd; t::BatchDiagnostics td;
  const auto qr=prepared?r.qeph.CopyPreparedSectionHistory(prepared->diagnostics.qeph,&next.q,1):
      r.qeph.CopyAcceptedSectionHistory(r.owner.accepted(),&next.q,1,&qd);
  const auto tr=prepared?r.t3.CopyPreparedSectionHistory(prepared->diagnostics.t3,&next.t,1):
      r.t3.CopyAcceptedSectionHistory(r.owner.accepted(),&next.t,1,&td);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  if(qr.status!=q::BatchStatus::Success||tr.status!=t::BatchStatus::Success) return false;
  if(!prepared) { EXPECT_EQ(qd.epoch,r.owner.accepted().epoch); EXPECT_EQ(td.epoch,qd.epoch); }
  output=next; return true;
}
void SameSections(const SectionPair& a,const SectionPair& b) {
  EXPECT_EQ(Bytes(a.q),Bytes(b.q)); EXPECT_EQ(Bytes(a.t),Bytes(b.t));
}
namespace {
void Near(double a,double b) {
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b));
  EXPECT_LE(std::abs(a-b),2e-12*std::max({1.,std::abs(a),std::abs(b)}));
}
template<class Force> void Compare(const Force& a,const Force& b) {
  const auto& x=a.proposed_history.data(); const auto& y=b.proposed_history.data();
  for(unsigned i=0;i<5;++i) { Near(x.stress[i],y.stress[i]); Near(x.material_stress[i],y.material_stress[i]); }
  for(unsigned i=0;i<3;++i) Near(x.bending_stress[i],y.bending_stress[i]);
  for(unsigned i=0;i<8;++i) Near(x.strain_curvature[i],y.strain_curvature[i]);
  for(unsigned i=0;i<2;++i) Near(x.internal_work[i],y.internal_work[i]);
  Near(x.thickness,y.thickness);
  for(unsigned i=0;i<std::size(a.internal_force);++i) {
    Near(a.internal_force[i].x,b.internal_force[i].x); Near(a.internal_force[i].y,b.internal_force[i].y);
    Near(a.internal_force[i].z,b.internal_force[i].z); Near(a.internal_couple[i].x,b.internal_couple[i].x);
    Near(a.internal_couple[i].y,b.internal_couple[i].y); Near(a.internal_couple[i].z,b.internal_couple[i].z);
  }
}
void Compare(const fe::sections::ShellLayeredJ2History& a,const fe::sections::ShellLayeredJ2History& b) {
  for(unsigned p=0;p<3;++p) {
    Near(a.point[p].plastic_strain,b.point[p].plastic_strain);
    Near(a.point[p].filtered_rate_per_s,b.point[p].filtered_rate_per_s);
    for(unsigned s=0;s<5;++s) Near(a.point[p].stress[s],b.point[p].stress[s]);
  }
}
}
namespace {
void CheckHostParameters(const Rig& r,const Prepared& p,const Staged& old_shell,
    const SectionPair& old_section,const Staged& next,const SectionPair& section,
    const fe::sections::PointParameters& qparameters,const fe::sections::PointParameters& tparameters) {
  q::LayeredJ2ForceTrial qtrial; t::LayeredJ2ForceTrial ttrial;
  ASSERT_EQ(q::EvaluateLayeredJ2Force(r.binding.qeph_reference(),qparameters,
      {old_shell.qeph.proposed_history,old_section.q.history},QephInterval(r,p),qtrial),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateLayeredJ2Force(r.binding.t3_reference(),tparameters,
      {old_shell.t3.proposed_history,old_section.t.history},T3Interval(r,p),ttrial),t::Status::kSuccess);
  Compare(next.qeph,qtrial.force); Compare(next.t3,ttrial.force);
  Compare(section.q.history,qtrial.proposed_section); Compare(section.t.history,ttrial.proposed_section);
  Near(section.q.cumulative_plastic_work_J,old_section.q.cumulative_plastic_work_J+
      qtrial.section_diagnostics.plastic_work_density_increment*old_shell.qeph.proposed_history.data().thickness*
      qtrial.force.kinematics.area);
  Near(section.t.cumulative_plastic_work_J,old_section.t.cumulative_plastic_work_J+
      ttrial.section_diagnostics.plastic_work_density_increment*old_shell.t3.proposed_history.data().thickness*
      ttrial.force.kinematics.area);
}
} // namespace
void CheckHostAdapters(const Rig& r,const Prepared& p,const Staged& old_shell,
    const SectionPair& old_section,const Staged& next,const SectionPair& section,
    tl::material::TabulatedShellPlasticityRate rate) {
  fe::sections::PointParameters parameters;
  const auto& material=r.binding.qeph_reference().input;
  ASSERT_EQ(tl::material::PrepareTabulatedShellPlasticity(material.young_modulus,material.poisson_ratio,material.density,
      {CurveX,CurveY,3},rate,parameters),tl::material::TabulatedShellPlasticityStatus::Ok);
  CheckHostParameters(r,p,old_shell,old_section,next,section,parameters,parameters);
}
void CheckHostAdapters(const Rig& r,const Prepared& p,const Staged& old_shell,
    const SectionPair& old_section,const Staged& next,const SectionPair& section,
    const fe::ShellBatchPlasticityBinding& catalog) {
  fe::sections::PointParameters qp,tp;
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qeph,0,&qp));
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::T3,0,&tp));
  CheckHostParameters(r,p,old_shell,old_section,next,section,qp,tp);
}
} // namespace resident_plasticity_test
