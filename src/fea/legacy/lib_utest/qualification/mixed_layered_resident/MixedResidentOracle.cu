#include "MixedResidentFixture.h"
#include <algorithm>
#include <cmath>

namespace mixed_layered_test {
namespace {
q::PrescribedInterval QInterval(const Rig& r,const Prepared& p,unsigned parent) {
  auto in=QephInterval(r,p);
  for(unsigned i=0;i<4;++i) {
    const auto n=r.binding.qeph_nodes(parent)[i];
    in.position_endpoint[i]={p.endpoint.x[3*n],p.endpoint.x[3*n+1],p.endpoint.x[3*n+2]};
    in.velocity_midpoint[i]={p.endpoint.v[3*n],p.endpoint.v[3*n+1],p.endpoint.v[3*n+2]};
    in.omega_midpoint[i]={p.endpoint.omega[3*n],p.endpoint.omega[3*n+1],p.endpoint.omega[3*n+2]};
  }
  return in;
}
t::PrescribedInterval TInterval(const Rig& r,const Prepared& p,unsigned parent) {
  auto in=T3Interval(r,p);
  for(unsigned i=0;i<3;++i) {
    const auto n=r.binding.t3_nodes(parent)[i];
    in.position[i]={p.endpoint.x[3*n],p.endpoint.x[3*n+1],p.endpoint.x[3*n+2]};
    in.velocity[i]={p.endpoint.v[3*n],p.endpoint.v[3*n+1],p.endpoint.v[3*n+2]};
    in.angular_velocity[i]={p.endpoint.omega[3*n],p.endpoint.omega[3*n+1],p.endpoint.omega[3*n+2]};
  }
  return in;
}
void Near(double a,double b) {
  ASSERT_TRUE(std::isfinite(a));ASSERT_TRUE(std::isfinite(b));
  EXPECT_LE(std::abs(a-b),2e-12*std::max({1.,std::abs(a),std::abs(b)}));
}
template<class Force> void ForceAgreement(const Force& actual,const Force& oracle) {
  const auto& a=actual.proposed_history.data();const auto& b=oracle.proposed_history.data();
  for(unsigned c=0;c<5;++c) {Near(a.stress[c],b.stress[c]);Near(a.material_stress[c],b.material_stress[c]);}
  for(unsigned c=0;c<3;++c)Near(a.bending_stress[c],b.bending_stress[c]);
  for(unsigned c=0;c<8;++c)Near(a.strain_curvature[c],b.strain_curvature[c]);
  for(unsigned c=0;c<2;++c)Near(a.internal_work[c],b.internal_work[c]);
  Near(a.thickness,b.thickness);
  if constexpr(std::is_same_v<Force,q::ForceTrial>)Near(a.hourglass_viscous_work,b.hourglass_viscous_work);
  for(unsigned n=0;n<std::size(actual.internal_force);++n) {
    for(const auto pair:{std::pair{actual.internal_force[n],oracle.internal_force[n]},
                         std::pair{actual.internal_couple[n],oracle.internal_couple[n]}}) {
      Near(pair.first.x,pair.second.x);Near(pair.first.y,pair.second.y);Near(pair.first.z,pair.second.z);
    }
  }
}
template<class History> void Stresses(const History& a,const History& b) {
  for(unsigned layer=0;layer<3;++layer)for(unsigned c=0;c<5;++c)Near(a.point[layer].stress[c],b.point[layer].stress[c]);
}
template<class Trial,class OldForce> void Plastic(const fe::ShellBatchLayeredSection& observed,
    const fe::ShellBatchLayeredSection& old,const Trial& expected,const OldForce& old_force) {
  ASSERT_NE(observed.plastic(),nullptr);ASSERT_NE(old.plastic(),nullptr);EXPECT_EQ(observed.elastic(),nullptr);
  const auto& value=*observed.plastic();Stresses(value.history,expected.proposed_section);
  for(unsigned layer=0;layer<3;++layer) {
    Near(value.history.point[layer].plastic_strain,expected.proposed_section.point[layer].plastic_strain);
    Near(value.history.point[layer].filtered_rate_per_s,expected.proposed_section.point[layer].filtered_rate_per_s);
  }
  Near(value.diagnostics.maximum_plastic_strain,expected.section_diagnostics.maximum_plastic_strain);
  Near(value.diagnostics.plastic_work_density_increment,expected.section_diagnostics.plastic_work_density_increment);
  Near(value.cumulative_plastic_work_J,old.plastic()->cumulative_plastic_work_J+
    expected.section_diagnostics.plastic_work_density_increment*old_force.proposed_history.data().thickness*expected.force.kinematics.area);
}
}
void CheckAdapters(const Rig& r,const fe::ShellBatchSectionBinding& catalog,const Prepared& p,
    const Frame& old,const Frame& actual) {
  // Each family contains both laws in opposite native index order. Host paths
  // call the independently qualified adapters directly, not the batch dispatcher.
  tl::material::ShellElasticLaw1PointParameters qp,tp;
  ASSERT_TRUE(catalog.ElasticParameters(fe::ShellBindingFamily::Qeph,0,&qp));
  ASSERT_TRUE(catalog.ElasticParameters(fe::ShellBindingFamily::T3,1,&tp));
  ASSERT_NE(old.qsection[0].elastic(),nullptr);ASSERT_NE(old.tsection[1].elastic(),nullptr);
  q::LayeredLaw1ForceTrial qe;t::LayeredLaw1ForceTrial te;
  ASSERT_EQ(q::EvaluateLayeredLaw1Force(r.binding.qeph_reference(0),qp,
    {old.qforce[0].proposed_history,*old.qsection[0].elastic()},QInterval(r,p,0),qe),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateLayeredLaw1Force(r.binding.t3_reference(1),tp,
    {old.tforce[1].proposed_history,*old.tsection[1].elastic()},TInterval(r,p,1),te),t::Status::kSuccess);
  ForceAgreement(actual.qforce[0],qe.force);ForceAgreement(actual.tforce[1],te.force);
  ASSERT_NE(actual.qsection[0].elastic(),nullptr);ASSERT_NE(actual.tsection[1].elastic(),nullptr);
  EXPECT_EQ(actual.qsection[0].plastic(),nullptr);EXPECT_EQ(actual.tsection[1].plastic(),nullptr);
  Stresses(*actual.qsection[0].elastic(),qe.proposed_section);Stresses(*actual.tsection[1].elastic(),te.proposed_section);
  fe::sections::PointParameters qplastic,tplastic;
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qeph,1,&qplastic));
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::T3,0,&tplastic));
  q::LayeredJ2ForceTrial qj;t::LayeredJ2ForceTrial tj;
  ASSERT_NE(old.qsection[1].plastic(),nullptr);ASSERT_NE(old.tsection[0].plastic(),nullptr);
  ASSERT_EQ(q::EvaluateLayeredJ2Force(r.binding.qeph_reference(1),qplastic,
    {old.qforce[1].proposed_history,old.qsection[1].plastic()->history},QInterval(r,p,1),qj),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateLayeredJ2Force(r.binding.t3_reference(0),tplastic,
    {old.tforce[0].proposed_history,old.tsection[0].plastic()->history},TInterval(r,p,0),tj),t::Status::kSuccess);
  ForceAgreement(actual.qforce[1],qj.force);ForceAgreement(actual.tforce[0],tj.force);
  Plastic(actual.qsection[1],old.qsection[1],qj,old.qforce[1]);Plastic(actual.tsection[0],old.tsection[0],tj,old.tforce[0]);
}
} // namespace mixed_layered_test
