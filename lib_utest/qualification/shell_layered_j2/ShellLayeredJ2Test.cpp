#include "LayeredJ2Fixture.h"
#include "lib_utest/qualification/native/law44/NativePoint.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>

namespace layered_j2_test {
namespace native=tl::qualification::law44;
template<class T> auto Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> b{}; std::memcpy(b.data(),&value,sizeof(T)); return b;
}
void Near(double a,double b,double absolute=1e-8) {
  EXPECT_NEAR(a,b,absolute+2e-11*std::max(std::abs(a),std::abs(b)));
}
TEST(ShellLayeredJ2, ExactNativeSectionTableAndElasticBendingLimit) {
  const auto rule=native::NativeSectionRule();
  for(unsigned i=0;i<3;++i) {
    EXPECT_EQ(sec::LayerPosition(i),rule.position[i]);
    EXPECT_EQ(sec::LayerForceWeight(i),rule.membrane_weight[i]);
    EXPECT_EQ(sec::LayerMomentWeight(i),rule.moment_weight[i]);
  }
  auto p=Parameters(); auto in=Input(p); in.strain_curvature_increment[5]=.01;
  sec::ShellLayeredJ2Result out;
  ASSERT_EQ(sec::UpdateShellLayeredJ2(p,{},in,out),sec::PointStatus::Ok);
  EXPECT_DOUBLE_EQ(out.material_stress[0],0.);
  Near(out.bending_stress[0],p.a11*in.reference_thickness*.01*rule.moment_weight[2]);
  EXPECT_EQ(out.diagnostics.maximum_plastic_strain,0.);
  EXPECT_NE(rule.moment_weight[2],1./12.);
}
TEST(ShellLayeredJ2, MixedMembraneBendingAndUnloadMatchIndependentNativePoints) {
  const auto p=Parameters(); const auto rule=native::NativeSectionRule(); auto in=Input(p);
  sec::ShellLayeredJ2History accepted;
  std::array<native::Result,3> previous{};
  const double schedule[4][8]={{.002,-.0006,.0003,.0001,-.0002,1.2,-.3,.2},
    {.001,-.0003,-.0001,0,0,.8,.2,-.1},{-.0002,.00006,0,0,0,-.1,0,0},
    {.001,-.0002,.0002,-.0001,.0001,.4,-.2,.1}};
  for(const auto& step:schedule) {
    for(unsigned c=0;c<8;++c) in.strain_curvature_increment[c]=step[c];
    sec::ShellLayeredJ2Result actual;
    ASSERT_EQ(sec::UpdateShellLayeredJ2(p,accepted,in,actual),sec::PointStatus::Ok);
    double force[5]{},moment[3]{},thickness=in.reported_thickness,plastic_work=0;
    for(unsigned layer=0;layer<3;++layer) {
      native::Input n;
      n.young=p.young_pa; n.poisson=p.poisson_ratio; n.density=p.density_kg_m3;
      n.transverse_shear_modulus=in.transverse_shear_modulus;
      n.plastic_strain=strains; n.yield_stress=yields; n.point_count=5;
      n.accepted_stress=previous[layer].stress; n.accepted_plastic_strain=previous[layer].plastic_strain;
      for(unsigned c=0;c<3;++c) n.strain_increment[c]=step[c]+rule.position[layer]*in.reference_thickness*step[c+5];
      n.strain_increment[3]=step[3]; n.strain_increment[4]=step[4];
      native::Result value; ASSERT_TRUE(native::Evaluate(n,value)); previous[layer]=value;
      for(unsigned c=0;c<5;++c) { Near(actual.history.point[layer].stress[c],value.stress[c]); force[c]+=rule.membrane_weight[layer]*value.stress[c]; }
      for(unsigned c=0;c<3;++c) moment[c]+=rule.moment_weight[layer]*value.stress[c];
      Near(actual.history.point[layer].plastic_strain,value.plastic_strain,1e-15);
      thickness+=value.total_thickness_strain*(rule.membrane_weight[layer]*in.reference_thickness);
      plastic_work+=rule.membrane_weight[layer]*value.plastic_work_density;
    }
    for(unsigned c=0;c<5;++c) Near(actual.material_stress[c],force[c]);
    for(unsigned c=0;c<3;++c) Near(actual.bending_stress[c],moment[c]);
    Near(actual.reported_thickness,thickness,1e-17);
    Near(actual.diagnostics.plastic_work_density_increment,plastic_work);
    accepted=actual.history; in.reported_thickness=actual.reported_thickness;
  }
  EXPECT_GT(accepted.point[2].plastic_strain,accepted.point[0].plastic_strain);
}
TEST(ShellLayeredJ2, ThirdLayerFailurePreservesWholeOutputAndExactRetry) {
  const auto p=Parameters(); auto in=Input(p); in.strain_curvature_increment[0]=.0001;
  sec::ShellLayeredJ2Result output;
  ASSERT_EQ(sec::UpdateShellLayeredJ2(p,{},in,output),sec::PointStatus::Ok);
  const auto clean=output; const auto held=Bytes(output);
  sec::ShellLayeredJ2History bad; bad.point[2].plastic_strain=.401;
  EXPECT_EQ(sec::UpdateShellLayeredJ2(p,bad,in,output),sec::PointStatus::CurveDomainExceeded);
  EXPECT_EQ(Bytes(output),held);
  ASSERT_EQ(sec::UpdateShellLayeredJ2(p,{},in,output),sec::PointStatus::Ok);
  EXPECT_EQ(Bytes(output),Bytes(clean));
  in.strain_curvature_increment[7]=std::numeric_limits<double>::infinity();
  EXPECT_EQ(sec::UpdateShellLayeredJ2(p,{},in,output),sec::PointStatus::InvalidIncrement);
  EXPECT_EQ(Bytes(output),held);
}
TEST(ShellLayeredJ2, GeneralizedWorkCountsPlasticityOnlyThroughStressWork) {
  const auto p=Parameters(); auto in=Input(p); in.strain_curvature_increment[0]=.004;
  sec::ShellLayeredJ2Result section;
  ASSERT_EQ(sec::UpdateShellLayeredJ2(p,{},in,section),sec::PointStatus::Ok);
  q::HistoryValues h; h.thickness=in.reported_thickness;
  ASSERT_TRUE(sec::ApplyLayeredJ2Work(section,in.strain_curvature_increment,.002,.0004,0.,h));
  Near(h.internal_work[0],.5*.002*.0004*section.material_stress[0]*.004,1e-17);
  EXPECT_GT(section.diagnostics.plastic_work_density_increment,0.);
  EXPECT_EQ(h.internal_work[1],0.);
}
TEST(ShellLayeredJ2, BothFamiliesYieldAndRetainAtomicHistoriesOnFailure) {
  const auto p=Parameters(); const auto f=Families(p);
  q::LayeredJ2ForceTrial qo; t::LayeredJ2ForceTrial to;
  ASSERT_EQ(q::EvaluateLayeredJ2Force(f.qr,p,f.qh,f.qi,qo),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateLayeredJ2Force(f.tr,p,f.th,f.ti,to),t::Status::kSuccess);
  EXPECT_GT(qo.section_diagnostics.maximum_plastic_strain,0.);
  EXPECT_GT(to.section_diagnostics.maximum_plastic_strain,0.);
  for(unsigned c=0;c<5;++c) Near(qo.proposed_section.point[1].stress[c],to.proposed_section.point[1].stress[c]);
  EXPECT_TRUE(sec::MatchesLayeredJ2Resultants(qo.proposed_section,qo.force.proposed_history.data()));
  EXPECT_TRUE(sec::MatchesLayeredJ2Resultants(to.proposed_section,to.force.proposed_history.data()));
  const auto qb=Bytes(qo); const auto tb=Bytes(to);
  auto qi=f.qi; auto ti=f.ti; qi.sample_index=ti.sample_index=3;
  EXPECT_NE(q::EvaluateLayeredJ2Force(f.qr,p,f.qh,qi,qo),q::Status::kSuccess);
  EXPECT_NE(t::EvaluateLayeredJ2Force(f.tr,p,f.th,ti,to),t::Status::kSuccess);
  EXPECT_EQ(Bytes(qo),qb); EXPECT_EQ(Bytes(to),tb);
  auto qh=f.qh; auto th=f.th; qh.section.point[2].stress[0]=th.section.point[2].stress[0]=1;
  EXPECT_NE(q::EvaluateLayeredJ2Force(f.qr,p,qh,f.qi,qo),q::Status::kSuccess);
  EXPECT_NE(t::EvaluateLayeredJ2Force(f.tr,p,th,f.ti,to),t::Status::kSuccess);
  EXPECT_EQ(Bytes(qo),qb); EXPECT_EQ(Bytes(to),tb);
  ASSERT_EQ(q::EvaluateLayeredJ2Force(f.qr,p,f.qh,f.qi,qo),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateLayeredJ2Force(f.tr,p,f.th,f.ti,to),t::Status::kSuccess);
  EXPECT_EQ(Bytes(qo),qb); EXPECT_EQ(Bytes(to),tb);
  static_assert(sizeof(t::ForceTrial)==976,"Legacy native T3 ForceTrial layout remains pinned");
}
} // namespace layered_j2_test
