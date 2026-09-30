#include "NativeAnalyticTestSupport.h"
#include "lib_src/elements/sections/ShellLayeredJ2.h"

namespace analytic_test {
namespace section=tl::fea::sections;
TEST(Law44AnalyticNative, NativeNip3IndependentHistoriesForceMomentWorkAndThickness) {
  const auto p=Prepare(); const auto rule=native::NativeSectionRule();
  std::array<native::AnalyticInput,3> independent{Native(),Native(),Native()};
  section::ShellLayeredJ2History accepted;
  double actual_thickness=.002,native_thickness=.002,max_bending=0,max_plastic_difference=0;
  long double actual_work=0,native_work=0;
  for(unsigned step=0;step<1536;++step) {
    SCOPED_TRACE(step);
    const double direction=step<640?1.:step<768?0.:-1.;
    const std::array<double,8> dx{direction*100*Dt,direction*-25*Dt,direction*200*Dt,
      direction*3*Dt,direction*-4*Dt,direction*120000*Dt,direction*-20000*Dt,direction*40000*Dt};
    section::ShellLayeredJ2Input in;
    std::copy(dx.begin(),dx.end(),in.strain_curvature_increment);
    in.reference_thickness=actual_thickness; in.reported_thickness=actual_thickness;
    in.transverse_shear_modulus=independent[0].point.transverse_shear_modulus; in.dt=Dt;
    section::ShellLayeredJ2Result actual;
    ASSERT_EQ(section::UpdateShellLayeredJ2(p,accepted,in,actual),Status::Ok);
    const double rate=native::NativeShellRate(dx,native_thickness,Dt);
    std::array<double,5> force{}; std::array<double,3> moment{};
    double thickness=native_thickness,work=0,mean_pla=0,max_pla=0,mean_tangent=0,min_tangent=1,mean_yield=0,last_yield=0;
    for(unsigned layer=0;layer<3;++layer) {
      auto& oracle=independent[layer]; auto& point=oracle.point;
      const double z=rule.position[layer]*native_thickness,weight=rule.membrane_weight[layer];
      for(unsigned c=0;c<3;++c) point.strain_increment[c]=dx[c]+z*dx[c+5];
      for(unsigned c=3;c<5;++c) point.strain_increment[c]=dx[c];
      point.rate.total_shell_rate_per_s=rate; native::AnalyticResult expected;
      ASSERT_TRUE(native::EvaluateAnalytic(oracle,weight*native_thickness,thickness,expected));
      thickness=expected.reported_thickness_m;
      for(unsigned c=0;c<5;++c) {
        Close(actual.history.point[layer].stress[c],expected.stress[c],1e-8);
        force[c]=force[c]+weight*expected.stress[c];
      }
      for(unsigned c=0;c<3;++c) moment[c]=moment[c]+rule.moment_weight[layer]*expected.stress[c];
      Close(actual.history.point[layer].plastic_strain,expected.plastic_strain,2e-14);
      Close(actual.history.point[layer].filtered_rate_per_s,expected.filtered_rate_per_s,2e-11);
      work=work+weight*expected.plastic_work_density; mean_pla+=weight*expected.plastic_strain;
      max_pla=std::max(max_pla,expected.plastic_strain); mean_tangent+=weight*expected.tangent_ratio;
      min_tangent=std::min(min_tangent,expected.tangent_ratio); mean_yield+=weight*expected.yield_before; last_yield=expected.yield_before;
      Accept(oracle,expected);
    }
    for(unsigned c=0;c<5;++c) Close(actual.material_stress[c],force[c],1e-8);
    for(unsigned c=0;c<3;++c) { Close(actual.bending_stress[c],moment[c],1e-8); max_bending=std::max(max_bending,std::abs(moment[c])); }
    Close(actual.reported_thickness,thickness,2e-14);
    const auto& d=actual.diagnostics;
    Close(d.plastic_work_density_increment,work,1e-8); Close(d.mean_plastic_strain,mean_pla,2e-14);
    Close(d.maximum_plastic_strain,max_pla,2e-14); Close(d.mean_tangent_ratio,mean_tangent,2e-14);
    Close(d.minimum_tangent_ratio,min_tangent,2e-14); Close(d.mean_yield_before_pa,mean_yield,1e-8);
    Close(d.last_point_yield_before_pa,last_yield,1e-8);
    actual_work+=d.plastic_work_density_increment*actual_thickness; native_work+=work*native_thickness;
    max_plastic_difference=std::max(max_plastic_difference,std::abs(actual.history.point[0].plastic_strain-actual.history.point[2].plastic_strain));
    accepted=actual.history; actual_thickness=actual.reported_thickness; native_thickness=thickness;
  }
  Close(static_cast<double>(actual_work),static_cast<double>(native_work),1e-8);
  EXPECT_GT(actual_work,0); EXPECT_GT(max_bending,1e5); EXPECT_GT(max_plastic_difference,.001);
}
} // namespace analytic_test
