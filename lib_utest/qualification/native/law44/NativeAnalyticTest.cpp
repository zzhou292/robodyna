#include "NativeAnalyticTestSupport.h"
#include <limits>

namespace analytic_test {
TEST(Law44AnalyticNative, OriginalModulusAndVirginVsYieldedSourceBranches) {
  // The last coefficient tuple is the supported synthetic ETAN=0 boundary.
  for(const auto source:std::array<Source,4>{Sources[0],Sources[1],Sources[2],Source{1000,20,0,1415,0}})
    for(double pla:{0.,.001}) {
    SCOPED_TRACE(source.mid);
    SCOPED_TRACE(pla);
    const auto p=Prepare(source); const auto in=Increment(p);
    auto oracle=Native(source); oracle.point.strain_increment[0]=in.strain_increment[0];
    oracle.point.rate.total_shell_rate_per_s=in.total_strain_rate_per_s;
    oracle.point.accepted_plastic_strain=pla;
    native::AnalyticResult expected;
    ASSERT_TRUE(native::EvaluateAnalytic(oracle,.0005,.002,expected));
    History accepted; accepted.plastic_strain=pla; Result result;
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,accepted,in,result),Status::Ok);
    Compare(result,expected);
    EXPECT_DOUBLE_EQ(p.plastic_hardening_pa,expected.native_plastic_hardening);
    ASSERT_GT(expected.plastic_increment,0); if(pla==0) EXPECT_DOUBLE_EQ(expected.tangent_ratio,.5);
    double thickness=.002;
    thickness=thickness+result.elastic_thickness_strain*.0005;
    thickness=thickness+result.plastic_thickness_strain*.0005;
    Close(thickness,expected.reported_thickness_m,2e-14);
  }
}
TEST(Law44AnalyticNative, DefaultCapBranchIsRejectedWithoutPublishingAShellFailure) {
  auto oracle=Native(); const auto p=Prepare();
  native::AnalyticResult expected; ASSERT_TRUE(native::EvaluateAnalytic(oracle,.0005,.002,expected));
  const double cap=(static_cast<double>(1e20f)-oracle.initial_yield)/expected.native_plastic_hardening;
  oracle.point.accepted_plastic_strain=std::nextafter(cap,0.);
  ASSERT_TRUE(native::EvaluateAnalytic(oracle,.0005,.002,expected));
  History accepted; accepted.plastic_strain=oracle.point.accepted_plastic_strain;
  auto input=Increment(p); input.strain_increment[0]=0; input.total_strain_rate_per_s=0;
  Result actual; ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,accepted,input,actual),Status::Ok);
  Compare(actual,expected);
  const auto before=Bytes(actual);
  const auto native_before=Bytes(expected);
  accepted.plastic_strain=cap; oracle.point.accepted_plastic_strain=cap;
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,accepted,input,actual),Status::HardeningDomainExceeded);
  EXPECT_EQ(Bytes(actual),before);
  EXPECT_FALSE(native::EvaluateAnalytic(oracle,.0005,.002,expected)); EXPECT_EQ(Bytes(expected),native_before);
}
TEST(Law44AnalyticNative, IndependentLoadHoldReverseHistoriesAndPhysicalThickness) {
  for(const auto source:Sources) {
    auto oracle=Native(source); const auto p=Prepare(source); History accepted;
    double actual_thickness=.002,native_thickness=.002,total_work=0,reverse_flow=0,reset_difference=0;
    unsigned plastic_steps=0; double rate_before_hold=0,rate_after_hold=0;
    for(unsigned step=0;step<1536;++step) {
      SCOPED_TRACE(source.mid);
      SCOPED_TRACE(step);
      const double sign=step<640?1.:step<768?0.:-1.;
      auto& point=oracle.point;
      point.strain_increment={sign*1e-4,sign*-3e-5,sign*2e-5,sign*1e-6,sign*-2e-6};
      std::array<double,8> dx{}; std::copy(point.strain_increment.begin(),point.strain_increment.end(),dx.begin());
      point.rate.total_shell_rate_per_s=native::NativeShellRate(dx,native_thickness,Dt);
      native::AnalyticResult expected;
      ASSERT_TRUE(native::EvaluateAnalytic(oracle,.25*native_thickness,native_thickness,expected));
      Input increment; std::copy(point.strain_increment.begin(),point.strain_increment.end(),increment.strain_increment);
      increment.transverse_shear_modulus=point.transverse_shear_modulus; increment.dt=Dt;
      increment.total_strain_rate_per_s=point.rate.total_shell_rate_per_s;
      Result result; ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,accepted,increment,result),Status::Ok);
      Compare(result,expected);
      const double layer=.25*actual_thickness;
      actual_thickness=actual_thickness+result.elastic_thickness_strain*layer;
      actual_thickness=actual_thickness+result.plastic_thickness_strain*layer;
      Close(actual_thickness,expected.reported_thickness_m,2e-14);
      if(step==639) rate_before_hold=expected.filtered_rate_per_s;
      if(step==767) rate_after_hold=expected.filtered_rate_per_s;
      if(step>1000) reverse_flow+=expected.plastic_increment;
      plastic_steps+=expected.plastic_increment>0; total_work+=expected.plastic_work_density*layer;
      if(step==600) {
        auto reset=oracle; reset.point.accepted_stress={}; reset.point.accepted_plastic_strain=0;
        reset.point.rate.accepted_filtered_rate_per_s=0; native::AnalyticResult wrong;
        ASSERT_TRUE(native::EvaluateAnalytic(reset,.25*native_thickness,native_thickness,wrong));
        reset_difference=std::abs(wrong.stress[0]-expected.stress[0]);
      }
      accepted=result.history; Accept(oracle,expected); native_thickness=expected.reported_thickness_m;
    }
    EXPECT_GT(plastic_steps,100u); EXPECT_GT(reverse_flow,.001); EXPECT_GT(total_work,0);
    EXPECT_LT(rate_after_hold,rate_before_hold*.01); EXPECT_GT(reset_difference,1e6);
    EXPECT_GT(std::abs(actual_thickness-.002),1e-6);
  }
}
TEST(Law44AnalyticNative, InvalidInputsAndVanishedNativeThicknessPreserveOutput) {
  auto input=Native(); input.point.strain_increment[0]=.05; native::AnalyticResult output;
  ASSERT_TRUE(native::EvaluateAnalytic(input,.0005,.002,output)); const auto old=Bytes(output);
  auto bad=input; bad.tangent_modulus=bad.point.young;
  EXPECT_FALSE(native::EvaluateAnalytic(bad,.0005,.002,output)); EXPECT_EQ(Bytes(output),old);
  bad=input; bad.point.rate.active=false;
  EXPECT_FALSE(native::EvaluateAnalytic(bad,.0005,.002,output)); EXPECT_EQ(Bytes(output),old);
  bad=input; bad.point.point_count=2;
  EXPECT_FALSE(native::EvaluateAnalytic(bad,.0005,.002,output)); EXPECT_EQ(Bytes(output),old);
  bad=input; bad.point.accepted_stress[4]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(native::EvaluateAnalytic(bad,.0005,.002,output)); EXPECT_EQ(Bytes(output),old);
  EXPECT_FALSE(native::EvaluateAnalytic(input,1.,1e-12,output)); EXPECT_EQ(Bytes(output),old);
  ASSERT_TRUE(native::EvaluateAnalytic(input,.0005,.002,output)); EXPECT_EQ(Bytes(output),old);
}
} // namespace analytic_test
