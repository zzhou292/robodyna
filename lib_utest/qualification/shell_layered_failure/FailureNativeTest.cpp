#include "FailureNativeFixture.h"
#include <gtest/gtest.h>
#include <cmath>

namespace layered_failure_test {
TEST(LayeredFailureNative, InactiveBiaxialPredictorRetainsPostViscosityZeroSigns) {
  const auto p=Parameters();auto in=Input(p);
  in.strain_curvature_increment[0]=.0004;
  in.strain_curvature_increment[1]=-.001;
  const auto base=Seed(7);auto native=NativeSeed(base);NativeTrace trace;WorkHistory work;
  NativeStep(p,1.,in,2.e-6,.01,.015,native,trace);
  sec::ShellLayeredJ2FailureResult result;
  ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.},base,in,2.e-6,result),sec::PointStatus::Ok);
  ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(result,in.strain_curvature_increment,
      in.reference_thickness,.01,trace.diagnostics[8],work));
  Compare(result,work,native,trace,in.reference_thickness,.01);
  const double viscous=trace.diagnostics[8]*(.0004+.5*(-.001));
  const double raw=.25*trace.point_values[0]+.5*trace.point_values[13]+.25*trace.point_values[26];
  ASSERT_GT(raw,0.);ASSERT_LT(viscous,0.);ASSERT_GT(raw+viscous,0.);
  EXPECT_FALSE(std::signbit(native.stress[0]));
  EXPECT_TRUE(std::signbit((raw*0.+viscous)*0.)); // Detect the rejected pre-mask order.
  for(unsigned c=0;c<5;++c) {
    EXPECT_EQ(std::signbit(work.stress[c]),std::signbit(native.stress[c]));
    EXPECT_EQ(std::signbit(work.material_stress[c]),std::signbit(native.material[c]));
  }
}
TEST(LayeredFailureNative, EverySubsetAndLocalInactivePointMatchesCompleteNativeLaw) {
  for(bool analytic:{false,true}) for(bool rate:{false,true}) for(unsigned mask=0;mask<8;++mask) {
    if(analytic&&!rate) continue; // Existing analytic source admission requires positive C/P.
    SCOPED_TRACE(analytic);
    SCOPED_TRACE(rate);
    SCOPED_TRACE(mask);
    const auto p=Parameters(analytic,rate);auto in=Input(p);
    in.strain_curvature_increment[0]=.004;in.strain_curvature_increment[1]=-.001;
    in.strain_curvature_increment[2]=.0007;in.strain_curvature_increment[3]=.0003;
    in.strain_curvature_increment[4]=-.0002;in.strain_curvature_increment[5]=1.3;
    in.strain_curvature_increment[6]=-.4;in.strain_curvature_increment[7]=.2;
    auto state=Seed(mask);auto native=NativeSeed(state);WorkHistory work;
    for(unsigned step=2;step<=4;++step) {
      NativeTrace trace;const double time=step*in.dt;
      NativeStep(p,1.,in,time,.01,.015,native,trace);
      sec::ShellLayeredJ2FailureResult result;
      ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.},state,in,time,result),sec::PointStatus::Ok);
      ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(result,in.strain_curvature_increment,
          in.reference_thickness,.01,trace.diagnostics[8],work));
      Compare(result,work,native,trace,in.reference_thickness,.01);
      state=result.history;
      in.reference_thickness=in.reported_thickness=result.current.reported_thickness;
    }
  }
}

TEST(LayeredFailureNative, IndependentRecurrencesReachRemovalAndRetainPostRemovalHistory) {
  for(bool analytic:{false,true}) for(bool rate:{false,true}) {
    if(analytic&&!rate) continue;
    SCOPED_TRACE(analytic);
    SCOPED_TRACE(rate);
    const auto p=Parameters(analytic,rate);auto in=Input(p);
    in.strain_curvature_increment[0]=.002;in.strain_curvature_increment[1]=-.0006;
    in.strain_curvature_increment[5]=1.4;
    sec::ShellLayeredJ2FailureHistory state;NativeState native;WorkHistory work;
    unsigned removed=0,post_removed=0,partial=0;
    for(unsigned step=1;step<=40;++step) {
      NativeTrace trace;const double time=step*in.dt;
      NativeStep(p,.004,in,time,.01,.015,native,trace);
      sec::ShellLayeredJ2FailureResult result;
      ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{.004},state,in,time,result),sec::PointStatus::Ok);
      ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(result,in.strain_curvature_increment,
          in.reference_thickness,.01,trace.diagnostics[8],work));
      Compare(result,work,native,trace,in.reference_thickness,.01);
      if(result.removed_now) ++removed;
      else if(!result.history.element_active) ++post_removed;
      else if(sec::ShellNip3FailedThickness(result.history.failure)>0) ++partial;
      state=result.history;
      in.reference_thickness=in.reported_thickness=result.current.reported_thickness;
      if(post_removed==2) break;
    }
    EXPECT_EQ(removed,1u);EXPECT_EQ(post_removed,2u);EXPECT_GT(partial,0u);
  }
}

TEST(LayeredFailureNative, RoundedCallerIncrementCanBeZeroWithPositiveConstitutiveIncrement) {
  sec::PointParameters p;
  const double x[]{0,1.e12+1},y[]{220e6,220e6};
  ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{x,y,2},p),sec::PointStatus::Ok);
  auto in=Input(p);in.strain_curvature_increment[0]=.001101;
  in.strain_curvature_increment[1]=-.3*in.strain_curvature_increment[0];
  sec::ShellLayeredJ2FailureHistory base;
  for(auto& point:base.saved.point) point.plastic_strain=1.e12;
  auto native=NativeSeed(base);NativeTrace trace;WorkHistory work;
  NativeStep(p,.01,in,1.e-6,.01,0,native,trace);
  sec::ShellLayeredJ2FailureResult result;
  ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{.01},base,in,1.e-6,result),sec::PointStatus::Ok);
  ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(result,in.strain_curvature_increment,.002,.01,0,work));
  Compare(result,work,native,trace,.002,.01);
  for(unsigned point=0;point<3;++point) {
    EXPECT_GT(result.constitutive_increment[point],0);
    EXPECT_EQ(result.caller_failure_increment[point],0);
    EXPECT_EQ(result.history.failure[point].damage,0);
  }
  EXPECT_EQ(result.current.diagnostics.plastic_work_density_increment,0);
  EXPECT_TRUE(result.history.element_active);
}
} // namespace layered_failure_test
