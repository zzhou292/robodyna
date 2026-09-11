// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
namespace t3_one_point_test {
TEST(T3OnePointNative, OriginalTriangleYieldUnloadReloadCarriesIndependentFullHistory) {
  for(bool transformed:{false,true}) {
    Fixture f(transformed);
    auto h=f.Virgin();
    NativeState native(h);
    double peak_pla=0,minimum_work=0,thickness_excursion=0;
    for(unsigned step=0;step<256;++step) {
      SCOPED_TRACE(step);
      const auto in=Path(f,step);
      t3::OnePointForceTrial out;
      ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,h,in,out),t3::Status::kSuccess);
      ASSERT_EQ(native.Step(f,in),0);
      CompareNative(out,native);
      h=out.proposed_history;
      peak_pla=std::max(peak_pla,native.state[31]);
      minimum_work=std::min(minimum_work,native.output[101]);
      thickness_excursion=std::max(thickness_excursion,std::abs(native.state[21]-.0005));
    }
    EXPECT_GT(peak_pla,.05);
    EXPECT_LT(minimum_work,0);
    EXPECT_GT(thickness_excursion,1e-6);
  }
}
TEST(T3OnePointNative, NativeSinglePointRemovalCurrentWorkAndFollowingZeroCache) {
  Fixture f;
  auto h=NearFailure(f);
  NativeState native(h);
  for(unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step);
    t3::OnePointForceTrial out;
    ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,h,Path(f,step),out),t3::Status::kSuccess);
    ASSERT_EQ(native.Step(f,Path(f,step)),0);
    CompareNative(out,native);
    EXPECT_EQ(native.state[25],0);
    EXPECT_EQ(native.state[34],Dt);
    if(step==0) {
      EXPECT_GT(native.output[101],0);
      EXPECT_GT(native.output[124],0);
      EXPECT_GT(std::abs(native.output[103]),1e3);
    } else EXPECT_EQ(native.output[101],0);
    for(unsigned i=75;i<93;++i) EXPECT_EQ(native.output[i],0);
    h=out.proposed_history;
  }
}
TEST(T3OnePointNative, ResetHistoryAndWrongVelocityPhaseAreDecisiveNegativeControls) {
  Fixture f;
  auto h=f.Virgin();
  NativeState native(h);
  for(unsigned step=0;step<40;++step) {
    t3::OnePointForceTrial out;
    ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,h,Path(f,step),out),t3::Status::kSuccess);
    ASSERT_EQ(native.Step(f,Path(f,step)),0);
    CompareNative(out,native);
    h=out.proposed_history;
  }
  const auto before=native;
  const auto in=Path(f,40);
  ASSERT_EQ(native.Step(f,in),0);
  NativeState reset(f.Virgin());
  ASSERT_EQ(reset.Step(f,in),0);
  EXPECT_GT(std::abs(native.state[31]-reset.state[31]),.01);
  auto wrong=before;
  auto wrong_phase=in;
  const auto next_phase=Path(f,41);
  for(unsigned i=0;i<3;++i) {
    wrong_phase.velocity[i]=next_phase.velocity[i];
    wrong_phase.angular_velocity[i]=next_phase.angular_velocity[i];
  }
  ASSERT_EQ(wrong.Step(f,wrong_phase),0);
  double force_difference=0;
  for(unsigned i=75;i<84;++i) force_difference=std::max(force_difference,std::abs(native.output[i]-wrong.output[i]));
  EXPECT_GT(force_difference,1e-4);
  // Native replay of the same accepted state is exact; comparison is over named values.
  auto retry=before;
  ASSERT_EQ(retry.Step(f,in),0);
  EXPECT_EQ(retry.state,native.state);
  EXPECT_EQ(retry.output,native.output);
}
TEST(T3OnePointNative, SharedZeroShearUsesNativeTablePositiveRateAndRateOffBranches) {
  const double strain[]{0,1,4};
  const double stress[]{10e6,12e6,18e6};
  for(bool rate_enabled:{false,true}) {
    Fixture f;
    mat::TabulatedShellPlasticityRate rate;
    if(rate_enabled) rate={true,8000,8,10000};
    ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(250e6,.35,1000,{strain,stress,3},rate,f.material),
        mat::TabulatedShellPlasticityStatus::Ok);
    auto h=f.Virgin();
    NativeState native(h);
    for(unsigned step=0;step<128;++step) {
      SCOPED_TRACE(step);
      t3::OnePointForceTrial out;
      ASSERT_EQ(t3::EvaluateOnePointLaw44Force(f.reference,f.material,f.failure,h,Path(f,step),out),t3::Status::kSuccess);
      ASSERT_EQ(native.Step(f,Path(f,step)),0);
      CompareNative(out,native);
      h=out.proposed_history;
    }
  }
}
extern "C" void law44_point_physical(int,const double*,double,double,double,double,
    const double*,const double*,const double*,double,double,double*);
TEST(T3OnePointNative, GeneralGsZeroNativePointKeepsNonzeroShearPacketAndCarriedStress) {
  const double strain[]{0,.2,1};
  const double yield[]{10e6,11e6,13e6};
  t3::OnePointMaterial material;
  ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(250e6,.35,1000,{strain,yield,3},material),
      mat::TabulatedShellPlasticityStatus::Ok);
  mat::TabulatedShellPlasticityHistory history;
  history.stress[3]=17;
  history.stress[4]=-9;
  mat::TabulatedShellPlasticityInput input;
  input.dt=Dt;
  input.strain_increment[0]=.06;
  input.strain_increment[3]=.13;
  input.strain_increment[4]=-.09;
  mat::TabulatedShellPlasticityResult actual;
  ASSERT_EQ(mat::UpdateLaw44ZeroShearPlasticity(material,history,input,actual),
      mat::TabulatedShellPlasticityStatus::Ok);
  const double curve[]{0,0,0,10e6,.2,11e6,1,13e6};
  const double base[]{0,0,0,17,-9,0};
  const double rate[]{0,0,0,1,0};
  double native[13]{};
  law44_point_physical(3,curve,250e6,.35,1000,0,base,input.strain_increment,rate,
      .0005,.0005,native);
  for(unsigned i=0;i<5;++i) Close(actual.history.stress[i],native[i],1e-6);
  Close(actual.history.plastic_strain,native[5]);
  Close(actual.plastic_increment,native[6]);
  Close(actual.tangent_ratio,native[7]);
  EXPECT_DOUBLE_EQ(native[3],17);
  EXPECT_DOUBLE_EQ(native[4],-9);
  const auto prior=Bytes(actual);
  EXPECT_NE(mat::UpdateLaw44MembranePlasticity(material,history,input,actual),
      mat::TabulatedShellPlasticityStatus::Ok);
  EXPECT_EQ(Bytes(actual),prior);
  EXPECT_NE(mat::UpdateLaw44ShellPlasticity(material,history,input,actual),
      mat::TabulatedShellPlasticityStatus::Ok);
  EXPECT_EQ(Bytes(actual),prior);
}
} // namespace t3_one_point_test
