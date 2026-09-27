// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
namespace controlled_test {
TEST(ControlledHourglass, EveryHistoryAndProjectionSlotMatchesCompleteNative) {
  for(const auto& x:BasisCases())Check(x);
}
TEST(ControlledHourglass, ZeroTimeTranslationAndZeroStiffnessRemainNativeQueries) {
  auto x=Moving();x.input.dt_s=0;x.input.raw_stiffness_n_m=0;
  for(auto& v:x.input.local_velocity_m_s)v={.25,-.125,.5};
  const auto y=Check(x);EXPECT_EQ(y.work_j,0);EXPECT_EQ(y.raw_stiffness_n_m,0);
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)EXPECT_EQ(y.proposed_state.force_n[k][h],x.state.force_n[k][h]);
}
TEST(ControlledHourglass, NativeRealLiteralAndSoundSpeedStiffeningBoundaries) {
  for(double nu:{0.,.4,.463,c::MaximumPoissonRatio}) {
    auto x=Moving();x.input.poisson_ratio=nu;
    const double gs=x.input.mu_pa*2,e0=gs*(1+nu),g0=.5*gs,c1=(1./3)*e0/(1-2*nu);
    for(double f:{.5,1.,std::nextafter(1.,2.),4.,16.,std::nextafter(16.,32.),25.}) {
      x.input.material_sound_speed_m_s=std::sqrt((c1+(4./3)*g0*f)/x.input.density_kg_m3);Check(x);
    }
  }
  auto x=Base();const double signs[]{1,1,-1,-1,-1,-1,1,1};
  for(unsigned n=0;n<8;++n)x.input.local_velocity_m_s[n].x=signs[n];
  x.input.material_sound_speed_m_s=1;
  const auto y=Check(x);const double gs=x.input.mu_pa*2;
  const double lamg=(1./3)*(gs*(1+x.input.poisson_ratio))/(1-2*x.input.poisson_ratio)+(4./3)*(.5*gs);
  const double expected=(double(.3f)*1*lamg*x.input.dt_s)*std::pow(x.input.current_volume_m3,1./3);
  EXPECT_DOUBLE_EQ(y.proposed_state.force_n[0][0],expected);
  const double wrong=(.3*lamg*x.input.dt_s)*std::pow(x.input.current_volume_m3,1./3);
  EXPECT_GT(std::abs(expected-wrong),1e-9*std::abs(expected));
}
TEST(ControlledHourglass, MultistepLoadingUnloadingCarriesNativeStateAndSignedWork) {
  auto actual=Moving(),native=actual;bool negative=false;
  for(unsigned step=0;step<32;++step) {
    const double sign=(step%8<4)?1.:-1.;
    for(unsigned n=0;n<8;++n) {
      actual.input.local_velocity_m_s[n]={sign*.3*(n%2?1:-1),sign*.17*(n%3?1:-2),sign*.23*(n%4?1:-3)};
      native.input.local_velocity_m_s[n]=actual.input.local_velocity_m_s[n];
    }
    actual.input.material_sound_speed_m_s=300+13*step;native.input.material_sound_speed_m_s=actual.input.material_sound_speed_m_s;
    c::Result result;ASSERT_EQ(c::EvaluateLaw42(actual.input,actual.state,result),c::Status::Success);
    const auto expected=Native(native);Compare(result,expected);negative=negative||result.work_j<0;
    Accept(actual,result);AcceptNative(native,expected);
  }
  EXPECT_TRUE(negative);
}
TEST(ControlledHourglass, FourthModeUsesNativeRateScatterAndModalWork) {
  auto x=Base();const double signs[]{1,-1,1,-1,-1,1,-1,1};
  for(unsigned n=0;n<8;++n)x.input.local_velocity_m_s[n].z=signs[n];
  const auto y=Check(x);EXPECT_DOUBLE_EQ(y.modal_velocity_m_s[2][3],1./8.);
  for(unsigned h=0;h<3;++h)EXPECT_EQ(y.modal_velocity_m_s[2][h],0);
  EXPECT_DOUBLE_EQ(y.work_j,x.input.dt_s*(y.modal_force_n[2][3]*(1./8.)));
  for(unsigned n=0;n<8;++n)EXPECT_DOUBLE_EQ(y.local_force_n[n].z,-signs[n]*y.modal_force_n[2][3]);
  // Native modal work is its own observation, not a reconstructed nodal ledger.
}
TEST(ControlledHourglass, WorkObservationSurvivesEnergyCancellation) {
  auto x=Moving();x.input.internal_energy_density_j_m3=1e30;
  const auto y=Check(x);EXPECT_EQ(y.internal_energy_density_j_m3,x.input.internal_energy_density_j_m3);
  EXPECT_NE(y.work_j,0);
}
TEST(ControlledHourglass, RejectionAndLateOverflowPreserveEveryOutputAndInputField) {
  const auto valid=Moving();c::Result sentinel=Check(valid);const auto before=Values(sentinel);
  std::vector<Case> cases;
  auto x=valid;x.input.poisson_ratio=std::nextafter(c::MaximumPoissonRatio,1.);cases.push_back(x);
  x=valid;x.input.poisson_ratio=.48999;cases.push_back(x);
  x=valid;x.input.poisson_ratio=-.01;cases.push_back(x);
  x=valid;x.input.dt_s=-1;cases.push_back(x);
  x=valid;x.input.reference_volume_m3=0;cases.push_back(x);
  x=valid;x.input.raw_stiffness_n_m=-1;cases.push_back(x);
  x=valid;x.input.projection[3][2]=std::numeric_limits<double>::quiet_NaN();cases.push_back(x);
  x=valid;x.input.local_velocity_m_s[7].z=std::numeric_limits<double>::infinity();cases.push_back(x);
  x=valid;x.state.force_n[2][3]=std::numeric_limits<double>::max();cases.push_back(x);
  for(const auto& item:cases){const auto old=item.state;EXPECT_NE(c::EvaluateLaw42(item.input,item.state,sentinel),c::Status::Success);EXPECT_EQ(Values(sentinel),before);
    for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)EXPECT_EQ(item.state.force_n[k][h],old.force_n[k][h]);}
  ASSERT_EQ(c::EvaluateLaw42(valid.input,valid.state,sentinel),c::Status::Success);EXPECT_EQ(Values(sentinel),before);
}
} // namespace controlled_test
