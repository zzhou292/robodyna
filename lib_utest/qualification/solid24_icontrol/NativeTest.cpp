// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeSupport.h"
namespace solid24_icontrol_test {
TEST(Solid24IcontrolNative, ReaderAndUpdateSlotsRetainDistinctPressureChannels) {
  const auto material=heph_test::Material();const double p[]{material.mu_pa,material.poisson_ratio,material.density_kg_m3,material.tension_cutoff_pa};
  double slots[6];ic1_native_slots(p,slots);
  EXPECT_DOUBLE_EQ(slots[0],(material.mu_pa*2)*(1+material.poisson_ratio));
  EXPECT_DOUBLE_EQ(slots[1],material.poisson_ratio);
  EXPECT_DOUBLE_EQ(slots[4],(material.mu_pa*2)*(1+material.poisson_ratio)/
    std::max(1e-20,3*(1-2*material.poisson_ratio)));
  EXPECT_DOUBLE_EQ(slots[2],2*material.mu_pa);EXPECT_DOUBLE_EQ(slots[3],slots[2]);
  EXPECT_GT(slots[4],slots[3]);EXPECT_DOUBLE_EQ(slots[5],2*slots[4]);
}
TEST(Solid24IcontrolNative, UndeformedAndUniformTranslationCompleteEveryNativeStage) {
  const auto reference=heph_test::Reference();const auto material=heph_test::Material(reference.input().density_kg_m3);
  History history(reference.input());s::History ignored;ASSERT_EQ(s::InitializeHistory(reference,material,ignored),s::ForceStatus::Success);
  auto interval=heph_test::Interval(reference,ignored);interval.dt_s=0;
  const auto initial=Step(history,interval,material);Ready(initial);ASSERT_FALSE(HasFailure());
  EXPECT_LT(ForceNorm(initial),1e-5);EXPECT_EQ(initial.values[21],0.);
  interval.dt_s=1e-6;
  for(unsigned n=0;n<8;++n) {interval.velocity_m_s[n]={1,-2,.5};interval.position_m[n].x+=1e-6;interval.position_m[n].y-=2e-6;interval.position_m[n].z+=.5e-6;}
  const auto translated=Step(history,interval,material);Ready(translated);ASSERT_FALSE(HasFailure());
  EXPECT_LT(ForceNorm(translated),1e-4);EXPECT_DOUBLE_EQ(translated.values[92],interval.dt_s);
}
TEST(Solid24IcontrolNative, ControlledHourglassCarriesForceHistoryAndNativeEnergy) {
  const auto reference=heph_test::Reference();const auto material=heph_test::Material(reference.input().density_kg_m3);
  History history(reference.input());s::History ignored;ASSERT_EQ(s::InitializeHistory(reference,material,ignored),s::ForceStatus::Success);
  auto interval=heph_test::Interval(reference,ignored);const double signs[]{1,-1,1,-1,-1,1,-1,1};
  for(unsigned n=0;n<8;++n)interval.velocity_m_s[history.reference.reference.permutation[n]].z=signs[n];
  const auto first=Step(history,interval,material);Ready(first);ASSERT_FALSE(HasFailure());
  double norm=0;for(unsigned i=9;i<21;++i)norm+=first.values[i]*first.values[i];EXPECT_GT(norm,0.);
  EXPECT_GT(first.values[90],0.);EXPECT_GT(first.values[7],first.values[53]);EXPECT_GT(ForceNorm(first),0.);
  std::copy_n(first.values.begin(),22,history.values.begin());interval.base_time_s=interval.dt_s;
  for(auto& velocity:interval.velocity_m_s)velocity={};
  const auto carried=Step(history,interval,material);Ready(carried);ASSERT_FALSE(HasFailure());
  EXPECT_GT(ForceNorm(carried),0.);for(unsigned i=9;i<21;++i)EXPECT_DOUBLE_EQ(carried.values[i],history.values[i]);
}
TEST(Solid24IcontrolNative, EveryControlledHistorySlotPreservesExplicitNativePacking) {
  const auto reference=heph_test::Reference();const auto material=heph_test::Material(reference.input().density_kg_m3);
  History history(reference.input());s::History ignored;ASSERT_EQ(s::InitializeHistory(reference,material,ignored),s::ForceStatus::Success);
  for(unsigned component=0;component<3;++component)for(unsigned mode=0;mode<4;++mode)
    history.values[9+4*component+mode]=(component%2?-1.:1.)*(.125+component+mode*.25);
  const auto trial=Step(history,heph_test::Interval(reference,ignored),material);Ready(trial);ASSERT_FALSE(HasFailure());
  for(unsigned component=0;component<3;++component)for(unsigned mode=0;mode<4;++mode)
    EXPECT_DOUBLE_EQ(trial.values[9+4*component+mode],history.values[9+4*component+mode]);
  EXPECT_GT(ForceNorm(trial),0.);
}
TEST(Solid24IcontrolNative, FoldedPositiveVolumeCellExercisesDistortionEnergyAndAssembly) {
  const auto reference=heph_test::Reference();const auto material=heph_test::Material(reference.input().density_kg_m3);
  History history(reference.input());s::History ignored;ASSERT_EQ(s::InitializeHistory(reference,material,ignored),s::ForceStatus::Success);
  auto interval=heph_test::Interval(reference,ignored);const auto a=history.reference.reference.permutation[0],b=history.reference.reference.permutation[6];
  bool exercised=false,geometric=false;
  for(double fraction:{1.0005,1.005,1.01}) {
    interval=heph_test::Interval(reference,ignored);const auto first=reference.input().position_m[a],opposite=reference.input().position_m[b];
    interval.position_m[a]={first.x+.5*(opposite.x-first.x),first.y+.5*(opposite.y-first.y),first.z+fraction*(opposite.z-first.z)};
    interval.velocity_m_s[a]={0,0,(opposite.z-first.z)*.1};
    interval.velocity_m_s[history.reference.reference.permutation[1]]={0,0,-.05*(opposite.z-first.z)};
    const auto trial=Step(history,interval,material);Ready(trial);ASSERT_FALSE(HasFailure());
    exercised=exercised||trial.values[21]!=0.;
    geometric=geometric||(trial.center_contacts+trial.corner_contacts)>0;
    for(unsigned axis=0;axis<3;++axis) {
      double sum=0,scale=0;for(unsigned n=0;n<8;++n){const auto f=trial.values[22+3*n+axis];sum+=f;scale+=std::abs(f);}
      EXPECT_LE(std::abs(sum),1e-11*std::max(1.,scale));
    }
  }
  EXPECT_TRUE(exercised);EXPECT_TRUE(geometric);
}
TEST(Solid24IcontrolNative, HydrostaticAndUnequalDiagonalStressReachDistortionWithoutQvis) {
  const auto reference=heph_test::Reference();const auto material=heph_test::Material(reference.input().density_kg_m3);
  History history(reference.input());s::History ignored;ASSERT_EQ(s::InitializeHistory(reference,material,ignored),s::ForceStatus::Success);
  for(bool hydro:{true,false}) {
    auto interval=heph_test::Interval(reference,ignored);const double scale[]{hydro?.9:.8,hydro?.9:.95,hydro?.9:1.05};
    for(unsigned n=0;n<8;++n) {
      const auto x=reference.input().position_m[n];interval.position_m[n]={scale[0]*x.x,scale[1]*x.y,scale[2]*x.z};
      interval.velocity_m_s[n]={-.25*x.x,-.25*x.y,-.25*x.z};
    }
    const auto trial=Step(history,interval,material);Ready(trial);ASSERT_FALSE(HasFailure());
    EXPECT_GT(trial.values[54],0.);EXPECT_EQ(trial.values[8],trial.values[54]);
    for(unsigned i=0;i<6;++i)EXPECT_EQ(trial.distortion_sigma[i],trial.values[46+i]);
    if(hydro) {
      EXPECT_LT(trial.distortion_sigma[0],0.);
      EXPECT_NEAR(trial.distortion_sigma[0],trial.distortion_sigma[1],1e-10*std::abs(trial.distortion_sigma[0]));
      EXPECT_NEAR(trial.distortion_sigma[0],trial.distortion_sigma[2],1e-10*std::abs(trial.distortion_sigma[0]));
    } else {EXPECT_NE(trial.distortion_sigma[0],trial.distortion_sigma[1]);EXPECT_NE(trial.distortion_sigma[1],trial.distortion_sigma[2]);}
  }
}
TEST(Solid24IcontrolNative, SourceStressCriterionIncludesHydrostaticMagnitude) {
  const auto m=heph_test::Material();const double p[]{m.mu_pa,m.poisson_ratio,m.density_kg_m3,m.tension_cutoff_pa};
  const double kin[]{1980.,500.,.02*.03*.04};double zero[6]{},hydro[]{-1e5,-1e5,-1e5,0,0,0},large[]{-1e12,0,0,0,0,0};
  double a[4],b[4],c[4];int ia=-1,ib=-1,ic=-1;
  ic1_native_parameter_probe(p,zero,kin,a,&ia);ic1_native_parameter_probe(p,hydro,kin,b,&ib);ic1_native_parameter_probe(p,large,kin,c,&ic);
  EXPECT_EQ(ia,0);EXPECT_EQ(ib,0);EXPECT_EQ(ic,1);EXPECT_GT(b[1],a[1]);EXPECT_GT(c[1],b[1]);
  EXPECT_EQ(a[0],b[0]);EXPECT_EQ(b[0],c[0]);EXPECT_EQ(a[2],b[2]);EXPECT_EQ(a[3],b[3]);
}
TEST(Solid24IcontrolNative, DampingVelocityCriterionDistinguishesZeroAndNonzeroMeanModes) {
  const auto reference=heph_test::Reference();const auto material=heph_test::Material(reference.input().density_kg_m3);
  History history(reference.input());s::History ignored;ASSERT_EQ(s::InitializeHistory(reference,material,ignored),s::ForceStatus::Success);
  const double signs[]{1,-1,1,-1,-1,1,-1,1};double energy[2]{};
  for(unsigned variant=0;variant<2;++variant) {
    auto interval=heph_test::Interval(reference,ignored);
    for(unsigned n=0;n<8;++n)interval.velocity_m_s[history.reference.reference.permutation[n]].z=signs[n]+(variant?.01:0.);
    const auto trial=Step(history,interval,material);Ready(trial);ASSERT_FALSE(HasFailure());
    EXPECT_EQ(trial.distortion_flag,0);EXPECT_EQ(trial.center_contacts+trial.corner_contacts,0);
    energy[variant]=trial.values[21];
  }
  EXPECT_EQ(energy[0],0.);EXPECT_GT(energy[1],0.);
}
TEST(Solid24IcontrolNative, LateNonfiniteStageRejectsOutputAndRepairs) {
  const auto reference=heph_test::Reference();const auto material=heph_test::Material(reference.input().density_kg_m3);
  History history(reference.input());s::History ignored;ASSERT_EQ(s::InitializeHistory(reference,material,ignored),s::ForceStatus::Success);
  for(unsigned i=9;i<21;++i)history.values[i]=std::numeric_limits<double>::max();
  const auto interval=heph_test::Interval(reference,ignored);const auto failed=Step(history,interval,material);
  EXPECT_NE(failed.status,0);EXPECT_EQ(failed.stages,native::RequiredStages);
  for(auto value:failed.values)EXPECT_EQ(value,-9876.25);
  for(unsigned i=9;i<21;++i)history.values[i]=0;
  const auto repaired=Step(history,interval,material);Ready(repaired);ASSERT_FALSE(HasFailure());
}
TEST(Solid24IcontrolNative, UnsupportedOrNonfiniteInputsDoNotPublishNumericalOutput) {
  const auto reference=heph_test::Reference();auto material=heph_test::Material(reference.input().density_kg_m3);
  History history(reference.input());s::History ignored;ASSERT_EQ(s::InitializeHistory(reference,material,ignored),s::ForceStatus::Success);
  auto interval=heph_test::Interval(reference,ignored);interval.velocity_m_s[0].x=std::numeric_limits<double>::quiet_NaN();
  auto bad=Step(history,interval,material);EXPECT_NE(bad.status,0);EXPECT_EQ(bad.stages,0);
  for(auto value:bad.values)EXPECT_EQ(value,-9876.25);
  interval=heph_test::Interval(reference,ignored);material.poisson_ratio=.495;
  bad=Step(history,interval,material);EXPECT_NE(bad.status,0);EXPECT_EQ(bad.stages,0);
  for(auto value:bad.values)EXPECT_EQ(value,-9876.25);
}
} // namespace solid24_icontrol_test
