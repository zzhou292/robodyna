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
  bool exercised=false;
  for(double fraction:{1.0005,1.005,1.01}) {
    interval=heph_test::Interval(reference,ignored);const auto first=reference.input().position_m[a],opposite=reference.input().position_m[b];
    interval.position_m[a]={first.x+fraction*(opposite.x-first.x),first.y+fraction*(opposite.y-first.y),first.z+fraction*(opposite.z-first.z)};
    interval.velocity_m_s[a]={(opposite.x-first.x)*.1,(opposite.y-first.y)*.1,(opposite.z-first.z)*.1};
    const auto trial=Step(history,interval,material);Ready(trial);ASSERT_FALSE(HasFailure());
    exercised=exercised||trial.values[21]!=0.;
    for(unsigned axis=0;axis<3;++axis) {
      double sum=0,scale=0;for(unsigned n=0;n<8;++n){const auto f=trial.values[22+3*n+axis];sum+=f;scale+=std::abs(f);}
      EXPECT_LE(std::abs(sum),1e-11*std::max(1.,scale));
    }
  }
  EXPECT_TRUE(exercised);
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
