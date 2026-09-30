// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
namespace h24_test {
TEST(H24ControlledAdapter, EveryStageAndSourcePermutationMatchesCompleteNative) {
  for(const auto& x:Cases())Check(x);
}
TEST(H24ControlledAdapter, ConstructorAndRigidTranslationUseNativePrefix) {
  auto x=Base();x.initialization=true;x.interval.dt_s=0;Check(x);
  x.initialization=false;x.interval.dt_s=1e-6;
  for(unsigned n=0;n<8;++n){x.interval.velocity_m_s[n]={1,-2,.5};x.interval.position_m[n].x+=1e-6;x.interval.position_m[n].y-=2e-6;x.interval.position_m[n].z+=.5e-6;}
  Check(x);
}
TEST(H24ControlledAdapter, IndependentCarriedLoadingUnloadingMatches32NativeIntervals) {
  auto x=Base();x.accepted.material.internal_energy_density_j_m3=123.25;auto native_history=NativeHistory(x);
  bool changed_density=false,changed_sound=false,nonzero_qvis=false;
  double initial_sound=0;
  for(unsigned step=0;step<32;++step) {
    SCOPED_TRACE(step);Move(x,(step%8<4?1.:-1.)*(.1+.01*step));
    Trial r;ASSERT_EQ(Evaluate(x,r),s::ForceStatus::Success);const auto n=Native(native_history,x);Compare(x,r,n);
    ASSERT_FALSE(HasFailure());changed_density|=r.material.history.density_kg_m3!=x.material.density_kg_m3;
    if(step==0)initial_sound=r.material.point.sound_speed_m_s;else changed_sound|=r.material.point.sound_speed_m_s!=initial_sound;
    nonzero_qvis|=r.material.history.bulk_pressure_pa!=0;Accept(x,r);std::copy_n(n.full.values.begin(),22,native_history.values.begin());
  }
  EXPECT_TRUE(changed_density);EXPECT_TRUE(changed_sound);EXPECT_TRUE(nonzero_qvis);
}
TEST(H24ControlledAdapter, ActualHourglassWorkSurvivesEnergyCancellation) {
  auto x=Base();Move(x);x.accepted.material.internal_energy_density_j_m3=1e30;
  const auto r=Check(x);EXPECT_EQ(r.stage.proposed_values.material.internal_energy_density_j_m3,r.material.history.internal_energy_density_j_m3);
  EXPECT_NE(r.stage.hourglass.work_j,0);
}
TEST(H24ControlledAdapter, PrefixDoesNotClaimFinalDistortionForce) {
  auto x=Base();const unsigned a=x.reference.source_slot(0),o=x.reference.source_slot(6);
  const auto first=x.reference.input().position_m[a],opposite=x.reference.input().position_m[o];
  x.interval.position_m[a]={first.x+.5*(opposite.x-first.x),first.y+.5*(opposite.y-first.y),first.z+1.005*(opposite.z-first.z)};
  x.interval.velocity_m_s[a]={0,0,.1*(opposite.z-first.z)};
  x.interval.velocity_m_s[x.reference.source_slot(1)]={0,0,-.05*(opposite.z-first.z)};
  Trial r;ASSERT_EQ(Evaluate(x,r),s::ForceStatus::Success);const auto n=Native(NativeHistory(x),x);Compare(x,r,n);
  EXPECT_GT(n.full.center_contacts+n.full.corner_contacts,0);bool distinct=false;
  for(unsigned slot=0;slot<8;++slot)for(unsigned k=0;k<3;++k)
    distinct|=b::Component(r.stage.world_force_before_distortion_n[slot],k)!=n.full.values[22+3*slot+k];
  EXPECT_TRUE(distinct);
}
TEST(H24ControlledAdapter, InvalidAndLateOverflowKeepCompleteOutputThenRepair) {
  auto valid=Base();Move(valid);Trial output=Check(valid);const auto before=Values(output);
  std::vector<Case> bad;
  auto x=valid;x.accepted.controlled_hourglass.force_n[2][3]=std::numeric_limits<double>::max();bad.push_back(x);
  x=valid;x.interval.velocity_m_s[0].x=std::numeric_limits<double>::quiet_NaN();bad.push_back(x);
  x=valid;x.interval.dt_s=-1;bad.push_back(x);
  for(const auto& item:bad){EXPECT_NE(Evaluate(item,output),s::ForceStatus::Success);EXPECT_EQ(Values(output),before);}
  auto geometry=output.geometry;geometry.current.frame.v[0]=std::numeric_limits<double>::infinity();
  auto stage=output.stage;EXPECT_EQ(c::EvaluateBeforeDistortion(valid.reference,valid.material,geometry,output.material,valid.interval.dt_s,
      valid.accepted.controlled_hourglass,stage),s::ForceStatus::InvalidInput);
  EXPECT_EQ(Evaluate(valid,output),s::ForceStatus::Success);EXPECT_EQ(Values(output),before);
}
TEST(H24ControlledAdapter, NativePoissonAndWorkingLengthBoundaryRemainExplicit) {
  auto x=Base();ASSERT_EQ(tl::material::law42::Prepare(24e6,c::hg::MaximumPoissonRatio,1980,1e26,x.material),tl::material::law42::Status::Ok);Check(x);
  ASSERT_EQ(tl::material::law42::Prepare(24e6,std::nextafter(c::hg::MaximumPoissonRatio,1.),1980,1e26,x.material),tl::material::law42::Status::Ok);
  Trial r;EXPECT_EQ(Evaluate(x,r),s::ForceStatus::UnsupportedProfile);
  auto source=solid24_test::Brick();source.profile.reference_strain=s::ReferenceStrain::TotalLagrangian10;source.profile.working_length=s::WorkingLengthUnit::Millimetre;
  x=Base(source);EXPECT_EQ(Evaluate(x,r),s::ForceStatus::UnsupportedProfile);
}
} // namespace h24_test
