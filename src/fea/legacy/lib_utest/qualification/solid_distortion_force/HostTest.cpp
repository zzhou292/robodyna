// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
namespace distortion_force_test {
TEST(DistortionForce, NativeSiAndMillimetrePacketsMatchEveryForceEnergyAndStiffnessField) {
  int center=0,corner=0;
  for(const auto& c:Cases()){const auto r=Check(c);center+=r.center_contacts;corner+=r.corner_contacts;}
  EXPECT_GT(center,0);EXPECT_GT(corner,0);
}
TEST(DistortionForce, NativeWorkingUnitsPreserveNoncovariantDampingFormula) {
  auto si=Base();for(unsigned n=0;n<8;++n)si.input.velocity_m_s[n]={0,0,(n%2?1.:-1.)+.01};
  const auto a=Check(si);auto mm=si;mm.units={.001,1000,1};const auto b=Check(mm);
  ASSERT_EQ(a.damping_applied,1);ASSERT_EQ(b.damping_applied,1);
  EXPECT_NEAR(b.distortion_work_increment_j,a.distortion_work_increment_j*.001,1e-14);
  for(unsigned n=0;n<8;++n)EXPECT_NEAR(b.force_n[n].z-mm.input.incoming_force_n[n].z,
    .001*(a.force_n[n].z-si.input.incoming_force_n[n].z),1e-12);
  EXPECT_NEAR(a.raw_stiffness_n_m,b.raw_stiffness_n_m,1e-7);
}
TEST(DistortionForce, NativeBatchOrChangesZeroMeanBuckledRow) {
  auto zero=Base(true),trigger=zero;
  for(unsigned n=0;n<8;++n){zero.input.velocity_m_s[n]={0,0,n%2?1.:-1.};trigger.input.velocity_m_s[n]={0,0,(n%2?1.:-1.)+.01};}
  for(auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}}) {
    zero.units=units;trigger.units=units;const auto z=Prepare(zero),t=Prepare(trigger);
    EXPECT_FALSE(Batch(z));EXPECT_TRUE(Batch(t));const auto alone=Check(zero);EXPECT_EQ(alone.damping_applied,0);
    const auto expected=Native({zero,trigger});d::ForceResult out;
    ASSERT_EQ(d::EvaluateForce(z,true,out),d::Status::Success);Compare(out,expected[0]);
    EXPECT_EQ(out.damping_applied,1);EXPECT_GT(out.distortion_work_increment_j,0);
    ASSERT_EQ(d::EvaluateForce(t,true,out),d::Status::Success);Compare(out,expected[1]);
  }
}
TEST(DistortionForce, LoadingUnloadingAndLargeCarryKeepSeparateDirectWork) {
  auto c=Folded();c.units={.001,1000,1};
  for(unsigned n=0;n<8;++n)c.input.velocity_m_s[n]={0,0,n%2?.02:-.02}; // Exact zero mean, no single-row damping.
  const auto first=Check(c);EXPECT_FALSE(Batch(Prepare(c)));EXPECT_NE(first.distortion_work_increment_j,0);
  c.input.distortion_energy_j=first.distortion_energy_j;
  for(auto& v:c.input.velocity_m_s)v.z=-v.z;
  const auto reverse=Check(c);EXPECT_NEAR(reverse.distortion_work_increment_j,-first.distortion_work_increment_j,1e-8);
  c.input.distortion_energy_j=1e30;const auto huge=Check(c);
  EXPECT_EQ(huge.distortion_energy_j,1e30);EXPECT_NE(huge.distortion_work_increment_j,0);
}
TEST(DistortionForce, NativeCenterScatterPreservesForceBalance) {
  for(auto c:Cases()) {
    for(auto& force:c.input.incoming_force_n)force={};
    const auto result=Check(c);b::Vec3 sum{};double scale=1;
    for(const auto& force:result.force_n){sum=b::Add(sum,force);scale+=std::abs(force.x)+std::abs(force.y)+std::abs(force.z);}
    EXPECT_LE(std::abs(sum.x)+std::abs(sum.y)+std::abs(sum.z),1e-12*scale);
  }
}
TEST(DistortionForce, DegenerateNativeFaceAndSecondaryNodeBranches) {
  unsigned exercised=0;
  for(unsigned edge=0;edge<4;++edge)for(auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}}) {
    auto c=Folded();c.units=units;
    c.input.position_m[4+edge]=c.input.position_m[4+(edge+1)%4];
    ASSERT_GT(b::SignedCenterVolume(c.input.position_m),0);Check(c);++exercised;
  }
  EXPECT_EQ(exercised,8u);
}
TEST(DistortionForce, DegenerateLeafFollowsNativeWithoutInventingFamilyJacobianAdmission) {
  // The real family rejects an invalid material/Jacobian before reaching here.
  // This raw-leaf test proves S8FOR_DISTOR has no added center-volume screen.
  for(auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}}) {
    auto c=Folded();c.units=units;auto values=Prepare(c);
    for(auto& point:values.input.position)point={};
    d::ForceResult result;
    ASSERT_EQ(d::EvaluateForce(values,Batch(values),result),d::Status::Success);
    Compare(result,NativePrepared(values));
    EXPECT_EQ(result.center_contacts,0);EXPECT_EQ(result.corner_contacts,0);
    EXPECT_EQ(result.damping_applied,1);
  }
}
TEST(DistortionForce, InvalidGeometryInputsBatchDecisionAndOverflowPublishNothing) {
  const auto good=Prepare(Folded());d::ForceResult sentinel;
  ASSERT_EQ(d::EvaluateForce(good,Batch(good),sentinel),d::Status::Success);
  auto reject=[&](d::PreparedForceValues value,bool batch){auto out=sentinel;
    EXPECT_NE(d::EvaluateForce(value,batch,out),d::Status::Success);Same(out,sentinel);};
  auto bad=good;bad.input.position[0].x=std::numeric_limits<double>::quiet_NaN();reject(bad,true);
  bad=good;bad.parameters.length=0;reject(bad,true);
  bad=good;bad.input.dt=-1;reject(bad,true);
  bad=good;bad.parameters.damping=std::numeric_limits<double>::max();bad.input.velocity[0].x=4;reject(bad,true);
  auto trigger=Base();for(unsigned n=0;n<8;++n)trigger.input.velocity_m_s[n]={0,0,(n%2?1.:-1.)+.01};
  reject(Prepare(trigger),false);
  auto output=sentinel;ASSERT_EQ(d::EvaluateForce(good,Batch(good),output),d::Status::Success);Same(output,sentinel);
}
} // namespace distortion_force_test
