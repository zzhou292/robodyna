// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "PrescribedPath.h"
#include <gtest/gtest.h>
namespace heph_test {
TEST(HephForce, RestTranslationAndPhysicalHourglassHistory) {
  auto r=Reference(); const auto p=Material(r.input().density_kg_m3); s::History h;
  ASSERT_EQ(s::InitializeHistory(r,p,h),s::ForceStatus::Success);
  auto i=Interval(r,h); s::ForceTrial t;
  for(auto& v:i.velocity_m_s)v={2,-3,1};
  ASSERT_EQ(s::EvaluateForce(r,h,i,p,t),s::ForceStatus::Success);
  EXPECT_EQ(Values(t).size(),187u);
  EXPECT_LT(ForceNorm(t),1e-8);
  EXPECT_GT(t.diagnostics.material.unscaled_element_dt_s,0);
  constexpr double mode[]{1,1,-1,-1,-1,-1,1,1};
  i=Interval(r,h);
  for(unsigned n=0;n<8;++n) {
    i.velocity_m_s[n].x=mode[n];
    i.position_m[n].x+=i.dt_s*mode[n];
  }
  ASSERT_EQ(s::EvaluateForce(r,h,i,p,t),s::ForceStatus::Success);
  EXPECT_GT(ForceNorm(t),0);
  EXPECT_GT(t.diagnostics.stabilization_work_j,0);
  EXPECT_GT(t.proposed_history.values().material.internal_energy_density_j_m3,0);
  double retained=0;
  for(const auto& row:t.proposed_history.values().physical_hourglass)for(double x:row)retained+=std::abs(x);
  EXPECT_GT(retained,0);
  const auto before=t;
  i.base_time_s=t.proposed_history.stamp().time_s;++i.sample_index;
  for(auto& v:i.velocity_m_s)v={};
  ASSERT_EQ(s::EvaluateForce(r,before.proposed_history,i,p,t),s::ForceStatus::Success);
  EXPECT_GT(ForceNorm(t),0); // Physical elastic modes persist with zero velocity.
  EXPECT_DOUBLE_EQ(t.diagnostics.stabilization_work_j,0);
}
TEST(HephForce, AffineStretchForceBalanceAndSourceIdentity) {
  const auto r=Reference(solid24_test::Distorted()); const auto p=Material(r.input().density_kg_m3);
  s::History h;ASSERT_EQ(s::InitializeHistory(r,p,h),s::ForceStatus::Success);
  auto i=Interval(r,h);s::ForceTrial t;
  for(unsigned n=0;n<8;++n) {
    i.velocity_m_s[n]={5*i.position_m[n].x,-2*i.position_m[n].y,0};
    i.position_m[n].x+=i.dt_s*i.velocity_m_s[n].x;
    i.position_m[n].y+=i.dt_s*i.velocity_m_s[n].y;
  }
  ASSERT_EQ(s::EvaluateForce(r,h,i,p,t),s::ForceStatus::Success);
  EXPECT_GT(ForceNorm(t),0);
  s::Vec3 sum{};for(const auto& f:t.rhs_force_n)sum=b::Add(sum,f);
  EXPECT_LT(std::sqrt(b::Dot(sum,sum)),1e-12*ForceNorm(t));
  auto changed=r.input();++changed.source_node_id[7];const auto other=Reference(changed);
  const auto before=t;
  EXPECT_EQ(s::EvaluateForce(other,h,i,p,t),s::ForceStatus::ReferenceMismatch);
  EXPECT_TRUE(Same(t,before));
  auto zero=r.input();zero.position_m[0].z=-0.0;
  const auto a=Reference(zero);zero.position_m[0].z=0.0;const auto c=Reference(zero);
  EXPECT_FALSE(s::force_detail::SameReference(a,c));
}
TEST(HephForce, LateInvalidMaterialOverflowAndExactRetryPreserveTrial) {
  const auto r=Reference();const auto p=Material(r.input().density_kg_m3);
  s::History h;ASSERT_EQ(s::InitializeHistory(r,p,h),s::ForceStatus::Success);
  auto good=Interval(r,h);s::ForceTrial t;
  good.velocity_m_s[7].x=1;
  ASSERT_EQ(s::EvaluateForce(r,h,good,p,t),s::ForceStatus::Success);
  const auto before=t;
  auto bad=good;bad.velocity_m_s[7].x=std::numeric_limits<double>::max();
  EXPECT_NE(s::EvaluateForce(r,h,bad,p,t),s::ForceStatus::Success);
  EXPECT_TRUE(Same(t,before));
  bad=good;bad.position_m[7].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(s::EvaluateForce(r,h,bad,p,t),s::ForceStatus::InvalidInput);
  EXPECT_TRUE(Same(t,before));
  bad=good;++bad.sample_index;
  EXPECT_EQ(s::EvaluateForce(r,h,bad,p,t),s::ForceStatus::InvalidInput);
  ASSERT_EQ(s::EvaluateForce(r,h,good,p,t),s::ForceStatus::Success);
  EXPECT_TRUE(Same(t,before));
}
TEST(HephForce, PrescribedMovingHistoryUsesExactAcceptedPhase) {
  for(const auto& input:{solid24_test::Brick(),solid24_test::Distorted()}) {
    const auto reference=Reference(input);
    const auto material=Material(input.density_kg_m3);
    s::History history;
    ASSERT_EQ(s::InitializeHistory(reference,material,history),s::ForceStatus::Success);
    for(unsigned step=1;step<=32;++step) {
      auto interval=Path(reference,step);
      interval.base_time_s=history.stamp().time_s;
      s::ForceTrial trial;
      ASSERT_EQ(s::EvaluateForce(reference,history,interval,material,trial),s::ForceStatus::Success);
      const auto before=trial;
      auto wrong=interval;
      wrong.base_time_s=std::nextafter(wrong.base_time_s,1.0);
      EXPECT_EQ(s::EvaluateForce(reference,history,wrong,material,trial),s::ForceStatus::InvalidInput);
      EXPECT_TRUE(Same(trial,before));
      EXPECT_EQ(trial.proposed_history.stamp().sample_index,step);
      EXPECT_EQ(trial.proposed_history.stamp().time_s,history.stamp().time_s+interval.dt_s);
      history=trial.proposed_history;
    }
  }
}
} // namespace heph_test
