// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "PrescribedPath.h"
#include <gtest/gtest.h>

namespace heph_test {
TEST(HephForceNative, CompleteMovingHistoryAndHourglassWorkAcrossReversal) {
  auto reversed=solid24_test::Distorted();
  for(unsigned n=0;n<4;++n)std::swap(reversed.position_m[n],reversed.position_m[n+4]);
  for(const auto& input:{solid24_test::Brick(),solid24_test::Distorted(),reversed}) {
    const auto reference=Reference(input);const auto material=Material(input.density_kg_m3);
    s::History history;ASSERT_EQ(s::InitializeHistory(reference,material,history),s::ForceStatus::Success);
    auto native=InitializeNative(input);
    double carried=0;
    for(unsigned step=1;step<=32;++step) {
      SCOPED_TRACE(step);
      auto interval=Path(reference,step);
      // Use the actual accumulated stamp; this test introduces no other clock.
      interval.base_time_s=history.stamp().time_s;
      s::ForceTrial result;
      ASSERT_EQ(s::EvaluateForce(reference,history,interval,material,result),s::ForceStatus::Success);
      const auto expected=NativeStep(native,interval,material);
      Compare(result,expected);
      EXPECT_GT(result.diagnostics.stabilization_modulus_pa,0);
      EXPECT_GT(result.diagnostics.material.unscaled_element_dt_s,0);
      s::Vec3 force{};
      for(const auto& f:result.rhs_force_n)force=b::Add(force,f);
      EXPECT_LT(std::sqrt(b::Dot(force,force)),1e-11*std::max(ForceNorm(result),1.0));
      carried+=std::abs(result.diagnostics.stabilization_work_j);
      history=result.proposed_history;AcceptNative(expected,native);
    }
    EXPECT_GT(carried,0);
    double modes=0;
    for(const auto& row:history.values().physical_hourglass)for(double value:row)modes+=std::abs(value);
    EXPECT_GT(modes,0);
  }
}

TEST(HephForceNative, LateCutoffIdentityAndOverflowRetainEarlierTrialThenExactRetry) {
  const auto reference=Reference(solid24_test::Distorted());
  const auto material=Material(reference.input().density_kg_m3);
  s::History history;ASSERT_EQ(s::InitializeHistory(reference,material,history),s::ForceStatus::Success);
  auto native=InitializeNative(reference.input());
  const auto interval=Path(reference,1);
  s::ForceTrial result;
  ASSERT_EQ(s::EvaluateForce(reference,history,interval,material,result),s::ForceStatus::Success);
  const auto expected=NativeStep(native,interval,material);Compare(result,expected);
  const auto before=result;
  auto bad=interval;bad.velocity_m_s[7].z=std::numeric_limits<double>::max();
  EXPECT_NE(s::EvaluateForce(reference,history,bad,material,result),s::ForceStatus::Success);
  EXPECT_TRUE(Same(result,before));
  auto changed=reference.input();changed.source_material_id++;
  EXPECT_EQ(s::EvaluateForce(Reference(changed),history,interval,material,result),s::ForceStatus::ReferenceMismatch);
  EXPECT_TRUE(Same(result,before));
  auto cutoff=material;cutoff.tension_cutoff_pa=1;
  s::History cutoff_history;
  ASSERT_EQ(s::InitializeHistory(reference,cutoff,cutoff_history),s::ForceStatus::Success);
  const auto native_cutoff=NativeStep(native,interval,cutoff);
  ASSERT_EQ(native_cutoff.status,1);
  EXPECT_EQ(s::EvaluateForce(reference,cutoff_history,interval,cutoff,result),s::ForceStatus::MaterialFailure);
  EXPECT_TRUE(Same(result,before));
  ASSERT_EQ(s::EvaluateForce(reference,history,interval,material,result),s::ForceStatus::Success);
  EXPECT_TRUE(Same(result,before));
}
}
