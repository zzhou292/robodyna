// SPDX-License-Identifier: AGPL-3.0-or-later
#include "JacobianSupport.h"
#include <gtest/gtest.h>
namespace solid24_test {
TEST(Solid24JacobianHost, OptionalGlobalStageRetainsEveryLegacyValue) {
  for (const auto& input:{Brick(),Distorted()}) {
    s::Reference old,total;
    ASSERT_EQ(s::InitializeReference(input,old),s::Status::Success);
    ASSERT_EQ(old.reference_jacobian(),nullptr);
    ASSERT_EQ(s::InitializeReference(TotalReference(input),total),s::Status::Success);
    ASSERT_NE(total.reference_jacobian(),nullptr);
    EXPECT_TRUE(SameValueBits(Values(total),Values(old)));
    for (unsigned n=0;n<8;++n) EXPECT_EQ(total.source_slot(n),old.source_slot(n));
  }
  s::Reference r; ASSERT_EQ(s::InitializeReference(TotalReference(Brick()),r),s::Status::Success);
  const auto& j=*r.reference_jacobian();
  EXPECT_DOUBLE_EQ(j.inverse[2],1/(4*.02));
  EXPECT_DOUBLE_EQ(j.inverse[3],1/(4*.03));
  EXPECT_DOUBLE_EQ(j.inverse[7],1/(4*.04));
  for (unsigned n:{0,1,4,5,6,8}) EXPECT_EQ(j.inverse[n],0);
  EXPECT_NEAR(j.volume_m3,.02*.03*.04,1e-20);
}
TEST(Solid24JacobianHost, NamedUnitTransformsNativeVolumeFloor) {
  for (const auto unit:{s::WorkingLengthUnit::Metre,s::WorkingLengthUnit::Millimetre}) {
    for (double factor:{.5,1.,2.}) {
      const auto input=FloorControl(unit,factor); s::Reference r;
      ASSERT_EQ(s::InitializeReference(input,r),s::Status::Success);
      const double x=input.position_m[1].x;
      const double multiplier=std::min(1.,factor);
      EXPECT_NEAR(r.reference_jacobian()->inverse[2],multiplier/(4*x),1e-14/x);
    }
  }
  auto input=FloorControl(s::WorkingLengthUnit::Millimetre,.5); s::Reference mm,metre;
  ASSERT_EQ(s::InitializeReference(input,mm),s::Status::Success);
  input.profile.working_length=s::WorkingLengthUnit::Metre;
  ASSERT_EQ(s::InitializeReference(input,metre),s::Status::Success);
  EXPECT_NEAR(mm.reference_jacobian()->inverse[2]/metre.reference_jacobian()->inverse[2],1e9,1e-5);
}
TEST(Solid24JacobianHost, InvalidTagsLateFailureAliasAndRetryPreserveOptionalPayload) {
  const auto good=TotalReference(Distorted(),s::WorkingLengthUnit::Millimetre);
  s::Reference result;
  ASSERT_EQ(s::InitializeReference(good,result),s::Status::Success);
  const auto before=JacobianValues(result); const auto old=Values(result);
  auto bad=good; bad.profile.reference_strain=static_cast<s::ReferenceStrain>(255);
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::UnsupportedProfile);
  bad=good; bad.profile.working_length=static_cast<s::WorkingLengthUnit>(255);
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::UnsupportedProfile);
  bad=good;
  for (auto& x:bad.position_m) {x.x*=1e6;x.y*=1e6;x.z*=1e6;}
  bad.density_kg_m3=std::numeric_limits<double>::max();
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::NonfiniteResult);
  EXPECT_EQ(JacobianValues(result),before); EXPECT_EQ(Values(result),old);
  EXPECT_EQ(result.input().profile.working_length,s::WorkingLengthUnit::Millimetre);
  ASSERT_EQ(s::InitializeReference(result.input(),result),s::Status::Success);
  EXPECT_EQ(JacobianValues(result),before); EXPECT_EQ(Values(result),old);
}
}  // namespace solid24_test
