// SPDX-License-Identifier: AGPL-3.0-or-later
#include "JacobianNative.h"
#include "SourceFixture.h"
#include <gtest/gtest.h>
namespace solid24_test {
TEST(Solid24JacobianNative, CompleteWorldToLocalCallerAndUnitFloorBoundaries) {
  auto reversed=Distorted();
  for (unsigned n=0;n<4;++n) std::swap(reversed.position_m[n],reversed.position_m[n+4]);
  for (const auto& source:{Brick(),Distorted(),reversed}) {
    const auto input=TotalReference(source); s::Reference result;
    ASSERT_EQ(s::InitializeReference(input,result),s::Status::Success);
    const auto native=GlobalNativeRaw(input);
    const auto legacy=Native(input);
    ASSERT_EQ(native.status,0); ASSERT_EQ(legacy.status,0);
    EXPECT_TRUE(SameValueBits(native.legacy(),legacy.values));
    EXPECT_EQ(native.permutation,legacy.permutation);
    EXPECT_TRUE(Agree(Values(result),native.legacy()));
    EXPECT_TRUE(JacobianAgree(JacobianValues(result),native.jacobian()));
  }
  for (const auto units:{s::WorkingLengthUnit::Metre,s::WorkingLengthUnit::Millimetre}) {
    for (double factor:{.5,1.,2.}) {
      auto input=FloorControl(units,factor); s::Reference result;
      ASSERT_EQ(s::InitializeReference(input,result),s::Status::Success);
      const double scale=units==s::WorkingLengthUnit::Metre?1.0:.001;
      for (auto& x:input.position_m) {x.x/=scale;x.y/=scale;x.z/=scale;}
      const auto native=GlobalNativeRaw(input); ASSERT_EQ(native.status,0);
      EXPECT_TRUE(JacobianAgree(JacobianValues(result),WorkingJacobianToSI(native.jacobian(),units)));
    }
  }
}
TEST(Solid24JacobianNative, All1309OriginalGlobalReferencesAndLegacyFieldsRemainQualified) {
  unsigned count=0;
  for (unsigned i=0;i<SourceCount;++i) {
    const auto source=Source(i); if (!IsBrick(source)) continue;
    SCOPED_TRACE(source.source_element_id);
    const auto input=TotalReference(source,s::WorkingLengthUnit::Millimetre);
    s::Reference result,legacy;
    ASSERT_EQ(s::InitializeReference(input,result),s::Status::Success);
    ASSERT_EQ(s::InitializeReference(source,legacy),s::Status::Success);
    EXPECT_TRUE(SameValueBits(Values(result),Values(legacy)));
    const auto si=GlobalNativeRaw(source),working=GlobalNativeRaw(Source(i,true));
    ASSERT_EQ(si.status,0); ASSERT_EQ(working.status,0);
    EXPECT_TRUE(SameValueBits(si.legacy(),Native(source).values));
    EXPECT_TRUE(JacobianAgree(JacobianValues(result),si.jacobian()));
    double world=0;
    for (const auto& x:input.position_m) world=std::max({world,std::abs(x.x),std::abs(x.y),std::abs(x.z)});
    const double bound=64+256*std::max(1.,world/result.geometry().characteristic_length_m);
    const auto native_mm=WorkingJacobianToSI(working.jacobian(),s::WorkingLengthUnit::Millimetre);
    EXPECT_TRUE(JacobianAgree(JacobianValues(result),native_mm,bound));
    double scale=0;
    for (unsigned n=0;n<9;++n) scale=std::max(scale,std::abs(native_mm[n]));
    auto altered=native_mm; altered[0]+=1e-5*scale;
    EXPECT_FALSE(JacobianAgree(JacobianValues(result),altered,bound));
    for (unsigned n=0;n<8;++n) {
      EXPECT_EQ(result.source_slot(n),unsigned(working.permutation[n]));
      EXPECT_EQ(result.input().source_node_id[n],source.source_node_id[n]);
    }
    ++count;
  }
  EXPECT_EQ(count,1309u); RecordProperty("original_heph_global_references",count);
}
}  // namespace solid24_test
