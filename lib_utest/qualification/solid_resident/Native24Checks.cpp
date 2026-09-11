// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeChecks.h"

namespace solid_resident_test {
void CheckNative(const s::Result24& result,const heph_test::NativeTrial& expected) {
  ASSERT_EQ(expected.status,0);
  const auto actual=Values(result);
  ASSERT_EQ(actual.size(),83u);
  const auto compare=[&](unsigned a,unsigned b) {
    EXPECT_TRUE(std::isfinite(actual[a]));
    EXPECT_NEAR(actual[a],expected.values[b],heph_test::ForceTolerance(b,expected.values))<<a<<'/'<<b;
  };
  for (unsigned k=0;k<21;++k) compare(k,k);
  for (unsigned k=0;k<24;++k) compare(21+k,22+k);
  for (unsigned k=0;k<36;++k) compare(45+k,151+k);
  EXPECT_NEAR(result.cache.stiffness.translation_n_m,.25*expected.values[183],
      3e-10*std::abs(.25*expected.values[183]));
  EXPECT_EQ(result.cache.stiffness.rotation_nm,0);
}
} // namespace solid_resident_test
