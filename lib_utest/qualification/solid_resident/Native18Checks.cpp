// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeChecks.h"
#include "../solid18_force/PacketValues.h"

namespace solid_resident_test {
void CheckNative(const s::Result18& result,const solid18_force_test::NativeResult& expected) {
  ASSERT_EQ(expected.status,0);
  const auto actual=Values(result);
  ASSERT_EQ(actual.size(),225u);
  using solid18_force_test::Range;
  for (unsigned ip=0;ip<8;++ip) {
    SCOPED_TRACE(ip);
    const auto* a=actual.data()+20*ip;
    const auto* b=expected.next.point.data()+20*ip;
    ASSERT_TRUE(Range(a,b,6,"resident point stress"));
    ASSERT_TRUE(Range(a+6,b+6,6,"resident point strain"));
    for (unsigned k=12;k<20;++k) ASSERT_TRUE(Range(a+k,b+k,1,"resident point history"));
  }
  ASSERT_TRUE(Range(actual.data()+160,expected.next.global.data(),6,"resident global stress"));
  for (unsigned k=6;k<11;++k)
    ASSERT_TRUE(Range(actual.data()+160+k,expected.next.global.data()+k,1,"resident global history"));
  ASSERT_TRUE(Range(actual.data()+171,expected.next.saved.data(),21,"resident saved positions"));
  ASSERT_TRUE(Range(actual.data()+192,expected.force.data(),24,"resident original-slot RHS"));
  for (unsigned k=0;k<7;++k)
    ASSERT_TRUE(Range(actual.data()+216+k,expected.diagnostics.data()+k,1,"resident diagnostic"));
  EXPECT_EQ(result.cache.diagnostics.selected_point,unsigned(expected.diagnostics[2]));
  EXPECT_NEAR(result.cache.stiffness.translation_n_m,.25*expected.diagnostics[4],
      3e-11*std::abs(.25*expected.diagnostics[4]));
  EXPECT_EQ(result.cache.stiffness.rotation_nm,0);
}
} // namespace solid_resident_test
