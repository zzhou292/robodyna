// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "PrescribedPath.h"
#include "../solid24_reference/SourceFixture.h"
#include <gtest/gtest.h>

namespace heph_test {
TEST(HephForceSource, All1309OriginalBricksRetainMappedSlotsAndNativeForceRecurrence) {
  unsigned count=0;
  for(unsigned index=0;index<solid24_test::SourceCount;++index) {
    const auto input=solid24_test::Source(index);
    if(!solid24_test::IsBrick(input))continue;
    SCOPED_TRACE(input.source_element_id);
    const auto reference=Reference(solid24_test::TotalReference(input,s::WorkingLengthUnit::Millimetre));
    const auto material=Material(input.density_kg_m3);
    s::History history;ASSERT_EQ(s::InitializeHistory(reference,material,history),s::ForceStatus::Success);
    auto native=InitializeNative(input);
    for(unsigned n=0;n<8;++n)ASSERT_EQ(reference.source_slot(n),unsigned(native.reference.permutation[n]));
    for(unsigned step=1;step<=3;++step) {
      SCOPED_TRACE(step);
      auto interval=Path(reference,step,1e-8);interval.base_time_s=history.stamp().time_s;
      s::ForceTrial result;
      ASSERT_EQ(s::EvaluateForce(reference,history,interval,material,result),s::ForceStatus::Success);
      const auto expected=NativeStep(native,interval,material);
      Compare(result,expected);
      history=result.proposed_history;AcceptNative(expected,native);
    }
    ++count;
  }
  EXPECT_EQ(count,1309u);RecordProperty("mapped_original_heph_force_parents",count);
}
}
