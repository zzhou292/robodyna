// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Assertions.h"
#include "ContinuationCases.h"
namespace type25_selection_test {
inline void Same(const s::NativeContinuationResult& a,
    const s::NativeContinuationResult& b,bool exact=false) {
  SameClassified(a,b,exact);
  EXPECT_EQ(a.row_replaced,b.row_replaced);
  EXPECT_EQ(a.constrained_axis_mask,b.constrained_axis_mask);
  for(unsigned i=0;i<4;++i) {
    EXPECT_EQ(a.sliding_match[i],b.sliding_match[i]);
    EXPECT_EQ(a.cylindrical_gap[i],b.cylindrical_gap[i]);
  }
}
inline s::NativeContinuationResult ContinuationSentinel() {
  s::NativeContinuationResult result;
  result.history=BasicContinuation().prior;
  result.classification_product=31;result.distance_squared=32;
  result.cache.occurrence=33;result.constrained_axis_mask=5;
  result.sliding_match[1]=1;result.cylindrical_gap[2]=1;
  for(auto& sector:result.sector){sector.penetration=34;sector.distance_squared=35;}
  return result;
}
} // namespace type25_selection_test
