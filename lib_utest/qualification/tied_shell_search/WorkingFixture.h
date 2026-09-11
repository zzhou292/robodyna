// SPDX-License-Identifier: MIT
#pragma once
#include "NativeOracle.h"
#include "BoundsFixture.h"

namespace tied_search_test {
inline ts::WorkingSearchInput OriginalWorkingShape(unsigned shape=0) {
  ts::WorkingSearchInput out;
  const auto source=Shape(shape);
  out.topology=source.topology;
  out.working_length_to_m=.001;
  out.master_thickness=1.234567;
  for(unsigned i=0;i<4;++i) {
    const auto p=source.geometry_m.master_position[i];
    out.geometry.master_position[i]={1000.1234+p.x*1000,123.4567+p.y*1000,p.z*1000};
  }
  out.geometry.secondary_position={1000.1234,123.4567,.1};
  return out;
}
inline ts::WorkingSearchBoundsInput WorkingBounds(const ts::WorkingSearchInput& in) {
  ts::WorkingSearchBoundsInput out;
  std::copy(std::begin(in.geometry.master_position),std::end(in.geometry.master_position),out.master_position);
  out.topology=in.topology;
  out.working_length_to_m=in.working_length_to_m;
  out.master_thickness=in.master_thickness;
  return out;
}
inline void CompareWorking(const ts::WorkingSearchInput& input,
    const ts::CandidateProjection& projection,const ts::NativeSearchBounds& bounds,bool within) {
  NativeChoice choice;
  Compare(projection,Native(input,1,choice));
  const auto expected=NativeBounds(WorkingBounds(input),input.geometry.secondary_position);
  const auto values=BoundValues(bounds);
  for(unsigned i=0;i<values.size();++i) EXPECT_DOUBLE_EQ(values[i],expected[i]);
  EXPECT_EQ(within,expected[7]==1);
}
} // namespace tied_search_test
