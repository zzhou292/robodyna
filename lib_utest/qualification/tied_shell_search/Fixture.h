// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/constraints/tied_shell/TiedSearch.h"
#include <gtest/gtest.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
namespace tied_search_test {
namespace ts=tl::constraints::tied_shell;
inline ts::SearchInput Shape(unsigned shape=0,double scale=.01) {
  ts::SearchInput in;
  in.geometry_m.master_position[0]={-scale,-scale,0};
  in.geometry_m.master_position[1]={scale,-scale,0};
  in.geometry_m.master_position[2]={scale,scale,0};
  in.geometry_m.master_position[3]={-scale,scale,0};
  if (shape==1) {
    in.geometry_m.master_position[0].z=scale*.08;
    in.geometry_m.master_position[2].z=scale*.03;
  }
  if (shape==2) {
    in.topology=ts::MasterTopology::TriangleRepeatedThird;
    in.geometry_m.master_position[2]={0,scale,0};
    in.geometry_m.master_position[3]=in.geometry_m.master_position[2];
  }
  in.master_thickness_m=scale*.05;
  in.working_length_to_m=.001;
  in.geometry_m.secondary_position={scale*.12,scale*.21,scale*.04};
  return in;
}
inline std::vector<ts::SearchInput> InvalidFiniteInputs() {
  auto normal_overflow=Shape(0,1e100);
  normal_overflow.working_length_to_m=1;
  normal_overflow.master_thickness_m=1;
  normal_overflow.geometry_m.secondary_position={0,0,1e102};
  auto diagonal_overflow=Shape();
  diagonal_overflow.working_length_to_m=1;
  diagonal_overflow.geometry_m.master_position[0].x=1e308;
  auto rounded_triangle=Shape(2);
  rounded_triangle.geometry_m.master_position[2].x=0x1.47ae147ae1490p-7;
  rounded_triangle.geometry_m.master_position[3].x=0x1.47ae147ae1491p-7;
  return {normal_overflow,diagonal_overflow,rounded_triangle};
}
template<class T> auto Bytes(const T& x) {
  std::array<unsigned char,sizeof(T)> out{}; std::memcpy(out.data(),&x,sizeof(T)); return out;
}
inline void Compare(const ts::CandidateProjection& a,const ts::CandidateProjection& b) {
  const double x[]{a.gap_m,a.penetration_m,a.distance_m,a.s,a.t,a.selection_distance};
  const double y[]{b.gap_m,b.penetration_m,b.distance_m,b.s,b.t,b.selection_distance};
  for(unsigned i=0;i<6;++i) {
    SCOPED_TRACE(i);
    EXPECT_NEAR(x[i],y[i],1e-14+2e-11*std::max(std::abs(x[i]),std::abs(y[i])));
  }
  EXPECT_EQ(a.fan,b.fan); EXPECT_EQ(a.admissible,b.admissible);
  EXPECT_EQ(a.outside_warning,b.outside_warning);
  EXPECT_EQ(a.working_length_to_m,b.working_length_to_m);
}
} // namespace tied_search_test
