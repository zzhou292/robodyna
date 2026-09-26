// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <cstring>
#include <gtest/gtest.h>
namespace main_geometry_test {
inline std::uint64_t Bits(double x) {
  std::uint64_t bits;std::memcpy(&bits,&x,sizeof(bits));return bits;
}
inline void Exact(double a,double b) { EXPECT_EQ(Bits(a),Bits(b)); }
inline void Same(const n::NativeExteriorMainGeometryResult& a,const n::NativeExteriorMainGeometryResult& b) {
  Exact(a.normal_before_orientation.x,b.normal_before_orientation.x);
  Exact(a.normal_before_orientation.y,b.normal_before_orientation.y);
  Exact(a.normal_before_orientation.z,b.normal_before_orientation.z);
  Exact(a.area,b.area);Exact(a.signed_volume,b.signed_volume);Exact(a.center_projection,b.center_projection);
  for(unsigned i=0;i<4;++i)EXPECT_EQ(a.source_corner[i],b.source_corner[i]);
  EXPECT_EQ(a.reversed,b.reversed);
}
} // namespace main_geometry_test
