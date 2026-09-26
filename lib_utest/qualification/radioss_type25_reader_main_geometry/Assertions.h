// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "../radioss_type25_main_geometry/Assertions.h"
namespace reader_main_geometry_test {
using main_geometry_test::Basic;
using main_geometry_test::Cases;
using main_geometry_test::Exact;
inline void Same(const n::NativeInternalMainGeometryResult& a,const n::NativeInternalMainGeometryResult& b) {
  Exact(a.normal_before_orientation.x,b.normal_before_orientation.x);
  Exact(a.normal_before_orientation.y,b.normal_before_orientation.y);
  Exact(a.normal_before_orientation.z,b.normal_before_orientation.z);
  Exact(a.area,b.area);Exact(a.signed_volume,b.signed_volume);
}
inline n::NativeInternalMainGeometryResult Sentinel() {return {{71.,-0.,13.},91.,-81.};}
inline std::vector<n::NativeExteriorMainGeometryInput> Invalid() {
  auto out=main_geometry_test::Invalid();
  out.back()=Basic();out.back().solid_raw[0].x=-std::numeric_limits<double>::max();
  out.back().solid_raw[6].x=std::numeric_limits<double>::max();return out;
}
}
