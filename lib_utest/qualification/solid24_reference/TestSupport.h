// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid24/Solid24Reference.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

namespace solid24_test {
namespace s = tl::fea::solid24;
inline s::ReferenceInput Brick() {
  s::ReferenceInput input;
  input.source_element_id=1; input.source_part_id=2;
  input.source_section_id=3; input.source_material_id=4;
  input.density_kg_m3=1980;
  const s::Vec3 x[8]{{0,0,0},{.02,0,0},{.02,.03,0},{0,.03,0},
                     {0,0,.04},{.02,0,.04},{.02,.03,.04},{0,.03,.04}};
  for(unsigned n=0;n<8;++n) { input.source_node_id[n]=100+n; input.position_m[n]=x[n]; }
  return input;
}
inline s::ReferenceInput Distorted() {
  auto input=Brick();
  input.position_m[6].x+=.002; input.position_m[6].y-=.003;
  input.position_m[4].z+=.001;
  for(auto& p:input.position_m) {
    const double x=.8*p.x-.6*p.y, y=.6*p.x+.8*p.y;
    p={x+2,y-.4,p.z+.7};
  }
  return input;
}
inline std::array<double,44> Values(const s::Reference& r) {
  std::array<double,44> result{};
  unsigned i=0;
  for(double v:r.geometry().frame.v) result[i++]=v;
  for(const auto& x:r.geometry().local_position_m) {result[i++]=x.x;result[i++]=x.y;result[i++]=x.z;}
  result[i++]=r.geometry().volume_m3;
  result[i++]=r.geometry().characteristic_length_m;
  for(double m:r.mass().source_slot_mass_kg) result[i++]=m;
  result[i++]=r.mass().element_mass_kg;
  return result;
}
inline bool AgreeWithRoundoff(const std::array<double,44>& a,const std::array<double,44>& b,
                             double roundoff_units) {
  constexpr unsigned offsets[]{0,9,33,34,35,43,44};
  for(unsigned group=0;group<6;++group) {
    double scale=0;
    for(unsigned i=offsets[group];i<offsets[group+1];++i)
      scale=std::max({scale,std::abs(a[i]),std::abs(b[i])});
    for(unsigned i=offsets[group];i<offsets[group+1];++i) {
      if(!std::isfinite(a[i]) || !std::isfinite(b[i]) ||
         std::abs(a[i]-b[i])>2e-11*std::abs(b[i])+roundoff_units*std::numeric_limits<double>::epsilon()*scale)
        return false;
    }
  }
  return true;
}
inline bool Agree(const std::array<double,44>& a,const std::array<double,44>& b) {
  return AgreeWithRoundoff(a,b,64);
}
inline bool AgreeWorkingUnits(const std::array<double,44>& a,const std::array<double,44>& b,
                              double conditioning) {
  if(!std::isfinite(conditioning)||conditioning<0)return false;
  // Original world-coordinate conversion precedes differences and frame
  // normalization. Only this cross-unit comparison accounts for that scale.
  return AgreeWithRoundoff(a,b,64+256*std::max(1.,conditioning));
}
}  // namespace solid24_test
