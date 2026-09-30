// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Selection.h"
#include "../radioss_type25_local_geometry/Cases.h"
#include <string>
#include <vector>
namespace type25_selection_test {
namespace n = tlfea::contact::radioss_type25;
namespace s = n::selection;
struct Case { std::string name; s::NativePairInput input; n::NativeGeometryHistory prior; };
inline s::Profile Profile() { return {1,5,1,false,false,false}; }
inline Case FromGeometry(const type25_geometry_test::Case& source) {
  Case out;out.name=source.name;const auto& in=source.input;auto& p=out.input;
  p.key=in.key;p.occurrence=0;p.local_main=9;p.secondary=in.secondary;
  p.segment_type=in.segment_type;p.secondary_gap=in.secondary_gap;
  p.main_coefficient=400;p.secondary_coefficient=2;
  for(unsigned i=0;i<4;++i) {
    p.main_node_ids[i]=in.main_node_ids[i];p.main_vertices[i]=in.main_vertices[i];
    p.normal_slot[i]=in.corner_normal[i];p.neighbors[i]=in.neighbors[i];
    p.boundary_ids[i]=in.boundary_ids[i];p.main_gap[i]=in.main_gap[i];
    p.main_gap_max=p.main_gap_max<p.main_gap[i]?p.main_gap[i]:p.main_gap_max;
    for(unsigned j=0;j<2;++j)p.vertex_bisector[i][j]=in.vertex_bisector[i][j];
  }
  out.prior=type25_geometry_test::History(in);
  return out;
}
inline Case Basic() { return FromGeometry({"basic",type25_geometry_test::Quad()}); }
inline std::vector<Case> Cases() {
  std::vector<Case> out;
  for(const auto& source:type25_geometry_test::Cases()) {
    auto c=FromGeometry(source);c.input.occurrence=out.size();out.push_back(c);
  }
  return out;
}
} // namespace type25_selection_test
