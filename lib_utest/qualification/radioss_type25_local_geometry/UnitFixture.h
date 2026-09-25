// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
namespace type25_geometry_test {
inline n::SiGeometryInput Si(const n::NativeGeometryInput& in,n::UnitScale u) {
  n::SiGeometryInput out;
  out.key=in.key;out.segment_type=in.segment_type;out.selection_code=in.selection_code;
  out.lb=in.lb;out.lc=in.lc;out.incoming_stiffness=in.incoming_stiffness*u.mass_kg/(u.time_s*u.time_s);
  out.secondary=v::Scale(in.secondary,u.length_m);out.secondary_gap=in.secondary_gap*u.length_m;
  for(unsigned i=0;i<4;++i) {
    out.main_node_ids[i]=in.main_node_ids[i];out.main_vertices[i]=v::Scale(in.main_vertices[i],u.length_m);
    out.corner_normal[i]=in.corner_normal[i];out.neighbors[i]=in.neighbors[i];
    out.boundary_ids[i]=in.boundary_ids[i];out.main_gap[i]=in.main_gap[i]*u.length_m;
    for(unsigned j=0;j<2;++j)out.vertex_bisector[i][j]=in.vertex_bisector[i][j];
  }
  return out;
}
inline n::SiRawGeometryResult Si(const n::NativeRawGeometryResult& in,n::UnitScale u) {
  n::SiRawGeometryResult out;
  out.key=in.key;out.selection_code=in.selection_code;out.normal=in.normal;
  for(unsigned i=0;i<4;++i)out.weights[i]=in.weights[i];
  out.geometric_penetration=in.geometric_penetration*u.length_m;out.gap=in.gap*u.length_m;
  out.distance=in.distance*u.length_m;out.incoming_stiffness=in.incoming_stiffness*u.mass_kg/(u.time_s*u.time_s);
  return out;
}
} // namespace type25_geometry_test
