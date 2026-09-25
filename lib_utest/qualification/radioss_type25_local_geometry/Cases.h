// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25LocalGeometry.h"
#include <string>
#include <vector>
namespace type25_geometry_test {
namespace n = tlfea::contact::radioss_type25;
namespace v = tl::math::fixed3;
struct Case { std::string name; n::NativeGeometryInput input; };
inline n::GeometryProfile Profile() { return {1, 1, 5, 1, false, false, false}; }
inline n::NativeGeometryInput Quad(unsigned sector = 0) {
  n::NativeGeometryInput in;
  in.key = {1101, 7, 0, 3};
  for (unsigned i = 0; i < 4; ++i) {
    in.main_node_ids[i] = 100 + i; in.neighbors[i] = 1; in.main_gap[i] = .4;
  }
  in.main_vertices[0] = {0, 0, 0}; in.main_vertices[1] = {4, 0, 0};
  in.main_vertices[2] = {4, 4, 0}; in.main_vertices[3] = {0, 4, 0};
  in.corner_normal[0] = {0, -1, 0}; in.corner_normal[1] = {1, 0, 0};
  in.corner_normal[2] = {0, 1, 0}; in.corner_normal[3] = {-1, 0, 0};
  in.segment_type = 4; in.secondary_gap = .2;
  in.selection_code = int(sector + 1); in.lb = in.lc = .25;
  in.incoming_stiffness = 400;
  in.secondary = {2, 1, .2};
  return in;
}
inline n::NativeGeometryInput Triangle() {
  auto in = Quad();
  in.main_node_ids[3] = in.main_node_ids[2];
  in.main_vertices[2] = in.main_vertices[3] = {0, 4, 0};
  in.corner_normal[1] = {0x1.6a09e6p-1f, 0x1.6a09e6p-1f, 0};
  in.corner_normal[3] = {-1, 0, 0};
  in.secondary = {1, 2, .2}; return in;
}
// Test data generator only: place a selected native center-triangle barycentric
// point and a plane-normal displacement, without calling production geometry.
inline void AtBarycentric(n::NativeGeometryInput& in, double z) {
  const bool triangle = in.main_node_ids[2] == in.main_node_ids[3];
  const auto center = triangle ? in.main_vertices[2] :
      v::Scale(v::Add(v::Add(v::Add(in.main_vertices[0], in.main_vertices[1]),
          in.main_vertices[2]), in.main_vertices[3]), .25);
  unsigned a = unsigned((in.selection_code < 0 ? -in.selection_code : in.selection_code) % 5 - 1);
  const unsigned b = (a + 1) % 4;
  in.secondary = v::Add(v::Add(v::Scale(center, 1 - in.lb - in.lc),
      v::Scale(in.main_vertices[a], in.lb)), v::Scale(in.main_vertices[b], in.lc));
  in.secondary.z += z;
}
inline n::NativeGeometryHistory History(const n::NativeGeometryInput& in) {
  n::NativeGeometryHistory row;
  row.secondary_source_id = in.key.secondary_source_id; row.generation = in.key.generation;
  row.row.irtlm[0] = in.key.main_segment; row.row.irtlm[1] = in.selection_code;
  row.row.irtlm[2] = 9; row.row.irtlm[3] = 1;
  row.row.history.normal = {.125, 300, .0625, 200, -.03125};
  row.row.history.previous_force = {.25, -.5, .125};
  row.row.history.staged_force = {-.125, .0625, -.25};
  row.row.penetration_auxiliary = .375; row.row.penetration_offset = .05;
  row.row.time_s[0] = -1e20; row.row.time_s[1] = 1e20;
  return row;
}
inline std::vector<Case> Cases() {
  std::vector<Case> rows;
  for (unsigned sector = 0; sector < 4; ++sector)
    for (double z : {-.2, 0., .0005, .001, .0010001, .2, .8}) {
      auto in = Quad(sector); AtBarycentric(in, z);
      rows.push_back({"q4-general-" + std::to_string(sector), in});
    }
  for (bool triangle : {false, true})
    for (unsigned edge = 0; edge < (triangle ? 3u : 1u); ++edge)
      for (double small : {0., .0125, .025, .0250001})
        for (double z : {-.2, .2}) {
          auto in = triangle ? Triangle() : Quad();
          if (edge == 0) { in.lb = .4; in.lc = 1 - in.lb - small; }
          if (edge == 1) { in.lb = small; in.lc = .4; }
          if (edge == 2) { in.lc = small; in.lb = .4; }
          AtBarycentric(in, z); rows.push_back({"interpolation-" + std::to_string(edge), in});
        }
  for (double outside : {-.1, 0., .00002, .1, .2, .3})
    for (double z : {.1, .40001, .40003, .45, .7}) {
      auto in = Quad(); in.lb = in.lc = .5; in.neighbors[0] = 0;
      in.secondary = {2, -outside, z};
      rows.push_back({"q4-free-edge-rounded-straight", in});
    }
  for (unsigned mask = 1; mask < 8; ++mask)
    for (auto position : {n::Vector{1, -.1, .1}, n::Vector{-.1, 1, .45},
                          n::Vector{2.1, 2.1, .1}}) {
      auto in = Triangle();
      in.neighbors[0] = !(mask & 1); in.neighbors[1] = !(mask & 2); in.neighbors[3] = !(mask & 4);
      in.secondary = position; rows.push_back({"t3-free-edge-" + std::to_string(mask), in});
    }
  for (bool triangle : {false, true})
    for (unsigned corner = 0; corner < (triangle ? 3u : 2u); ++corner)
      for (unsigned kind = 0; kind < 5; ++kind) {
        auto in = triangle ? Triangle() : Quad();
        in.boundary_ids[corner] = 900 + corner;
        if (corner == 0) { in.lb = 1; in.lc = 0; }
        if (corner == 1) { in.lb = 0; in.lc = 1; }
        if (corner == 2) in.lb = in.lc = 0;
        AtBarycentric(in, .1);
        in.secondary.x -= (kind == 2 || kind == 3) ? .3 : .1;
        in.secondary.y -= (kind == 1 || kind == 3) ? .3 : .1;
        if (kind != 4) {
          in.vertex_bisector[corner][0] = {-1, 0, 0};
          in.vertex_bisector[corner][1] = {0, -1, 0};
        }
        rows.push_back({"vertex-bisectors-" + std::to_string(kind), in});
      }
  auto sum = Quad();
  sum.lb = 1; sum.lc = 0; sum.boundary_ids[0] = 91;
  sum.vertex_bisector[0][0] = {1, 0, 0};
  sum.vertex_bisector[0][1] = {0x1p-25f, 1, 0};
  sum.secondary = {.1, .1, .1};
  rows.push_back({"float32-bisector-sum-rounding", sum});
  auto warped = Quad(2); warped.main_vertices[2].z = .25;
  warped.main_gap[0] = .3; warped.main_gap[1] = .45; warped.main_gap[2] = .6;
  AtBarycentric(warped, .1); rows.push_back({"warped-q4-unequal-gaps", warped});
  auto solid = Quad(); solid.segment_type = 0; solid.secondary_gap = 0;
  for (auto& gap : solid.main_gap) gap = 0;
  AtBarycentric(solid, -.2); rows.push_back({"solid-no-shell-offset", solid});
  return rows;
}
} // namespace type25_geometry_test
