// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected local I25COR3_3 geometry preparation, pinned OpenRadioss a62b27e6.
#pragma once
#include "../GeometryTypes.h"
#include "../NormalResponse.h"
#include "lib_src/math/Fixed3Operations.h"
namespace tlfea::contact::radioss_type25::geometry_detail {
namespace v = tl::math::fixed3;
inline constexpr double em20 = 1. / native_constant::ep20;
inline constexpr double em03 = 1. / 1000.;
inline constexpr double em04 = 1. / 10000.;
inline constexpr double epseg = (2. + 0.5) / 100.;
TL_MATH_HOST_DEVICE inline double Max(double a, double b) { return a < b ? b : a; }
TL_MATH_HOST_DEVICE inline double Min(double a, double b) { return a < b ? a : b; }
TL_MATH_HOST_DEVICE inline bool SameStored(float a, float b) {
  static_assert(sizeof(float) == 4, "Native stored normals require binary32");
  const auto* first = reinterpret_cast<const unsigned char*>(&a);
  const auto* second = reinterpret_cast<const unsigned char*>(&b);
  for (unsigned i = 0; i < sizeof(float); ++i) if (first[i] != second[i]) return false;
  return true;
}
TL_MATH_HOST_DEVICE inline Vector Promote(StoredNormal a) { return {a.x, a.y, a.z}; }
TL_MATH_HOST_DEVICE inline Vector AddStored(StoredNormal a, StoredNormal b) {
  // Fortran REAL*4 + REAL*4 rounds before assignment to my_real.
  const float x = a.x + b.x, y = a.y + b.y, z = a.z + b.z;
  return {x, y, z};
}
TL_MATH_HOST_DEVICE inline Vector Normalize(Vector a, double floor) {
  return v::Scale(a, 1. / Max(floor, ::sqrt(v::Dot(a, a))));
}
TL_MATH_HOST_DEVICE inline bool Supported(const GeometryProfile& p) {
  return p.gap_mode == 1 && p.sharp == 1 && p.initial_penetration == 5 &&
      p.damping_flag == 1 && !p.adhesion && !p.thermal && !p.foreign_row;
}
template<class U> TL_MATH_HOST_DEVICE inline bool Valid(const GeometryInput<U>& in) {
  if (!in.key.secondary_source_id || in.key.main_segment <= 0 ||
      in.key.history_index == SIZE_MAX || !v::Finite(in.secondary) ||
      !tl::math::Finite(in.lb) || !tl::math::Finite(in.lc) ||
      !normal_detail::Nonnegative(in.secondary_gap) ||
      !normal_detail::Nonnegative(in.incoming_stiffness)) return false;
  int sector = in.selection_code % 5;
  if (sector < 0) sector = -sector;
  if (sector < 1 || sector > 4) return false;
  const bool triangle = in.main_node_ids[2] == in.main_node_ids[3];
  if (triangle && sector != 1) return false; // Native current T3 selection.
  for (unsigned i = 0; i < 4; ++i) {
    if (!in.main_node_ids[i] || !v::Finite(in.main_vertices[i]) ||
        !v::Finite(Promote(in.corner_normal[i])) ||
        !normal_detail::Nonnegative(in.main_gap[i])) return false;
    for (unsigned j = 0; j < i; ++j)
      if (in.main_node_ids[i] == in.main_node_ids[j] &&
          (in.main_vertices[i].x != in.main_vertices[j].x ||
           in.main_vertices[i].y != in.main_vertices[j].y ||
           in.main_vertices[i].z != in.main_vertices[j].z)) return false;
    for (unsigned j = 0; j < 2; ++j)
      if (!v::Finite(Promote(in.vertex_bisector[i][j]))) return false;
    // Equal native boundary references must describe the same stored values.
    for (unsigned j = 0; j < i; ++j) if (in.boundary_ids[i] && in.boundary_ids[i] == in.boundary_ids[j])
      for (unsigned k = 0; k < 2; ++k) {
        const auto a = in.vertex_bisector[i][k], b = in.vertex_bisector[j][k];
        if (!SameStored(a.x, b.x) || !SameStored(a.y, b.y) || !SameStored(a.z, b.z)) return false;
      }
  }
  return true;
}
struct Work {
  Vector point[5]{}, normal[5]{}, arm[4]{}, plane{}, projected{}, closest{};
  Vector boundary_normal{};
  double la = 0, gap = 0, bb = 0, edge_distance = 0;
  unsigned sector = 0, a = 0, b = 0;
  bool triangle = false, shell_contact = false, boundary = false, closest_defined = false;
};
TL_MATH_HOST_DEVICE inline void Prepare(const NativeGeometryInput& in, Work& w) {
  int sector = in.selection_code % 5;
  if (sector < 0) sector = -sector;
  w.sector = unsigned(sector - 1); w.a = w.sector; w.b = (w.sector + 1) % 4;
  w.triangle = in.main_node_ids[2] == in.main_node_ids[3];
  for (unsigned i = 0; i < 4; ++i) {
    w.point[i] = in.main_vertices[i]; w.normal[i] = Promote(in.corner_normal[i]);
  }
  w.point[4] = w.triangle ? w.point[2] :
      v::Scale(v::Add(v::Add(v::Add(w.point[0], w.point[1]), w.point[2]), w.point[3]), .25);
  w.normal[4] = w.triangle ? w.normal[3] :
      v::Scale(v::Add(v::Add(v::Add(w.normal[0], w.normal[1]), w.normal[2]), w.normal[3]), .25);
  w.normal[4] = Normalize(w.normal[4], em20);
  for (unsigned i = 0; i < 4; ++i) w.arm[i] = v::Subtract(w.point[i], w.point[4]);
  w.plane = Normalize(v::Cross(w.arm[w.a], w.arm[w.b]), native_constant::em30);
  const double center_gap = w.triangle ? in.main_gap[2] :
      .25 * (in.main_gap[0] + in.main_gap[1] + in.main_gap[2] + in.main_gap[3]);
  w.la = 1. - in.lb - in.lc;
  // IGAP1's COR3 GAPMXL is native EP30, not a caller-created clearance.
  w.gap = Min(in.secondary_gap + w.la * center_gap +
      in.lb * in.main_gap[w.a] + in.lc * in.main_gap[w.b], native_constant::ep20 * native_constant::ep10);
  w.bb = v::Dot(v::Subtract(w.point[4], in.secondary), w.plane);
}
TL_MATH_HOST_DEVICE inline Vector Closest(const NativeGeometryInput& in, const Work& w) {
  return v::Add(v::Add(v::Scale(w.point[4], w.la), v::Scale(w.point[w.a], in.lb)),
                v::Scale(w.point[w.b], in.lc));
}
} // namespace tlfea::contact::radioss_type25::geometry_detail
