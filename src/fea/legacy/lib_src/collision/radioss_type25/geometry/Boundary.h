// SPDX-License-Identifier: AGPL-3.0-or-later
// I25DST3_3 complete selected shell boundary census, source a62b27e6.
#pragma once
#include "General.h"
namespace tlfea::contact::radioss_type25::geometry_detail {
TL_MATH_HOST_DEVICE inline void SelectBoundary(Work& w, Vector normal, Vector origin) {
  w.boundary_normal = normal;
  w.edge_distance = v::Dot(v::Subtract(w.projected, origin), normal);
  w.boundary = true;
}
TL_MATH_HOST_DEVICE inline void VertexBoundary(const NativeGeometryInput& in,
    Work& w, unsigned corner) {
  const auto a = in.vertex_bisector[corner][0], b = in.vertex_bisector[corner][1];
  const bool present = a.x != 0 || a.y != 0 || a.z != 0 || b.x != 0 || b.y != 0 || b.z != 0;
  if (present) {
    const auto delta = v::Subtract(w.projected, w.point[corner]);
    const double p1 = v::Dot(delta, Promote(a)), p2 = v::Dot(delta, Promote(b));
    if (p1 < in.secondary_gap && p2 < in.secondary_gap)
      SelectBoundary(w, Normalize(AddStored(a, b), native_constant::em30), w.point[corner]);
    else if (p1 < in.secondary_gap) SelectBoundary(w, Promote(a), w.point[corner]);
    else if (p2 < in.secondary_gap) SelectBoundary(w, Promote(b), w.point[corner]);
  } else {
    Vector direction;
    if (!w.triangle) direction = w.arm[corner];
    else {
      const auto j = (corner + 1) % 3, k = (corner + 2) % 3;
      direction = v::Subtract(v::Scale(w.point[corner], 2.), v::Add(w.point[j], w.point[k]));
    }
    const double inverse = 1. / Max(em20, ::sqrt(v::Dot(direction, direction)));
    const double pn = v::Dot(v::Subtract(w.projected, w.point[corner]), direction) * inverse;
    if (pn < in.secondary_gap) SelectBoundary(w, v::Scale(direction, inverse), w.point[corner]);
  }
}
TL_MATH_HOST_DEVICE inline void Boundary(const NativeGeometryInput& in, Work& w) {
  if (!w.shell_contact) return;
  w.projected = v::Add(in.secondary, v::Scale(w.plane, w.bb));
  if (!w.triangle) {
    if (in.neighbors[w.sector] == 0) {
      SelectBoundary(w, w.normal[w.sector], w.point[w.a]);
    } else if ((in.boundary_ids[w.a] && !in.boundary_ids[w.b]) ||
               (in.boundary_ids[w.b] && !in.boundary_ids[w.a])) {
      VertexBoundary(in, w, in.boundary_ids[w.a] ? w.a : w.b);
    }
  } else if (!in.neighbors[0] || !in.neighbors[1] || !in.neighbors[3]) {
    w.edge_distance = in.secondary_gap;
    const unsigned normals[3]{w.a, w.b, 4}, neighbors[3]{0, 1, 3};
    const unsigned origins[3]{w.a, w.b, 4};
    // Native sequential strict-min updates retain first-owner ties.
    for (unsigned j = 0; j < 3; ++j) {
      const auto normal = w.normal[normals[j]];
      const double distance = v::Dot(v::Subtract(w.projected, w.point[origins[j]]), normal);
      if (!in.neighbors[neighbors[j]] && distance < w.edge_distance) {
        w.boundary = true; w.edge_distance = distance; w.boundary_normal = normal;
      }
    }
  } else {
    unsigned count = 0, corner = 0;
    for (unsigned i = 0; i < 3; ++i)
      if (in.boundary_ids[i]) { ++count; corner = i; }
    if (count == 1) VertexBoundary(in, w, corner);
  }
}
} // namespace tlfea::contact::radioss_type25::geometry_detail
