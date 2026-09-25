// SPDX-License-Identifier: AGPL-3.0-or-later
// Complete selected ISHARP1 rounded/straight decision, I25DST3_3 a62b27e6.
#pragma once
#include "Boundary.h"
namespace tlfea::contact::radioss_type25::geometry_detail {
TL_MATH_HOST_DEVICE inline void Vertical(Work& w, NativeRawGeometryResult& out) {
  out.normal = w.plane;
  out.geometric_penetration = Max(0., w.gap + w.bb);
}
TL_MATH_HOST_DEVICE inline bool Sharp(const NativeGeometryInput& in, Work& w,
                                     NativeRawGeometryResult& out) {
  if (!w.boundary) return true;
  const double main_gap = w.gap - in.secondary_gap;
  if (w.edge_distance > 0 && w.bb + main_gap < 0) {
    // A valid native candidate defines XP on this branch. Reject malformed
    // prepared data rather than fabricate the donor's unassigned scratch.
    if (!w.closest_defined) return false;
    const auto center = v::Add(w.closest, v::Scale(w.plane, main_gap));
    const auto direction = v::Subtract(in.secondary, center);
    const double distance = ::sqrt(v::Dot(direction, direction));
    if (distance > em04) {
      out.normal = v::Scale(direction, 1. / distance);
      out.geometric_penetration = Max(0., in.secondary_gap - distance);
    } else {
      w.edge_distance -= in.secondary_gap;
      if (-w.bb < w.gap + w.edge_distance) {
        out.normal = w.boundary_normal;
        out.geometric_penetration = Max(0., -w.edge_distance);
      } else Vertical(w, out);
    }
  } else {
    w.edge_distance -= in.secondary_gap;
    if (w.edge_distance >= 0) {
      out.geometric_penetration = 0;
      return true;
    }
    if (w.gap + w.edge_distance > 0) {
      if (-w.bb < w.gap + w.edge_distance) {
        out.normal = w.boundary_normal;
        out.geometric_penetration = -w.edge_distance;
      } else Vertical(w, out);
    }
  }
  // Native DIST is not recomputed when a boundary changes N/PENE.
  return true;
}
} // namespace tlfea::contact::radioss_type25::geometry_detail
