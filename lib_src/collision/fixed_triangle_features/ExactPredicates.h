// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SurfaceContactTypes.h"

namespace tlfea::contact::fixed_triangle_features::exact {

struct Sign {
  int value = 0;
  bool valid = false;
};

// Exact signs for finite binary64 inputs.  The implementation converts every
// coordinate to a common dyadic integer scale and uses fixed-capacity integer
// arithmetic; it allocates no memory and never depends on long-double width.
Sign Orient2D(Vec3 a, Vec3 b, Vec3 c, int dropped_axis) noexcept;
Sign Orient3D(Vec3 a, Vec3 b, Vec3 c, Vec3 d) noexcept;
// Exact sign of ((b-a) x (c-a)) dot direction.  Unlike constructing
// a+direction and calling Orient3D, this does not round the chart direction
// through a translated point.
Sign DirectedTriangle(Vec3 a, Vec3 b, Vec3 c, Vec3 direction) noexcept;

enum class ClosestStratumKind {
  Vertex,
  Edge,
  Face,
};

struct ClosestTriangleStratum {
  ClosestStratumKind kind = ClosestStratumKind::Face;
  // Vertex ordinal for Vertex, local edge ordinal (i,i+1) for Edge.
  unsigned local = 0;
};

// Exact Ericson Voronoi-region partition.  All dot products, determinant-like
// products, and zero/tie decisions are evaluated as aligned dyadic integers.
// Returns false only if the fixed arithmetic bound is violated.
bool ClosestStratum(Vec3 point, const Vec3 (&triangle)[3],
                    ClosestTriangleStratum* output) noexcept;

}  // namespace tlfea::contact::fixed_triangle_features::exact
