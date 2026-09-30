// SPDX-License-Identifier: AGPL-3.0-or-later
// Native PCOORI / PEVECI node-defined orientation branch.
#pragma once
#include "Units.h"

namespace tl::fea::beam18::detail {
TL_BEAM18_HD inline Status PrepareGeometry(const Input& input, UnitFactors units,
                                           Geometry& output) noexcept {
  using namespace tl::math::fixed3;
  const Vec3 chord = Subtract(input.position[1],input.position[0]);
  const double xx = (input.position[0].x-input.position[1].x)*(input.position[0].x-input.position[1].x);
  const double yy = (input.position[0].y-input.position[1].y)*(input.position[0].y-input.position[1].y);
  const double zz = (input.position[0].z-input.position[1].z)*(input.position[0].z-input.position[1].z);
  Geometry next{};
  next.length = ::sqrt(xx+yy+zz);
  if (!tl::math::Finite(next.length)) return Status::NonfiniteResult;
  if (next.length <= (1.0/(1e10*1e10))) return Status::DegenerateGeometry;
  bool fallback = input.source_node_id[2] == 0 ||
      input.source_node_id[2] == input.source_node_id[0] ||
      input.source_node_id[2] == input.source_node_id[1];
  Vec3 seed = Subtract(input.position[2],input.position[0]);
  if (!fallback) {
    const double norm = ::sqrt(seed.x*seed.x+seed.y*seed.y+seed.z*seed.z);
    if (!tl::math::Finite(norm)) return Status::NonfiniteResult;
    if (norm < (1.0/(1e10*1e10))) return Status::DegenerateGeometry;
    const double d1 = chord.x*seed.y-chord.y*seed.x;
    const double d2 = chord.y*seed.z-chord.z*seed.y;
    const double d3 = chord.z*seed.x-chord.x*seed.z;
    const double determinant = ::sqrt(d1*d1+d2*d2+d3*d3);
    if (!tl::math::Finite(determinant)) return Status::NonfiniteResult;
    fallback = determinant < (2.0/1e6);
  }
  if (fallback) {
    const double sum = chord.x*chord.x+chord.z*chord.z;
    next.orientation_branch = sum < (1.0/(1e10*1e10)) ? OrientationBranch::GlobalZ : OrientationBranch::GlobalY;
    next.orientation_seed = sum < (1.0/(1e10*1e10)) ? Vec3{0,0,1} : Vec3{0,1,0};
  } else {
    const double sum = ::sqrt(seed.x*seed.x+seed.y*seed.y+seed.z*seed.z);
    next.orientation_seed = {seed.x/sum,seed.y/sum,seed.z/sum};
  }
  next.length_m = next.length*units.length;
  for (unsigned i = 0; i < 2; ++i) next.endpoint_m[i] = Scale(input.position[i],units.length);
  if (!Positive(next.length_m) || !Finite(next.orientation_seed) ||
      !Finite(next.endpoint_m[0]) || !Finite(next.endpoint_m[1])) return Status::NonfiniteResult;
  output = next;
  return Status::Success;
}
} // namespace tl::fea::beam18::detail
