// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/elements/solid18/Solid18Orientation.h"

namespace tl::fea::solid18::law44::detail {
TL_SOLID18_HD inline bool SameScalar(double a, double b) noexcept {
  // All coordinates were checked finite. Equality plus the sign distinguishes
  // both zero encodings and therefore compares every admitted binary64 bit.
  return a == b && ::copysign(1.0, a) == ::copysign(1.0, b);
}
TL_SOLID18_HD inline bool SamePosition(Vec3 a, Vec3 b) noexcept {
  return SameScalar(a.x, b.x) && SameScalar(a.y, b.y) && SameScalar(a.z, b.z);
}
TL_SOLID18_HD inline bool Supported(const ResolvedProfile& p) noexcept {
  return p.material_law == 44 && p.native_isolid == 18 && p.engine_jhbe == 17 &&
         p.integration == 2 && p.nptr == 2 && p.npts == 2 && p.nptt == 2 &&
         p.pressure == 1 && p.small_strain == 2 && p.convected_frame == 1;
}
TL_SOLID18_HD inline bool ValidateTopology(const ReferenceInput& in,
                                         SourceTopology& output) noexcept {
  if (!in.source_element_id || !in.source_part_id || !in.source_section_id ||
      !in.source_material_id || !solid18::detail::Positive(in.density_kg_m3)) return false;
  for (unsigned i = 0; i < 8; ++i) {
    if (!in.source_node_id[i] || !solid18::detail::Finite(in.position_m[i])) return false;
  }
  bool repeated = false;
  for (unsigned i = 0; i < 8; ++i) {
    for (unsigned j = 0; j < i; ++j) {
      if (in.source_node_id[i] != in.source_node_id[j]) continue;
      if (!((i == 5 && j == 4) || (i == 7 && j == 6)) ||
          !SamePosition(in.position_m[i], in.position_m[j])) return false;
      repeated = true;
    }
  }
  if (repeated && (in.source_node_id[4] != in.source_node_id[5] ||
                   in.source_node_id[6] != in.source_node_id[7])) return false;
  output = repeated ? SourceTopology::RepeatedPairs56And78 : SourceTopology::EightDistinct;
  return true;
}
}  // namespace tl::fea::solid18::law44::detail
