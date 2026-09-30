// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid6zTypes.h"

namespace tl::fea::solid6z {
// Explicit topology conversion for raw [A,B,C,D,E,E,F,F]. It is not a
// formulation selector or an emulation of the OpenRadioss converter.
struct CollapsedBrickTopology {
  std::uint64_t raw_source_node_id[8]{};
  std::uint8_t six_to_raw[6]{};
};
TL_BRICK_HD inline Status MapCollapsedTopEdges(const std::uint64_t (&raw)[8],
                                              CollapsedBrickTopology& output) noexcept {
  constexpr unsigned unique[6]{0,1,2,3,4,6};
  if (raw[4] != raw[5] || raw[6] != raw[7]) return Status::UnsupportedProfile;
  for (unsigned i = 0; i < 6; ++i) {
    if (!raw[unique[i]]) return Status::InvalidInput;
    for (unsigned j = 0; j < i; ++j) {
      if (raw[unique[i]] == raw[unique[j]]) return Status::UnsupportedProfile;
    }
  }
  CollapsedBrickTopology next;
  for (unsigned n = 0; n < 8; ++n) next.raw_source_node_id[n] = raw[n];
  constexpr unsigned order[6]{0,1,4,3,2,6};
  for (unsigned n = 0; n < 6; ++n) next.six_to_raw[n] = order[n];
  output = next;
  return Status::Success;
}
}  // namespace tl::fea::solid6z
