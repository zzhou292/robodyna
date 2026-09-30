#pragma once
#include "lib_src/collision/broadphase/Sweep.h"
#include "lib_src/collision/SelfContactBroadphaseTypes.h"
#include <algorithm>
#include <vector>

namespace surface_broadphase_test {
using Key = tlfea::contact::SelfContactPairKey;
inline Key Canonical(int a, int b) {
  return (static_cast<Key>(std::min(a, b)) << 32) | static_cast<unsigned>(std::max(a, b));
}
// Independent exhaustive O(n^2) oracle: no sweep stop, sort order or production
// filter helper participates in the expected complete set.
inline std::vector<Key> Exhaustive(const std::vector<AABB>& boxes) {
  std::vector<Key> out;
  for (std::size_t a = 0; a < boxes.size(); ++a)
    for (std::size_t b = a + 1; b < boxes.size(); ++b) {
      const double lo_a[]{boxes[a].min.x, boxes[a].min.y, boxes[a].min.z};
      const double hi_a[]{boxes[a].max.x, boxes[a].max.y, boxes[a].max.z};
      const double lo_b[]{boxes[b].min.x, boxes[b].min.y, boxes[b].min.z};
      const double hi_b[]{boxes[b].max.x, boxes[b].max.y, boxes[b].max.z};
      bool separated = false;
      for (unsigned axis = 0; axis < 3; ++axis)
        separated = separated || hi_a[axis] < lo_b[axis] || hi_b[axis] < lo_a[axis];
      if (!separated) out.push_back(Canonical(boxes[a].objectId, boxes[b].objectId));
    }
  std::sort(out.begin(), out.end());
  return out;
}
} // namespace surface_broadphase_test
