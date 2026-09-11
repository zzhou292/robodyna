// SPDX-License-Identifier: AGPL-3.0-or-later
// Fixed local packet/scatter only; coefficient production stays formulation-owned.
#pragma once
#include "../math/Quaternion.h"
#include <cstddef>

#if defined(__CUDACC__)
#define TL_SHELL_STIFFNESS_HD __host__ __device__
#else
#define TL_SHELL_STIFFNESS_HD
#endif

namespace tl::fea::shell_nodal_stiffness {
template<std::size_t Count>
struct Packet {
  double translation[Count]{};
  double rotation[Count]{};
};

// All local sums are checked before any write. Different source parents may
// share nodes; a single native cell must contain distinct global slots.
template<std::size_t Count>
TL_SHELL_STIFFNESS_HD inline bool Add(const std::size_t (&nodes)[Count],
    const Packet<Count>& value,double* translation,double* rotation,
    std::size_t count) noexcept {
  if (!translation || !rotation || translation==rotation) return false;
  double next_translation[Count],next_rotation[Count];
  for (std::size_t slot=0;slot<Count;++slot) {
    if (nodes[slot]>=count || !tl::math::Finite(value.translation[slot]) || value.translation[slot]<0 ||
        !tl::math::Finite(value.rotation[slot]) || value.rotation[slot]<0) return false;
    for (std::size_t prior=0;prior<slot;++prior) if (nodes[prior]==nodes[slot]) return false;
    if (!tl::math::Finite(translation[nodes[slot]]) || translation[nodes[slot]]<0 ||
        !tl::math::Finite(rotation[nodes[slot]]) || rotation[nodes[slot]]<0) return false;
    next_translation[slot]=translation[nodes[slot]]+value.translation[slot];
    next_rotation[slot]=rotation[nodes[slot]]+value.rotation[slot];
    if (!tl::math::Finite(next_translation[slot]) || !tl::math::Finite(next_rotation[slot])) return false;
  }
  for (std::size_t slot=0;slot<Count;++slot) {
    translation[nodes[slot]]=next_translation[slot];
    rotation[nodes[slot]]=next_rotation[slot];
  }
  return true;
}
} // namespace tl::fea::shell_nodal_stiffness

#undef TL_SHELL_STIFFNESS_HD
