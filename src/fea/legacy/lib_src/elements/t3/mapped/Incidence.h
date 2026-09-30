// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../mapped_shell/Incidence.h"

namespace tl::fea::t3::mapped {
template<class Element>
bool BuildIncidence(const Element* elements, std::size_t parents, std::size_t nodes,
    std::uint32_t* offsets, std::size_t offset_count,
    std::uint32_t* incidence, std::size_t incidence_count) noexcept {
  return mapped_shell::BuildIncidence<3>(elements, parents, nodes,
      offsets, offset_count, incidence, incidence_count);
}
} // namespace tl::fea::t3::mapped
