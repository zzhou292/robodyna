// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>

namespace tl::fea::qeph::mapped {
// Flat equivalent of cpu_utils::BuildNodeIncidence's parent/local traversal.
// Own arena scratch only. All borrowed connectivity is checked before writes;
// prefix offsets double as insertion cursors, so no nested vectors are needed.
template<class Element>
bool BuildIncidence(const Element* elements,std::size_t parents,std::size_t nodes,
    std::uint32_t* offsets,std::size_t offset_count,
    std::uint32_t* incidence,std::size_t incidence_count) noexcept {
  if (!parents || parents>UINT32_MAX/4 || !nodes || nodes>=UINT32_MAX ||
      offset_count!=nodes+1 || incidence_count!=4*parents ||
      !elements || !offsets || !incidence) return false;
  for (std::size_t parent=0;parent<parents;++parent) for (unsigned slot=0;slot<4;++slot) {
    const auto node=elements[parent].nodes[slot];
    if (node>=nodes) return false;
    for (unsigned prior=0;prior<slot;++prior) if (node==elements[parent].nodes[prior]) return false;
  }
  for (std::size_t node=0;node<=nodes;++node) offsets[node]=0;
  for (std::size_t parent=0;parent<parents;++parent) for (auto node:elements[parent].nodes) ++offsets[node+1];
  for (std::size_t node=1;node<=nodes;++node) offsets[node]+=offsets[node-1];
  for (std::size_t parent=0;parent<parents;++parent) for (unsigned slot=0;slot<4;++slot) {
    incidence[offsets[elements[parent].nodes[slot]]++]=static_cast<std::uint32_t>(4*parent+slot);
  }
  for (std::size_t node=nodes;node>0;--node) offsets[node]=offsets[node-1];
  offsets[0]=0;
  return true;
}
} // namespace tl::fea::qeph::mapped
