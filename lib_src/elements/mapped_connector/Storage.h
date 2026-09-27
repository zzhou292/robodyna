// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../mapped_shell/NodeGather.h"
#include "../mapped_shell/Incidence.h"
#include "../../../lib_utils/BoundedArena.h"
namespace tl::fea::mapped_connector {
enum class Failure { None, Endpoint, Coefficient, Assembly };
enum class Order { AllEndpointsFirst, ParentThenAssembly };
struct Parent {
  Failure failure = Failure::None;
  std::size_t node = SIZE_MAX;
  double translation = 0, rotation = 0;
};
struct Memory {
  std::uint32_t* offsets = nullptr;
  std::uint32_t* incidence = nullptr;
  std::uint32_t* touched_nodes = nullptr;
  std::size_t touched_count = 0;
  Parent* parent = nullptr;
  mapped_shell::AssemblyNode* node = nullptr;
  unsigned long long* failure = nullptr;
};
struct Layout {
  util::ArenaRegion offsets, incidence, touched_nodes, parent, node, failure;
};
inline bool AppendLayout(util::BoundedArenaLayout& arena, std::size_t parents,
    std::size_t nodes, Layout& output) noexcept {
  if (!parents || parents > UINT32_MAX / 2 || !nodes || nodes >= UINT32_MAX) return false;
  const auto touched = nodes < 2 * parents ? nodes : 2 * parents;
  Layout next;
  if (!arena.Append<std::uint32_t>(nodes + 1, next.offsets) ||
      !arena.Append<std::uint32_t>(2 * parents, next.incidence) ||
      !arena.Append<std::uint32_t>(touched, next.touched_nodes) ||
      !arena.Append<Parent>(parents, next.parent) ||
      !arena.Append<mapped_shell::AssemblyNode>(touched, next.node) ||
      !arena.Append<unsigned long long>(1, next.failure)) return false;
  output = next;
  return true;
}
inline Memory Rebase(void* base, const Layout& layout) noexcept {
  if (!layout.failure.count) return {};
  return {util::ArenaPointer<std::uint32_t>(base, layout.offsets),
      util::ArenaPointer<std::uint32_t>(base, layout.incidence),
      util::ArenaPointer<std::uint32_t>(base, layout.touched_nodes), 0,
      util::ArenaPointer<Parent>(base, layout.parent),
      util::ArenaPointer<mapped_shell::AssemblyNode>(base, layout.node),
      util::ArenaPointer<unsigned long long>(base, layout.failure)};
}
inline bool Construct(util::HostArena& arena, const Layout& layout) noexcept {
  return !layout.failure.count || (arena.Construct<std::uint32_t>(layout.offsets) &&
      arena.Construct<std::uint32_t>(layout.incidence) &&
      arena.Construct<std::uint32_t>(layout.touched_nodes) && arena.Construct<Parent>(layout.parent) &&
      arena.Construct<mapped_shell::AssemblyNode>(layout.node) &&
      arena.Construct<unsigned long long>(layout.failure));
}
template<class Element>
inline bool Build(const Element* elements, std::size_t parents, std::size_t nodes,
    const Layout& layout, Memory& memory) noexcept {
  if (!layout.failure.count) return true;
  if (!mapped_shell::BuildIncidence<2>(elements, parents, nodes, memory.offsets,
          layout.offsets.count, memory.incidence, layout.incidence.count)) return false;
  std::size_t touched = 0;
  for (std::size_t node = 0; node < nodes; ++node) if (memory.offsets[node] != memory.offsets[node + 1]) {
    if (touched == layout.touched_nodes.count) return false;
    memory.touched_nodes[touched++] = static_cast<std::uint32_t>(node);
  }
  memory.touched_count = touched;
  return true;
}
} // namespace tl::fea::mapped_connector
