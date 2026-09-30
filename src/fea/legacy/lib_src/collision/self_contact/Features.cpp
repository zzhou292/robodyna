// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <algorithm>

namespace tlfea::contact::self_contact {
namespace {
std::uint32_t Node(const SelfContactSurfaceParent& parent, unsigned local) noexcept {
  return parent.arity == 4 ? parent.q4.nodes[local] : parent.t3.nodes[local];
}
}
SelfContactSurfaceReport BuildFeatures(const tl::fea::ShellPhysicalBinding& physical,
    const SelfContactSurfaceForecast& forecast, Features& f) noexcept {
  for (std::size_t p = 0; p < forecast.parent_count; ++p) {
    f.faces[p] = static_cast<std::uint32_t>(p);
    const auto& parent = f.parents[p];
    for (unsigned local = 0; local < parent.arity; ++local) {
      const auto node = Node(parent, local);
      const auto next = Node(parent, (local + 1) % parent.arity);
      const auto nid = physical.domain()->nodes()[node].source_id;
      const auto other = physical.domain()->nodes()[next].source_id;
      f.vertex_uses[f.use_count] = {nid, parent.source.source_parent_id, node,
          static_cast<std::uint32_t>(p), local};
      f.edge_uses[f.use_count] = {{std::min(nid, other), std::max(nid, other)},
          parent.source.source_parent_id, static_cast<std::uint32_t>(p), local};
      ++f.use_count;
    }
  }
  std::sort(f.faces, f.faces + forecast.parent_count, [&](auto a, auto b) {
    return f.parents[a].source.source_parent_id < f.parents[b].source.source_parent_id;
  });
  std::sort(f.vertex_uses, f.vertex_uses + f.use_count, [](const auto& a, const auto& b) {
    if (a.source_node_id != b.source_node_id) return a.source_node_id < b.source_node_id;
    if (a.source_parent_id != b.source_parent_id) return a.source_parent_id < b.source_parent_id;
    return a.local < b.local;
  });
  for (std::size_t i = 0; i < f.use_count; ++i) {
    const auto& use = f.vertex_uses[i];
    if (!i || use.source_node_id != f.vertex_uses[i - 1].source_node_id) {
      if (f.vertex_count == forecast.vertex_capacity)
        return {SelfContactSurfaceStatus::ResourceLimit, use.parent, "Unique source vertex cap exceeded"};
      f.vertices[f.vertex_count++] = {use.source_node_id, use.domain_node, static_cast<std::uint32_t>(i), 0};
    }
    const auto vertex = static_cast<std::uint32_t>(f.vertex_count - 1);
    ++f.vertices[vertex].use_count;
    f.parents[use.parent].vertices[use.local] = vertex;
  }
  std::sort(f.edge_uses, f.edge_uses + f.use_count, [](const auto& a, const auto& b) {
    if (a.source_node_ids[0] != b.source_node_ids[0]) return a.source_node_ids[0] < b.source_node_ids[0];
    if (a.source_node_ids[1] != b.source_node_ids[1]) return a.source_node_ids[1] < b.source_node_ids[1];
    if (a.source_parent_id != b.source_parent_id) return a.source_parent_id < b.source_parent_id;
    return a.local < b.local;
  });
  for (std::size_t i = 0; i < f.use_count; ++i) {
    const auto& use = f.edge_uses[i];
    const bool first = !i || use.source_node_ids[0] != f.edge_uses[i - 1].source_node_ids[0] ||
        use.source_node_ids[1] != f.edge_uses[i - 1].source_node_ids[1];
    if (first) {
      if (f.edge_count == forecast.edge_capacity)
        return {SelfContactSurfaceStatus::ResourceLimit, use.parent, "Unique source edge cap exceeded"};
      const auto& parent = f.parents[use.parent];
      auto a = parent.vertices[use.local];
      auto b = parent.vertices[(use.local + 1) % parent.arity];
      if (f.vertices[a].source_node_id > f.vertices[b].source_node_id) std::swap(a, b);
      f.edges[f.edge_count++] = {{use.source_node_ids[0], use.source_node_ids[1]}, {a, b},
          static_cast<std::uint32_t>(i), 0};
    }
    const auto edge = static_cast<std::uint32_t>(f.edge_count - 1);
    ++f.edges[edge].use_count;
    f.parents[use.parent].edges[use.local] = edge;
  }
  return {};
}
} // namespace tlfea::contact::self_contact
