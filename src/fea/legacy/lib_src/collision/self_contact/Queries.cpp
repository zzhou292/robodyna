// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tlfea::contact {
namespace {
template<class T> tl::util::ConstView<T> View(const T* values, std::size_t count) noexcept {
  static const T empty{};
  return {values ? values : &empty, count};
}
}
tl::util::ConstView<SelfContactSurfaceParent> SelfContactSurfaceBinding::parents() const noexcept {
  return View(impl_ ? impl_->features.parents : nullptr, impl_ ? impl_->forecast.parent_count : 0);
}
tl::util::ConstView<SelfContactVertex> SelfContactSurfaceBinding::vertices() const noexcept {
  return View(impl_ ? impl_->features.vertices : nullptr, impl_ ? impl_->features.vertex_count : 0);
}
tl::util::ConstView<SelfContactEdge> SelfContactSurfaceBinding::edges() const noexcept {
  return View(impl_ ? impl_->features.edges : nullptr, impl_ ? impl_->features.edge_count : 0);
}
tl::util::ConstView<SelfContactVertexUse> SelfContactSurfaceBinding::vertex_uses() const noexcept {
  return View(impl_ ? impl_->features.vertex_uses : nullptr, impl_ ? impl_->features.use_count : 0);
}
tl::util::ConstView<SelfContactEdgeUse> SelfContactSurfaceBinding::edge_uses() const noexcept {
  return View(impl_ ? impl_->features.edge_uses : nullptr, impl_ ? impl_->features.use_count : 0);
}
tl::util::ConstView<std::uint32_t> SelfContactSurfaceBinding::faces() const noexcept {
  return View(impl_ ? impl_->features.faces : nullptr, impl_ ? impl_->forecast.parent_count : 0);
}
TopologicalIncidence SelfContactSurfaceBinding::VertexInFace(std::size_t vertex, std::size_t parent) const noexcept {
  if (vertex >= vertices().size() || parent >= parents().size()) return TopologicalIncidence::Invalid;
  const auto& face = parents()[parent];
  for (unsigned local = 0; local < face.arity; ++local)
    if (face.vertices[local] == vertex) return TopologicalIncidence::Incident;
  return TopologicalIncidence::Disjoint;
}
TopologicalIncidence SelfContactSurfaceBinding::EdgesShareVertex(std::size_t a, std::size_t b) const noexcept {
  if (a >= edges().size() || b >= edges().size()) return TopologicalIncidence::Invalid;
  for (unsigned i = 0; i < 2; ++i)
    for (unsigned j = 0; j < 2; ++j)
      if (edges()[a].vertices[i] == edges()[b].vertices[j]) return TopologicalIncidence::Incident;
  return TopologicalIncidence::Disjoint;
}
} // namespace tlfea::contact
