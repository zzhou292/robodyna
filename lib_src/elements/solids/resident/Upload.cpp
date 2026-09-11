// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"

namespace tl::fea::solids::batch_detail {
namespace {
template<class Traits> bool CopyFamily(util::ConstView<typename Traits::Parent> source,
    util::HostArena& arena, const FamilyLayout& layout) {
  if (!source.size()) return true;
  auto* parents = arena.Construct<typename Traits::Parent>(layout.parents);
  if (!parents || !arena.Construct<State<Traits>>(layout.slab[0]) ||
      !arena.Construct<State<Traits>>(layout.slab[1]) ||
      !arena.Construct<int>(layout.status)) return false;
  for (std::size_t p = 0; p < source.size(); ++p) parents[p] = source[p];
  return true;
}
} // namespace
BatchReport BuildUpload(const BatchConfig& config, const Model& model,
    util::HostArena& arena, const ArenaLayout& layout, Storage& output) {
  auto* storage = arena.Construct<Storage>(layout.header);
  if (!storage || !CopyFamily<Traits18>(model.solid18(), arena, layout.solid18) ||
      !CopyFamily<Traits24>(model.solid24(), arena, layout.solid24) ||
      !CopyFamily<Traits6z>(model.solid6z(), arena, layout.solid6z)) {
    return {BatchStatus::ResourceLimit, "Solid upload arena cannot construct typed parents"};
  }
  auto next = RebasedHeader(arena.data(), layout);
  next.config = config;
  next.source_instance_id = model.source_instance_id();
  if (layout.material36.count) {
    auto* materials = arena.Construct<solid18::Material>(layout.material36);
    auto* curves = arena.Construct<double>(layout.curves);
    if (!materials || !curves) {
      return {BatchStatus::ResourceLimit, "Solid owned curve upload layout differs"};
    }
    std::size_t cursor = 0;
    for (std::size_t m = 0; m < model.materials36().size(); ++m) {
      materials[m] = model.materials36()[m].value;
      const auto& curve = materials[m].curve;
      for (std::size_t p = 0; p < curve.count; ++p) {
        curves[cursor + p] = curve.plastic_strain[p];
        curves[cursor + curve.count + p] = curve.yield_stress_pa[p];
      }
      materials[m].curve.plastic_strain = curves + cursor;
      materials[m].curve.yield_stress_pa = curves + cursor + curve.count;
      cursor += 2 * curve.count;
    }
  }
  if (layout.material42.count) {
    auto* materials = arena.Construct<solid24::Material>(layout.material42);
    if (!materials) return {BatchStatus::ResourceLimit, "Solid scalar material upload layout differs"};
    for (std::size_t m = 0; m < model.materials42().size(); ++m)
      materials[m] = model.materials42()[m].value;
  }
  *storage = next;
  output = next;
  return {};
}
void RebaseCurves(const Model& model, const ArenaLayout& layout, void* device,
    Storage& header) noexcept {
  if (!layout.material36.count) return;
  auto* curves = util::ArenaPointer<double>(device, layout.curves);
  std::size_t cursor = 0;
  for (std::size_t m = 0; m < model.materials36().size(); ++m) {
    auto& material = header.material36[m];
    material.curve.plastic_strain = curves + cursor;
    material.curve.yield_stress_pa = curves + cursor + material.curve.count;
    cursor += 2 * material.curve.count;
  }
}
} // namespace tl::fea::solids::batch_detail
