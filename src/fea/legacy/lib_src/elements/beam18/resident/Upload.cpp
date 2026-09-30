// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"

namespace tl::fea::beam18::batch_detail {
Material ExpectedMaterial(const Model& model, std::size_t index, double* curves) noexcept {
  std::size_t offset = 0;
  for (std::size_t m = 0; m < index; ++m) offset += 2 * model.materials()[m].value.curve.count;
  auto value = model.materials()[index].value;
  value.curve.plastic_strain = curves + offset;
  value.curve.yield_stress_pa = curves + offset + value.curve.count;
  return value;
}
BatchReport BuildUpload(const BatchConfig& config, const Model& model,
    util::HostArena& arena, const ArenaLayout& layout, Storage& output) {
  auto* header = arena.Construct<Storage>(layout.header);
  auto* parents = arena.Construct<Parent>(layout.parents);
  auto* materials = arena.Construct<Material>(layout.materials);
  auto* curves = arena.Construct<double>(layout.curves);
  if (!header || !parents || !materials || !curves ||
      !arena.Construct<State>(layout.slab[0]) || !arena.Construct<State>(layout.slab[1]) ||
      !arena.Construct<int>(layout.status))
    return {BatchStatus::ResourceLimit, "Beam upload cannot construct complete typed storage"};
  for (std::size_t p = 0; p < model.parents().size(); ++p) parents[p] = model.parents()[p];
  auto* cursor = curves;
  for (std::size_t m = 0; m < model.materials().size(); ++m) {
    const auto& curve = model.materials()[m].value.curve;
    for (std::size_t i = 0; i < curve.count; ++i) {
      cursor[i] = curve.plastic_strain[i];
      cursor[curve.count + i] = curve.yield_stress_pa[i];
    }
    cursor += 2 * curve.count;
    materials[m] = ExpectedMaterial(model, m, curves);
  }
  auto next = RebasedHeader(arena.data(), layout);
  next.config = config;
  next.source_instance_id = model.source_instance_id();
  *header = next;
  output = next;
  return {};
}
void RebaseCurves(const Model& model, const ArenaLayout& layout, void* device,
    Storage& host_header) noexcept {
  auto* curves = util::ArenaPointer<double>(device, layout.curves);
  for (std::size_t m = 0; m < model.materials().size(); ++m)
    host_header.materials[m] = ExpectedMaterial(model, m, curves);
}
} // namespace tl::fea::beam18::batch_detail
