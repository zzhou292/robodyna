// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MaterialUpload.h"

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
      !CopyFamily<Traits6z>(model.solid6z(), arena, layout.solid6z) ||
      !CopyFamily<Traits18Law44>(model.solid18_law44(), arena, layout.solid18_law44) ||
      !CopyFamily<Traits18Law90>(model.solid18_law90(), arena, layout.solid18_law90)) {
    return {BatchStatus::ResourceLimit, "Solid upload arena cannot construct typed parents"};
  }
  if ((layout.scratch18.count && !arena.Construct<Scratch18>(layout.scratch18)) ||
      (layout.scratch44.count && !arena.Construct<ExtendedScratch<Traits18Law44>>(layout.scratch44)) ||
      (layout.scratch90.count && !arena.Construct<ExtendedScratch<Traits18Law90>>(layout.scratch90))) {
    return {BatchStatus::ResourceLimit, "Solid18 force scratch cannot be constructed"};
  }
  auto next = RebasedHeader(arena.data(), layout);
  next.config = config;
  next.source_instance_id = model.source_instance_id();
  const auto report = UploadMaterials(model, arena, layout);
  if (!report) return report;
  *storage = next;
  output = next;
  return {};
}
} // namespace tl::fea::solids::batch_detail
