// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"

namespace tl::fea::type45::resident_detail {
BatchReport BuildUpload(const BatchConfig& config,const Model& model,util::HostArena& arena,
    const ArenaLayout& layout) {
  auto* storage=arena.Construct<Storage>(layout.header);
  auto* joints=arena.Construct<Joint>(layout.joints);
  if(!storage || !joints || !arena.Construct<State>(layout.slab[0]) ||
      !arena.Construct<State>(layout.slab[1]) ||
      !arena.Construct<AutomaticStiffnessContext>(layout.contexts) || !arena.Construct<Status>(layout.status))
    return {BatchStatus::ResourceLimit,"Joint upload arena cannot construct typed values"};
  for(std::size_t j=0;j<model.joints().size();++j) joints[j]=model.joints()[j];
  auto next=RebasedHeader(arena.data(),layout);
  next.config=config;next.source_instance_id=model.source_instance_id();
  *storage=next;return {};
}
} // namespace tl::fea::type45::resident_detail
