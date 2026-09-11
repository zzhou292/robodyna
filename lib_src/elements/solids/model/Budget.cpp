// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"

namespace tl::fea::solids::model_detail {
ModelReport Preflight(const NodalNodeDomain& domain,ModelInput input,ModelLimits limits,
                       std::size_t fixed,Layout& layout) {
  const ModelLimits hard;
  const auto resource=[&](const char* text){return Error(ModelStatus::ResourceLimit,text,input);};
  if(!domain.prepared() || input.source_instance_id!=domain.source_instance_id())
    return Error(ModelStatus::InvalidInput,"Prepared matching source domain required",input);
  if(!limits.max_parents || limits.max_parents>hard.max_parents ||
      !limits.max_materials || limits.max_materials>hard.max_materials ||
      !limits.max_curve_points || limits.max_curve_points>hard.max_curve_points ||
      !limits.max_nodes || limits.max_nodes>hard.max_nodes ||
      !limits.max_host_bytes || limits.max_host_bytes>hard.max_host_bytes ||
      domain.node_count()>limits.max_nodes || input.solid18.size()>limits.max_parents ||
      input.solid24.size()>limits.max_parents || input.solid6z.size()>limits.max_parents)
    return resource("Solid model limits exceed the bounded scope");
  const auto count=Count(input); // Three previously bounded counts cannot overflow.
  if(count>limits.max_parents)return resource("Combined solid parent count exceeds cap");
  if(!count)return Error(ModelStatus::InvalidInput,"Solid model requires a parent",input);
  util::BoundedArenaLayout scratch(limits.max_host_bytes),minimum(limits.max_host_bytes);
  util::ArenaRegion unused;
  if(!scratch.Append<solid18::Reference>(input.solid18.size(),layout.reference18) ||
      !scratch.Append<solid24::Reference>(input.solid24.size(),layout.reference24) ||
      !scratch.Append<solid6z::Reference>(input.solid6z.size(),layout.reference6z) ||
      !scratch.Append<std::size_t>(count,layout.material_indices) ||
      !minimum.Append<unsigned char>(fixed+sizeof(Scratch),unused) ||
      !minimum.Append<unsigned char>(scratch.bytes(),unused) ||
      !minimum.Append<unsigned char>(Index::Bytes(count)-sizeof(Index),unused) ||
      !minimum.Append<unsigned char>(domain.owned_payload_bytes(),unused) ||
      !minimum.Append<Parent18>(input.solid18.size(),unused) ||
      !minimum.Append<Parent24>(input.solid24.size(),unused) ||
      !minimum.Append<Parent6z>(input.solid6z.size(),unused) ||
      !minimum.Append<SolidCoefficientParent>(count,unused))
    return resource("Solid model lower-bound bytes exceed cap before borrowed reads");
  if(!Range(input.solid18.data(),input.solid18.size()) ||
      !Range(input.solid24.data(),input.solid24.size()) ||
      !Range(input.solid6z.data(),input.solid6z.size()))
    return Error(ModelStatus::InvalidInput,"Solid parent pointer/count pair is invalid",input);
  layout.fixed_bytes=fixed;
  layout.scratch_arena_bytes=scratch.bytes();
  layout.scratch_bytes=sizeof(Scratch)+scratch.bytes()+Index::Bytes(count)-sizeof(Index)+
      sizeof(SolidNodeContributions)+sizeof(std::shared_ptr<void>);
  return {};
}
ModelReport CompleteLayout(ModelInput input,ModelLimits limits,Layout& layout) {
  util::BoundedArenaLayout arena(limits.max_host_bytes),startup(limits.max_host_bytes);
  util::ArenaRegion unused;
  if(!arena.Append<Parent18>(input.solid18.size(),layout.parent18) ||
      !arena.Append<Parent24>(input.solid24.size(),layout.parent24) ||
      !arena.Append<Parent6z>(input.solid6z.size(),layout.parent6z) ||
      !arena.Append<Material36>(layout.count36,layout.material36) ||
      !arena.Append<Material42>(layout.count42,layout.material42) ||
      !arena.Append<double>(2*layout.curve_points,layout.curves) ||
      !startup.Append<unsigned char>(layout.fixed_bytes,unused) ||
      !startup.Append<unsigned char>(arena.bytes(),unused) ||
      !startup.Append<unsigned char>(layout.scratch_bytes,unused))
    return Error(ModelStatus::ResourceLimit,"Complete solid material and staging bytes exceed cap",input);
  layout.arena_bytes=arena.bytes();
  layout.owned_bytes=layout.fixed_bytes+arena.bytes();
  layout.startup_bytes=startup.bytes(); // Coefficient/domain payload is added after its bounded construction.
  return {};
}
} // namespace tl::fea::solids::model_detail
