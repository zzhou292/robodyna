// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"

namespace tl::fea::solids::model_detail {
ModelReport Preflight(const NodalNodeDomain& domain,ModelInput input,ModelLimits limits,
                       std::size_t fixed,Layout& layout) {
  const ModelLimits hard;
  const auto resource=[&](const char* text){return Error(ModelStatus::ResourceLimit,text,input);};
  if(!domain.prepared() || input.source_instance_id!=domain.source_instance_id())
    return Error(ModelStatus::InvalidInput,"Prepared matching source domain required",input);
  const bool extra = input.solid18_law44.size() || input.solid18_law90.size();
  if ((input.profile == ModelProfile::OriginalThreeFamilies &&
       (extra || input.solid18_law44.data() || input.solid18_law90.data())) ||
      (input.profile == ModelProfile::ExtendedLaw44Law90 && !extra) ||
      (input.profile != ModelProfile::OriginalThreeFamilies && input.profile != ModelProfile::ExtendedLaw44Law90))
    return Error(ModelStatus::InvalidInput,"Solid model extension requires its explicit profile",input);
  if(!limits.max_parents || limits.max_parents>hard.max_parents ||
      !limits.max_materials || limits.max_materials>hard.max_materials ||
      !limits.max_curve_points || limits.max_curve_points>hard.max_curve_points ||
      !limits.max_nodes || limits.max_nodes>hard.max_nodes ||
      !limits.max_host_bytes || limits.max_host_bytes>hard.max_host_bytes ||
      domain.node_count()>limits.max_nodes || input.solid18.size()>limits.max_parents ||
      input.solid24.size()>limits.max_parents || input.solid6z.size()>limits.max_parents ||
      input.solid18_law44.size()>limits.max_parents || input.solid18_law90.size()>limits.max_parents)
    return resource("Solid model limits exceed the bounded scope");
  const auto count=Count(input); // Five previously bounded counts cannot overflow.
  if(count>limits.max_parents)return resource("Combined solid parent count exceeds cap");
  if(!count)return Error(ModelStatus::InvalidInput,"Solid model requires a parent",input);
  control::Limits control_limits;
  control_limits.max_parents=limits.max_parents;control_limits.max_packets=limits.max_parents;
  control_limits.max_host_bytes=limits.max_host_bytes;
  const auto control_report=control::Selection::Forecast(input.controls,count,control_limits,layout.control_budget);
  if(!control_report)return Error(control_report.status==control::Status::ResourceLimit?
      ModelStatus::ResourceLimit:ModelStatus::InvalidInput,control_report.message,input);

  util::BoundedArenaLayout scratch(limits.max_host_bytes),minimum(limits.max_host_bytes);
  util::ArenaRegion unused;
  if(!scratch.Append<solid18::Reference>(input.solid18.size(),layout.reference18) ||
      !scratch.Append<solid24::Reference>(input.solid24.size(),layout.reference24) ||
      !scratch.Append<solid6z::Reference>(input.solid6z.size(),layout.reference6z) ||
      !scratch.Append<solid18::law44::Reference>(input.solid18_law44.size(),layout.reference44) ||
      !scratch.Append<solid18::total_strain::Reference>(input.solid18_law90.size(),layout.reference90) ||
      !scratch.Append<std::size_t>(count,layout.material_indices) ||
      !minimum.Append<unsigned char>(fixed+sizeof(Scratch),unused) ||
      !minimum.Append<unsigned char>(scratch.bytes(),unused) ||
      !minimum.Append<unsigned char>(Index::Bytes(count)-sizeof(Index),unused) ||
      !minimum.Append<unsigned char>(domain.owned_payload_bytes(),unused) ||
      !minimum.Append<unsigned char>(layout.control_budget.startup_bytes-sizeof(control::Selection),unused) ||
      !minimum.Append<Parent18>(input.solid18.size(),unused) ||
      !minimum.Append<Parent24>(input.solid24.size(),unused) ||
      !minimum.Append<Parent6z>(input.solid6z.size(),unused) ||
      !minimum.Append<Parent18Law44>(input.solid18_law44.size(),unused) ||
      !minimum.Append<Parent18Law90>(input.solid18_law90.size(),unused) ||
      !minimum.Append<SolidCoefficientParent>(count,unused))
    return resource("Solid model lower-bound bytes exceed cap before borrowed reads");
  if(!Range(input.solid18.data(),input.solid18.size()) ||
      !Range(input.solid24.data(),input.solid24.size()) ||
      !Range(input.solid6z.data(),input.solid6z.size()) ||
      !Range(input.solid18_law44.data(),input.solid18_law44.size()) ||
      !Range(input.solid18_law90.data(),input.solid18_law90.size()))
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
      !arena.Append<Parent18Law44>(input.solid18_law44.size(),layout.parent44) ||
      !arena.Append<Parent18Law90>(input.solid18_law90.size(),layout.parent90) ||
      !arena.Append<Material36>(layout.count36,layout.material36) ||
      !arena.Append<Material42>(layout.count42,layout.material42) ||
      !arena.Append<Material44>(layout.count44,layout.material44) ||
      !arena.Append<Material90>(layout.count90,layout.material90) ||
      !arena.Append<double>(2*layout.curve_points,layout.curves) ||
      !startup.Append<unsigned char>(layout.fixed_bytes,unused) ||
      !startup.Append<unsigned char>(arena.bytes(),unused) ||
      !startup.Append<unsigned char>(layout.scratch_bytes,unused) ||
      !startup.Append<unsigned char>(layout.control_budget.startup_bytes-sizeof(control::Selection),unused))
    return Error(ModelStatus::ResourceLimit,"Complete solid material and staging bytes exceed cap",input);
  layout.arena_bytes=arena.bytes();
  layout.owned_bytes=layout.fixed_bytes+arena.bytes()+layout.control_budget.owned_bytes-sizeof(control::Selection);
  layout.startup_bytes=startup.bytes(); // Coefficient/domain payload is added after its bounded construction.
  return {};
}
} // namespace tl::fea::solids::model_detail
