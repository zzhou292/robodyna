// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tl::fea::beam18::model_detail {
ModelReport Count(const NodalNodeDomain& domain,ModelInput input,ModelLimits limits,
    std::size_t implementation_bytes) noexcept {
  if(!domain.prepared()||!input.source_instance_id||input.source_instance_id!=domain.source_instance_id()||
      input.profile!=ModelProfile::CircularFourPointLaw44V1||!input.parents.size())
    return {S::InvalidInput,"Prepared domain and explicit beam source/profile are required"};
  const ModelLimits hard;
  if(!limits.max_parents||limits.max_parents>hard.max_parents||!limits.max_materials||
      limits.max_materials>hard.max_materials||!limits.max_curve_points||limits.max_curve_points>hard.max_curve_points||
      !limits.max_nodes||limits.max_nodes>hard.max_nodes||!limits.max_host_bytes||limits.max_host_bytes>hard.max_host_bytes||
      input.parents.size()>limits.max_parents||domain.node_count()>limits.max_nodes)
    return {S::ResourceLimit,"Beam model counts or caps exceed scope"};
  util::BoundedArenaLayout preflight(limits.max_host_bytes);
  util::ArenaRegion ignored;
  if(!preflight.Append<unsigned char>(sizeof(Model)+implementation_bytes+64,ignored)||
      !preflight.Append<unsigned char>(domain.owned_payload_bytes(),ignored)||
      !preflight.Append<unsigned char>(2*Index::Bytes(input.parents.size()),ignored))
    return {S::ResourceLimit,"Beam retained domain and identity scratch exceed cap"};
  if(!nodal_domain_detail::ValidRange(input.parents.data(),input.parents.size()))
    return {S::InvalidInput,"Invalid beam parent range"};
  return {};
}
ModelReport Inventory(ModelInput input,ModelLimits limits,const Index& elements,const Index& materials,
    Plan& plan) noexcept {
  for(std::size_t p=0;p<input.parents.size();++p) {
    const auto& row=input.parents[p]; const auto& ref=row.reference;
    if(!ref.prepared()||!force_detail::MaterialValid(ref,row.material))
      return {S::InvalidInput,"Invalid prepared beam reference/material",p};
    if(elements.First(ref.input().source_element_id)!=p)
      return {S::DuplicateIdentity,"Repeated beam source EID",p};
    const auto& curve=row.material.curve;
    if(!nodal_domain_detail::ValidRange(curve.plastic_strain,curve.count)||
        !nodal_domain_detail::ValidRange(curve.yield_stress_pa,curve.count))
      return {S::InvalidInput,"Invalid material curve range",p};
    Material checked;
    if(tl::material::law44::solid::Prepare(row.material.material,curve,checked)!=
        tl::material::law44::solid::Status::Ok||!SameMaterial(checked,row.material))
      return {S::InvalidInput,"Prepared beam material or curve differs from preparation",p};
    const auto first=materials.First(ref.input().source_material_id);
    if(first!=p) {
      if(!SameMaterial(row.material,input.parents[first].material))
        return {S::IdentityMismatch,"Repeated beam MID changed material or curve bits",p};
      continue;
    }
    if(++plan.material_count>limits.max_materials||curve.count>limits.max_curve_points-plan.curve_points)
      return {S::ResourceLimit,"Beam material pool exceeds cap",p};
    plan.curve_points+=curve.count;
  }
  return {};
}
ModelReport Budget(const NodalNodeDomain& domain,ModelInput input,ModelLimits limits,
    std::size_t implementation_bytes,Plan& plan) noexcept {
  util::BoundedArenaLayout retained(limits.max_host_bytes),peak(limits.max_host_bytes);
  util::ArenaRegion ignored;
  if(!plan.arena.Append<Parent>(input.parents.size(),plan.parents)||
      !plan.arena.Append<MaterialRecord>(plan.material_count,plan.materials)||
      !plan.arena.Append<double>(2*plan.curve_points,plan.curves)||
      !retained.Append<unsigned char>(sizeof(Model)+implementation_bytes+64,ignored)||
      !retained.Append<unsigned char>(plan.arena.bytes(),ignored)||
      !retained.Append<unsigned char>(domain.owned_payload_bytes()-sizeof(NodalNodeDomain),ignored)||
      !peak.Append<unsigned char>(retained.bytes(),ignored)||
      !peak.Append<unsigned char>(2*Index::Bytes(input.parents.size())+sizeof(Plan),ignored))
    return {S::ResourceLimit,"Complete beam model plus startup scratch exceeds cap"};
  plan.retained=retained.bytes(); plan.startup=peak.bytes(); return {};
}
} // namespace tl::fea::beam18::model_detail
