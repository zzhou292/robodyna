// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../RadiossType25SearchStartup.h"
#include "../search/Ranges.h"
#include "../search_startup/Internal.h"
#include "../current_normals/Types.h"
#include "../../self_contact_filters/Environment.h"
#include "lib_src/math/ScalarBits.h"
#include <algorithm>
#include <climits>
namespace tlfea::contact::radioss_type25::initial_source::detail {
Report Admit(const Input& in,Limits limits,Forecast& forecast) noexcept {
  namespace r=search::detail;
  const auto& c=in.controls;const auto& top=in.starter;const auto& src=in.contact;
  units_detail::Factors unit_factors;
  if(!units_detail::Make(in.units,unit_factors))return {Status::InvalidInput};
  if(in.phase!=Phase::StarterNormalsAndPreBucGaps||
      in.interface_phase!=InterfaceCensusPhase::CompleteOriginalAndDeclaredAdditions)return {Status::WrongPhase};
  if(c.level!=1||c.gap_mode!=1||c.initial_penetration!=5||c.damping_flag!=1||(c.sharp!=1&&c.sharp!=2)||
      c.arithmetic_precision!=0||c.partitions!=1||c.starter_workers!=1||c.edge_mode!=0||c.thermal!=0||
      c.native_voxel_capacity!=8000000||c.neighbor_removal!=2||c.tied_removal!=1||c.thickness_update!=0||c.stiffness_formulation!=4||
      c.stiffness_mass_update!=0||c.curvature!=0||c.native_packet_size!=128||c.gap_load_cards||c.drad!=0||c.gap_load!=0||
      !tl::math::SameScalarBits(c.base_multiplier,static_cast<double>(.20f)))return {Status::UnsupportedProfile};
  if(!self_contact_filters::CompatibleHostArithmetic())return {Status::UnsupportedArithmetic};
  if(!std::isfinite(in.global_search_gap)||in.global_search_gap<0)return {Status::InvalidInput};
  const Limits hard;
  const auto n=src.node_count,g=src.main_count,s=src.secondary_count,p=top.primary_count,nr=src.normal_count;
  if(!n||!g||!s||!p||!in.stamp.source||!in.stamp.topology||!in.stamp.physical_domain||!in.native_interface_id)
    return {Status::InvalidInput};
  if(n>limits.nodes||n>hard.nodes||s>limits.secondaries||s>hard.secondaries||g>limits.mains||g>hard.mains||
      in.main_node_count>n||!in.main_node_count||in.solid_count>limits.solids||in.solid_count>hard.solids||p>g||nr>4*g||
      !limits.max_tasks||limits.max_tasks>hard.max_tasks||!limits.max_pairs||limits.max_pairs>hard.max_pairs||
      !limits.max_device_bytes||limits.max_device_bytes>hard.max_device_bytes||
      !limits.max_host_bytes||limits.max_host_bytes>hard.max_host_bytes||
      in.interface_count>65536||in.tied_interface_count>limits.tied.max_interfaces||
      top.normal_incidence_count>4*g||top.primary_role_count>p||top.primary_identity_count>p||
      top.raw_origin_count>startup::Limits{}.max_raw_origins)return {Status::ResourceLimit};
  if(in.mesh.coordinates!=startup::Coordinates::Native||in.mesh.node_count!=n||top.node_count!=n||top.main_count!=g||
      in.mesh.primary_count!=p||top.source_generation!=in.stamp.topology||in.mesh.source_generation!=in.stamp.topology||
      src.generation!=in.stamp.topology||in.main_search_gap_count!=g||top.starter.reference_count!=nr)
    return {Status::InvalidInput};
  std::size_t position_bytes=0;
  if(!r::Span(src.nodes,n)||!r::Span(src.mains,g)||!r::Span(src.secondary,s)||!r::Span(src.normals,nr)||
      !r::Span(in.mesh.node_source_ids,n)||!r::VectorSpan(in.mesh.positions,n,position_bytes)||
      !r::Span(top.mains,g)||!r::Span(top.starter.face_normals,4*g)||!r::Span(top.starter.references,nr)||
      !r::Span(top.normal_offsets,nr+1)||!r::Span(top.normal_mains,top.normal_incidence_count)||
      !r::Span(top.expanded_to_primary,g)||!r::Span(top.primary_to_partner,p)||
      !r::Span(top.primary_roles,top.primary_role_count)||!r::Span(top.primary_identities,top.primary_identity_count)||
      !r::Span(top.raw_origins,top.raw_origin_count)||!r::Span(top.raw_origin_to_primary,top.raw_origin_count)||
      !r::Span(in.main_nodes,in.main_node_count)||!r::Span(in.main_search_gap,g)||!r::Span(in.solids,in.solid_count)||!r::Span(in.interfaces,in.interface_count)||
      !r::Span(in.tied_interfaces,in.tied_interface_count)||
      !r::Span(in.auxiliary_rigid_primary_ids,in.auxiliary_rigid_primary_count))return {Status::InvalidInput};
  if(top.post_gapm) {
    if(!r::Span(top.post_gapm,1))return {Status::InvalidInput};
    const auto& post=*top.post_gapm;
    if(post.primary_count!=p||post.before_shell_count!=p||post.main_count!=g||
        !r::Span(post.primary_corners,p)||!r::Span(post.before_shell,p)||!r::Span(post.final_support,g))return {Status::InvalidInput};
  }
  if(src.normal_to_main.offset_count!=nr+1||src.normal_to_main.entry_count!=top.normal_incidence_count||
      !r::Span(src.normal_to_main.offsets,nr+1)||!r::Span(src.normal_to_main.entries,top.normal_incidence_count)||
      src.removed_main_by_secondary.offsets||src.removed_main_by_secondary.offset_count||
      src.removed_main_by_secondary.entries||src.removed_main_by_secondary.entry_count)return {Status::WrongPhase};
  for(std::size_t i=0;i<=nr;++i)if(src.normal_to_main.offsets[i]!=top.normal_offsets[i])return {Status::InvalidInput};
  for(std::size_t i=0;i<top.normal_incidence_count;++i)
    if(src.normal_to_main.entries[i]!=top.normal_mains[i])return {Status::InvalidInput};
  if((in.solid_scope==SolidScope::ExplicitNoSolids&&in.solid_count)||
      (in.solid_scope==SolidScope::CompleteEightSlotModel&&!in.solid_count)||
      (in.solid_scope!=SolidScope::ExplicitNoSolids&&in.solid_scope!=SolidScope::CompleteEightSlotModel))return {Status::UnsupportedProfile};
  const bool interval=in.native_population.policy==search_startup::NativePopulationPolicy::CompleteModelMultiplierTier;
  if(in.contributors.census!=search_startup::Census::CompleteDeclaredModel||in.contributors.physical_nodes!=n||
      (interval?(in.contributors.native_auxiliary_nodes!=SIZE_MAX||in.auxiliary_rigid_primary_count||in.auxiliary_rigid_primary_ids):
        (in.contributors.native_auxiliary_nodes!=in.auxiliary_rigid_primary_count||in.contributors.native_auxiliary_nodes!=in.contributors.rigid_bodies))||
      in.contributors.tied_interfaces!=in.tied_interface_count||in.contributors.unsupported_elements)
    return {Status::UnsupportedProfile};
  if(interval) {
    double low=0,high=0;
    if(in.native_population.lower<n||in.native_population.upper<in.native_population.lower||
        search_startup::ResolveMultiplier(in.native_population.lower,&low)!=search_startup::Status::Ok||
        search_startup::ResolveMultiplier(in.native_population.upper,&high)!=search_startup::Status::Ok||
        !tl::math::SameScalarBits(low,high))return {Status::UnsupportedProfile};
  } else if(in.native_population.policy!=search_startup::NativePopulationPolicy::ExactDeclaredAuxiliaryIds||
      in.native_population.lower||in.native_population.upper)return {Status::UnsupportedProfile};
  std::size_t type2=0,type25=0,target=0;
  for(std::size_t i=0;i<in.interface_count;++i) {
    const auto& f=in.interfaces[i];
    if(!f.source_id||f.source_id>INT_MAX||!f.native_storage_ordinal||
        (i&&f.native_storage_ordinal<=in.interfaces[i-1].native_storage_ordinal)||
        (f.origin!=InterfaceOrigin::OriginalDefinition&&f.origin!=InterfaceOrigin::DeclaredAdditionalInterface))return {Status::InvalidInput};
    if(f.kind==InterfaceKind::Type2)++type2;
    else if(f.kind==InterfaceKind::Type25)++type25;
    else return {Status::UnsupportedProfile};
    if(f.source_id==in.native_interface_id){if(f.kind!=InterfaceKind::Type25)return {Status::InvalidInput};++target;}
  }
  if(target!=1||!type25||type2!=in.tied_interface_count||in.contributors.other_interfaces!=type25-1)
    return {Status::InvalidInput};
  if(type2&&in.tied_phase!=tied_removal::Finalization::CompactedAfterKinChk)return {Status::WrongPhase};
  if(!type2&&(in.contributors.cin_links||in.tied_phase!=tied_removal::Finalization::Unspecified))return {Status::WrongPhase};
  std::size_t found=0;
  for(const auto* f=in.interfaces;f!=in.interfaces+in.interface_count;++f)if(f->kind==InterfaceKind::Type2) {
    const auto& tie=in.tied_interfaces[found++];
    if(tie.source_id!=f->source_id||tie.native_ordinal!=f->native_storage_ordinal||tie.level!=28)return {Status::InvalidInput};
  }
  for(std::size_t i=0;i<n;++i) {
    const auto x=in.mesh.positions.at(std::uint32_t(i));
    if(!src.nodes[i].source_id||src.nodes[i].source_id>INT_MAX||src.nodes[i].source_id!=in.mesh.node_source_ids[i]||
        src.nodes[i].constraint<0||src.nodes[i].constraint>7||src.nodes[i].skew<0||
        !tl::math::Finite(x.x)||!tl::math::Finite(x.y)||!tl::math::Finite(x.z))
      return {Status::InvalidInput,i};
  }
  for(std::size_t i=0;i<s;++i) {
    const auto& a=src.secondary[i];
    if(a.node>=n||!std::isfinite(a.coefficient)||a.coefficient<0||!std::isfinite(a.gap)||a.gap<0||a.initial_contact_flag)
      return {Status::InvalidInput,SIZE_MAX,SIZE_MAX,i};
    if(i&&src.nodes[src.secondary[i-1].node].source_id>=src.nodes[a.node].source_id)return {Status::InvalidInput,SIZE_MAX,SIZE_MAX,i};
  }
  for(std::size_t i=0;i<g;++i) {
    const auto& a=src.mains[i];const auto& b=top.mains[i];
    if(a.global_id!=b.global_id||a.segment_type!=b.segment_type||!std::isfinite(a.coefficient)||
        !tl::math::SameScalarBits(a.maximum_gap,in.main_search_gap[i])||
        !std::isfinite(in.main_search_gap[i])||in.main_search_gap[i]<0)return {Status::InvalidInput,SIZE_MAX,i};
    for(unsigned k=0;k<4;++k) {
      const auto& normal=top.starter.face_normals[4*i+k];const auto& actual=a.normal_slot[k];
      if(a.nodes[k]!=b.nodes[k]||a.nodes[k]>=n||a.neighbors[k]!=b.neighbors[k]||a.normal_reference[k]!=b.normal_reference[k]||
          !std::isfinite(a.gap[k])||a.gap[k]<0||!std::isfinite(actual.x)||!std::isfinite(actual.y)||!std::isfinite(actual.z)||
          !tl::math::SameScalarBits(normal.x,actual.x)||!tl::math::SameScalarBits(normal.y,actual.y)||
          !tl::math::SameScalarBits(normal.z,actual.z))return {Status::InvalidInput,SIZE_MAX,i};
    }
  }
  for(std::size_t i=0;i<nr;++i) {
    const auto& a=src.normals[i];const auto& b=top.starter.references[i];
    if(a.boundary!=b.boundary||a.boundary<0||a.boundary>1)return {Status::InvalidInput};
    for(unsigned k=0;k<2;++k) {
      const auto x=a.bisector[k],y=b.bisector[k];
      if(!std::isfinite(x.x)||!std::isfinite(x.y)||!std::isfinite(x.z)||
          !tl::math::SameScalarBits(x.x,y.x)||!tl::math::SameScalarBits(x.y,y.y)||!tl::math::SameScalarBits(x.z,y.z))return {Status::InvalidInput};
    }
  }
  for(std::size_t i=0;i<in.solid_count;++i) {
    const auto& solid=in.solids[i];
    if(!solid.native_source_id||solid.native_source_id>INT_MAX||!solid.part_source_id||solid.part_source_id>INT_MAX)return {Status::InvalidInput};
    for(auto node:solid.nodes)if(node>=n)return {Status::InvalidInput,node};
  }
  forecast.status=Status::Ok;return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::initial_source::detail
