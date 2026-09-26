// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../RadiossType25SearchStartup.h"
#include "../../RadiossType25TiedRemoval.h"
#include <algorithm>
#include <new>
namespace tlfea::contact::radioss_type25::initial_source::detail {
namespace {
Status Convert(search_startup::Status value) noexcept {
  if(value==search_startup::Status::Ok)return Status::Ok;
  if(value==search_startup::Status::ResourceLimit)return Status::ResourceLimit;
  if(value==search_startup::Status::UnsupportedProfile)return Status::UnsupportedProfile;
  if(value==search_startup::Status::UnsupportedArithmetic)return Status::UnsupportedArithmetic;
  if(value==search_startup::Status::NonfiniteResult)return Status::NonfiniteResult;
  return Status::InvalidInput;
}
}
Report PrepareHost(const Input& in,Limits limits,Prepared& output) noexcept {
 try {
  Prepared p;const auto n=in.contact.node_count,g=in.contact.main_count,s=in.contact.secondary_count;
  p.nodes.assign(in.contact.nodes,in.contact.nodes+n);p.mains.assign(in.contact.mains,in.contact.mains+g);
  p.secondary.assign(in.contact.secondary,in.contact.secondary+s);
  p.references.assign(in.contact.normals,in.contact.normals+in.contact.normal_count);
  p.main_gap.assign(in.main_search_gap,in.main_search_gap+g);p.main_nodes.assign(in.main_nodes,in.main_nodes+in.main_node_count);
  std::vector<std::uint8_t> main_membership(n,0);
  for(auto node:p.main_nodes){if(node>=n||main_membership[node])return {Status::InvalidInput};main_membership[node]=1;}
  for(const auto& main:p.mains)for(auto node:main.nodes){if(!main_membership[node])return {Status::InvalidInput};main_membership[node]=2;}
  for(auto node:p.main_nodes)if(main_membership[node]!=2)return {Status::InvalidInput};
  if(in.solid_count)p.solids.assign(in.solids,in.solids+in.solid_count);
  p.positions.resize(n);p.node_ids.resize(n);p.codes.resize(n);p.secondary_nodes.resize(s);
  for(std::size_t i=0;i<n;++i){const auto x=in.mesh.positions.at(std::uint32_t(i));p.positions[i]={x.x,x.y,x.z};p.node_ids[i]=p.nodes[i].source_id;p.codes[i]=p.nodes[i].constraint;}
  auto ids=p.node_ids;std::sort(ids.begin(),ids.end());
  if(std::adjacent_find(ids.begin(),ids.end())!=ids.end())return {Status::InvalidInput};
  std::vector<std::uint64_t> interface_ids;
  for(std::size_t i=0;i<in.interface_count;++i)interface_ids.push_back(in.interfaces[i].source_id);
  std::sort(interface_ids.begin(),interface_ids.end());
  if(std::adjacent_find(interface_ids.begin(),interface_ids.end())!=interface_ids.end())return {Status::InvalidInput};
  std::vector<search_startup::Secondary> secondary(s);
  for(std::size_t i=0;i<s;++i){const auto& a=p.secondary[i];secondary[i]={a.node,a.coefficient,a.gap};p.secondary_nodes[i]=a.node;}
  search_startup::Input source;source.mesh=in.mesh;source.topology=in.starter;
  source.contributors=in.contributors;source.auxiliary_rigid_primary_ids=in.auxiliary_rigid_primary_ids;
  source.native_population=in.native_population;
  source.auxiliary_rigid_primary_count=in.auxiliary_rigid_primary_count;
  source.covered_type25_siblings=in.contributors.other_interfaces;
  source.global_gap_phase=search_startup::GlobalGapPhase::ExplicitPreNodalUpdate;source.source_global_gap=in.global_search_gap;
  source.secondary=secondary.data();source.secondary_count=s;source.main_gaps=p.main_gap.data();source.main_count=g;
  source.profile.level=in.controls.level;source.profile.gap_mode=in.controls.gap_mode;
  source.profile.neighbor_removal=in.controls.neighbor_removal;source.profile.initial_penetration=in.controls.initial_penetration;
  source.profile.edge_mode=in.controls.edge_mode;source.profile.thermal_mode=in.controls.thermal;
  source.profile.curvature=in.controls.curvature;source.profile.partitions=in.controls.partitions;
  source.profile.gap_load_cards=search_startup::LoadCards::Absent;
  source.profile.initialization=search_startup::Initialization::SerialNative;
  const auto plan=search_startup::PreflightComposed(source,limits.geometric);
  if(plan.status!=search_startup::Status::Ok)return {Convert(plan.status)};
  tl::util::HostArena geometric_output,geometric_scratch;
  if(!geometric_output.Initialize(plan.output_bytes)||!geometric_scratch.Initialize(plan.scratch_bytes))return {Status::ResourceLimit};
  search_startup::GeometricSnapshot geometric;search_startup::Snapshot search;
  search_startup::Report report;
  if(in.tied_interface_count) {
    report=search_startup::BuildComposedGeometricBeforeTied(source,limits.geometric,geometric_output,geometric_scratch,&geometric);
    search=geometric.geometry;
  } else report=search_startup::BuildComposedNoTied(source,limits.geometric,geometric_output,geometric_scratch,&search);
  if(report.status!=search_startup::Status::Ok)return {Convert(report.status),report.node,report.main};
  p.native_nodes=search.native_model_nodes;p.population=search.native_population;p.native_nodes_exact=search.native_model_nodes_exact;
  p.diagnostics.mean_length=search.mean_length;
  p.diagnostics.engine_margin=search.margin;p.diagnostics.initial_margin=in.controls.base_multiplier*search.mean_length;
  p.removal_offsets.resize(g+1);
  for(std::size_t i=0;i<=g;++i)p.removal_offsets[i]=search.main_offsets[i];
  if(search.removal_count)p.removal_nodes.assign(search.removed_nodes,search.removed_nodes+search.removal_count);
  // Runtime/initial GPU membership needs sorted nodes, while the source CSR's
  // original order remains in its immutable geometric/tied construction.
  for(std::size_t i=0;i<g;++i)std::sort(p.removal_nodes.begin()+p.removal_offsets[i],p.removal_nodes.begin()+p.removal_offsets[i+1]);
  tl::util::HostArena tied_output,tied_scratch;
  tied_removal::Snapshot final;
  if(in.tied_interface_count) {
    std::vector<tied_removal::History> provisional(s); // CSR value packet only, never a production history seed.
    tied_removal::Input tied;tied.source=source;tied.geometric=geometric;tied.finalization=in.tied_phase;
    tied.tied_removal=in.controls.tied_removal;tied.interfaces=in.tied_interfaces;tied.interface_count=in.tied_interface_count;
    tied.history=provisional.data();tied.history_count=s;
    // ININTR allocates16G. get_list_remnode:322-324 calls UPGRADE_REMNODE
    // with exact used KREMNODE(G+1) only when larger; that routine assigns
    // IPARI62/S_REMNODE exactly. This max is the proved caller sequence,
    // not a guess that allocated extent always equals the used prefix.
    tied.native_removal_extent=std::max(std::size_t{16}*g,search.removal_count);
    const auto tp=tied_removal::PreflightComposed(tied,limits.tied);
    if(tp.status!=search_startup::Status::Ok)return {Convert(tp.status)};
    if(!tied_output.Initialize(tp.output_bytes)||!tied_scratch.Initialize(tp.scratch_bytes))return {Status::ResourceLimit};
    const auto tr=tied_removal::BuildComposed(tied,limits.tied,tied_output,tied_scratch,&final);
    if(tr.status!=search_startup::Status::Ok)return {Convert(tr.status),SIZE_MAX,tr.main,tr.row};
    p.added_removals=final.added_removals;search=final.search;
  }
  p.final_offsets.assign(search.secondary_offsets,search.secondary_offsets+s+1);
  if(search.removal_count)p.final_mains.assign(search.removed_mains,search.removed_mains+search.removal_count);
  p.final_main_offsets.assign(search.main_offsets,search.main_offsets+g+1);
  if(search.removal_count)p.final_nodes.assign(search.removed_nodes,search.removed_nodes+search.removal_count);
  const auto& top=in.starter;
  p.topology.assign(top.mains,top.mains+g);
  p.normal_offsets.assign(top.normal_offsets,top.normal_offsets+top.starter.reference_count+1);
  if(top.normal_incidence_count)p.normal_mains.assign(top.normal_mains,top.normal_mains+top.normal_incidence_count);
  p.expanded_to_primary.assign(top.expanded_to_primary,top.expanded_to_primary+g);
  p.primary_to_partner.assign(top.primary_to_partner,top.primary_to_partner+top.primary_count);
  if(top.primary_role_count)p.primary_roles.assign(top.primary_roles,top.primary_roles+top.primary_role_count);
  if(top.primary_identity_count)p.primary_identities.assign(top.primary_identities,top.primary_identities+top.primary_identity_count);
  if(top.raw_origin_count) {
    p.raw_origins.assign(top.raw_origins,top.raw_origins+top.raw_origin_count);
    p.raw_origin_to_primary.assign(top.raw_origin_to_primary,top.raw_origin_to_primary+top.raw_origin_count);
  }
  if(top.post_gapm) {
    const auto& post=*top.post_gapm;p.post_gapm=post;
    p.primary_corners.assign(post.primary_corners,post.primary_corners+post.primary_count);
    p.before_shell.assign(post.before_shell,post.before_shell+post.before_shell_count);
    p.final_support.assign(post.final_support,post.final_support+post.main_count);
  }
  p.starter=top;
  p.solid_offsets.assign(n+1,0);p.solid_incidence.resize(8*p.solids.size());
  std::vector<std::pair<std::uint64_t,std::uint32_t>> solid_ids;
  for(std::size_t i=0;i<p.solids.size();++i) {
    solid_ids.push_back({p.solids[i].native_source_id,std::uint32_t(i)});
    for(auto node:p.solids[i].nodes)++p.solid_offsets[node+1];
  }
  std::sort(solid_ids.begin(),solid_ids.end());
  for(std::size_t i=1;i<solid_ids.size();++i)if(solid_ids[i-1].first==solid_ids[i].first)return {Status::InvalidInput};
  for(std::size_t i=0;i<n;++i)p.solid_offsets[i+1]+=p.solid_offsets[i];
  auto cursor=p.solid_offsets;
  // Original BUILD_CNEL traverses raw corner first, then complete solid rows.
  for(unsigned k=0;k<8;++k)for(std::size_t i=0;i<p.solids.size();++i)
    p.solid_incidence[cursor[p.solids[i].nodes[k]]++]=std::uint32_t(i);
  p.support_solid.assign(g,UINT32_MAX);p.internal_main.assign(g,0);
  if(in.starter.post_gapm)for(std::size_t m=0;m<g;++m) {
    const auto& support=in.starter.post_gapm->final_support[m];
    p.internal_main[m]=support.second_solid_source_id!=0;
    const auto& first=support.first;
    if(first.kind!=startup::PhysicalSupportKind::EightSlotSolid)continue;
    const auto it=std::lower_bound(solid_ids.begin(),solid_ids.end(),std::make_pair(first.source_element_id,std::uint32_t{0}));
    if(it==solid_ids.end()||it->first!=first.source_element_id)return {Status::InvalidInput,SIZE_MAX,m};
    p.support_solid[m]=it->second;
  }
  p.sweep_mains.resize(g);p.main_ranks.resize(g);
  for(std::size_t m=0;m<g;++m) {
    auto& entry=p.sweep_mains[m];entry.rank=std::uint32_t(m);p.main_ranks[m]=std::uint32_t(m);
    entry.source.source_id=std::uint64_t(p.mains[m].global_id);entry.source.segment_type=p.mains[m].segment_type;
    for(unsigned k=0;k<4;++k)entry.source.nodes[k]=p.mains[m].nodes[k];
  }
  output=std::move(p);BindPrepared(output,in);return {Status::Ok};
 } catch(const std::bad_alloc&) {return {Status::ResourceLimit};}
 catch(...) {return {Status::InvalidInput};}
}
void BindPrepared(Prepared& p,const Input& in) noexcept {
  // Scalars only from the caller descriptor. Arrays below bind owned copies;
  // tables used solely by the already-completed source admission are absent.
  Input d;d.phase=in.phase;d.stamp=in.stamp;d.units=in.units;d.controls=in.controls;
  d.global_search_gap=in.global_search_gap;d.native_interface_id=in.native_interface_id;
  d.solid_scope=in.solid_scope;d.contributors=in.contributors;d.tied_phase=in.tied_phase;
  d.native_population=in.native_population;
  d.mesh.profile=in.mesh.profile;d.mesh.topology=in.mesh.topology;
  d.mesh.source_generation=in.mesh.source_generation;d.mesh.coordinates=startup::Coordinates::Native;
  d.mesh.units=in.units;d.mesh.node_count=p.nodes.size();d.mesh.primary_count=p.starter.primary_count;
  static_assert(sizeof(Vector)==3*sizeof(double),"Packed native coordinates require three doubles");
  d.mesh.positions={reinterpret_cast<const double*>(p.positions.data()),std::uint32_t(p.positions.size()),3,1};
  d.mesh.node_source_ids=p.node_ids.data();
  auto& top=p.starter;top.mains=p.topology.data();top.expanded_to_primary=p.expanded_to_primary.data();
  top.primary_to_partner=p.primary_to_partner.data();top.normal_offsets=p.normal_offsets.data();
  top.normal_mains=p.normal_mains.empty()?nullptr:p.normal_mains.data();
  top.primary_roles=p.primary_roles.empty()?nullptr:p.primary_roles.data();
  top.primary_identities=p.primary_identities.empty()?nullptr:p.primary_identities.data();
  top.raw_origins=p.raw_origins.empty()?nullptr:p.raw_origins.data();
  top.raw_origin_to_primary=p.raw_origin_to_primary.empty()?nullptr:p.raw_origin_to_primary.data();
  // Normal face slots are already owned inside each lifecycle Main. They are
  // not exposed as a falsely contiguous flat StoredNormal view.
  top.starter.face_normals=nullptr;top.starter.references=p.references.data();
  if(in.starter.post_gapm) {
    p.post_gapm.primary_corners=p.primary_corners.data();p.post_gapm.before_shell=p.before_shell.data();
    p.post_gapm.final_support=p.final_support.data();top.post_gapm=&p.post_gapm;
  } else top.post_gapm=nullptr;
  d.starter=top;auto& c=d.contact;c.nodes=p.nodes.data();c.node_count=p.nodes.size();
  c.mains=p.mains.data();c.main_count=p.mains.size();c.secondary=p.secondary.data();c.secondary_count=p.secondary.size();
  c.normals=p.references.data();c.normal_count=p.references.size();c.generation=in.contact.generation;
  c.normal_to_main={p.normal_offsets.data(),p.normal_offsets.size(),p.normal_mains.empty()?nullptr:p.normal_mains.data(),p.normal_mains.size()};
  c.removed_main_by_secondary={p.final_offsets.data(),p.final_offsets.size(),p.final_mains.empty()?nullptr:p.final_mains.data(),p.final_mains.size()};
  d.main_nodes=p.main_nodes.data();d.main_node_count=p.main_nodes.size();
  d.main_search_gap=p.main_gap.data();d.main_search_gap_count=p.main_gap.size();
  d.solids=p.solids.empty()?nullptr:p.solids.data();d.solid_count=p.solids.size();p.descriptor=d;
}
} // namespace tlfea::contact::radioss_type25::initial_source::detail
