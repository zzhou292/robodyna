// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
namespace glass_removal_test {
void ContactSource::PrepareInitial(const nodal_empty_test::Fixture& physical,const n::TransactionConfig& config) {
  namespace is=n::initial_source;
  // The complete two-interface census is the declared coupon model. It does
  // not impersonate the original vehicle interface/source identities.
  const is::InterfaceIdentity interfaces[]{
    {910200,1,is::InterfaceKind::Type25,is::InterfaceOrigin::DeclaredAdditionalInterface},
    {910201,2,is::InterfaceKind::Type25,is::InterfaceOrigin::DeclaredAdditionalInterface}};
  auto input_mains=mains;
  for(std::size_t i=0;i<input_mains.size();++i)for(unsigned k=0;k<4;++k)
    input_mains[i].normal_slot[k]=starter.starter.face_normals[4*i+k];
  std::vector<std::uint32_t> main_nodes;std::vector<bool> seen(ids.size());
  for(const auto& face:primary)for(auto node:face.nodes)if(!seen[node]){seen[node]=true;main_nodes.push_back(node);}
  std::vector<double> gaps(mains.size(),.001);
  is::Input in;in.phase=is::Phase::StarterNormalsAndPreBucGaps;
  in.stamp={id,starter.source_generation,physical.domain.source_instance_id(),3};
  in.units=config.units;in.engine_handoff=is::EngineHandoff::SourceProvedFreshSerialSearchAtZero;
  in.controls={1,1,5,1,1,0,1,1,0,0,2,1,0,4,0,false,double(.20f),0,0,8000000,0,128};
  in.interface_phase=is::InterfaceCensusPhase::CompleteOriginalAndDeclaredAdditions;
  in.interfaces=interfaces;in.interface_count=2;in.native_interface_id=id;
  auto& mesh=in.mesh;mesh.profile=starter.profile;mesh.topology=starter.topology;
  mesh.coordinates=s::Coordinates::Native;mesh.units=config.units;mesh.source_generation=starter.source_generation;
  mesh.node_source_ids=ids.data();mesh.node_count=ids.size();
  mesh.positions={physical.positions.data(),std::uint32_t(ids.size()),3,1};
  mesh.primary=primary.data();mesh.primary_count=primary.size();
  in.starter=starter;in.contact=Common().selection;in.contact.mains=input_mains.data();
  in.contact.normals=starter.starter.references;in.contact.removed_main_by_secondary={};
  in.main_nodes=main_nodes.data();in.main_node_count=main_nodes.size();
  in.main_search_gap=gaps.data();in.main_search_gap_count=gaps.size();in.global_search_gap=.002;
  in.solid_scope=is::SolidScope::ExplicitNoSolids;
  in.contributors={n::search_startup::Census::CompleteDeclaredModel,ids.size(),3};
  in.contributors.other_interfaces=1;
  is::Limits limits;limits.max_tasks=256;limits.max_pairs=256;
  limits.max_host_bytes=16u<<20;limits.max_device_bytes=16u<<20;
  limits.geometric.max_removals=128;limits.tied.search=limits.geometric;
  const auto report=is::PrepareSource(in,limits,initial);
  if(report.status!=is::Status::Ok)throw std::runtime_error("Glass genuine initial source rejected status="+
      std::to_string(unsigned(report.status))+" node="+std::to_string(report.node)+" main="+std::to_string(report.main));
}
} // namespace glass_removal_test
