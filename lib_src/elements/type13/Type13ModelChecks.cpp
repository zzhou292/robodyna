// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type13ModelInternal.h"
#include "Type13Units.h"

namespace tl::fea::type13::model_detail {
ModelReport Preflight(const ModelInput& in,ModelLimits limit,std::size_t fixed_bytes,Layout& out) {
  const ModelLimits hard;
  if(!limit.max_connections||limit.max_connections>hard.max_connections||
     !limit.max_properties||limit.max_properties>hard.max_properties||
     !limit.max_local_nodes||limit.max_local_nodes>hard.max_local_nodes||
     !limit.max_global_nodes||limit.max_global_nodes>hard.max_global_nodes||
     !limit.max_host_bytes||limit.max_host_bytes>hard.max_host_bytes||
     !in.connection_count||in.connection_count>limit.max_connections||
     !in.property_count||in.property_count>limit.max_properties||
     !in.node_count||in.node_count>limit.max_local_nodes||
     !in.global_node_count||in.global_node_count>limit.max_global_nodes)
    return Error(ModelStatus::ResourceLimit,"TYPE13 model counts or hard limits exceeded");
  Layout next;
  util::BoundedArenaLayout arena(limit.max_host_bytes);
  if(!arena.Append<ModelNode>(in.node_count,next.nodes)||
     !arena.Append<OwnedProperty>(in.property_count,next.properties)||
     !arena.Append<ModelConnection>(in.connection_count,next.connections)||
     !arena.Append<Startup>(in.connection_count,next.startup))
    return Error(ModelStatus::ResourceLimit,"TYPE13 owned arena exceeds cap");
  next.arena_bytes=arena.bytes();
  util::BoundedArenaLayout budget(limit.max_host_bytes);
  util::ArenaRegion ignored;
  // Includes fixed headers and conservative shared-control reserve supplied by owner.
  if(!budget.Append<unsigned char>(fixed_bytes,ignored)||
     !budget.Append<unsigned char>(next.arena_bytes,ignored))
    return Error(ModelStatus::ResourceLimit,"TYPE13 complete owned payload exceeds cap");
  next.owned_bytes=budget.bytes();
  const auto scratch=sizeof(Scratch)+2*Index::Storage::ExtraBytes(in.node_count)+
      Index::Storage::ExtraBytes(in.property_count)+Index::Storage::ExtraBytes(in.connection_count)+
      decltype(Scratch::used_nodes)::ExtraBytes(in.node_count)+
      decltype(Scratch::used_properties)::ExtraBytes(in.property_count);
  if(!budget.Append<unsigned char>(scratch,ignored))
    return Error(ModelStatus::ResourceLimit,"TYPE13 simultaneous startup scratch exceeds cap");
  next.startup_bytes=budget.bytes();
  detail::UnitFactors factors;
  if(!in.source_instance_id||!detail::ResolveUnits(in.units,factors)||
     !Range(in.nodes,in.node_count)||!Range(in.properties,in.property_count)||
     !Range(in.connections,in.connection_count))
    return Error(ModelStatus::InvalidInput,"TYPE13 source units or borrowed ranges are invalid");
  // Fixed four-by-five curve shape/range validation precedes any curve value read.
  for(std::size_t p=0;p<in.property_count;++p) {
    const auto& declaration=in.properties[p];
    if(!declaration.source_id||!Same(declaration.input.units,in.units))
      return Error(ModelStatus::InvalidInput,"TYPE13 property identity or units differ",ModelEntry::Property,p);
    for(const auto& curve:declaration.input.curves)
      if(curve.count!=CurvePoints||!Range(curve.points,curve.count))
        return Error(ModelStatus::InvalidInput,"TYPE13 curve range or shape is invalid",ModelEntry::Property,p);
  }
  out=next;return {};
}
ModelReport CheckNodes(const ModelInput& in,Scratch& scratch) {
  scratch.node_ids.Prepare(in.node_count,[&](auto n){return in.nodes[n].source_id;});
  scratch.global_nodes.Prepare(in.node_count,[&](auto n){return in.nodes[n].global_node;});
  scratch.property_ids.Prepare(in.property_count,[&](auto p){return in.properties[p].source_id;});
  scratch.element_ids.Prepare(in.connection_count,[&](auto e){return in.connections[e].source_id;});
  scratch.used_nodes.Resize(in.node_count);
  scratch.used_properties.Resize(in.property_count);
  for(std::size_t n=0;n<in.node_count;++n) {
    const auto& node=in.nodes[n];
    if(!node.source_id||!tl::math::fixed3::Finite(node.position_native)||
       (node.global_node!=SIZE_MAX&&node.global_node>=in.global_node_count))
      return Error(ModelStatus::InvalidInput,"Invalid TYPE13 source node",ModelEntry::Node,n);
    if(scratch.node_ids.First(node.source_id)!=n||
       (node.global_node!=SIZE_MAX&&scratch.global_nodes.First(node.global_node)!=n))
      return Error(ModelStatus::DuplicateIdentity,"Repeated TYPE13 source or owner node",ModelEntry::Node,n);
  }
  return {};
}
ModelReport CheckConnection(const ModelInput& in,std::size_t e,const Scratch& scratch) {
  const auto& c=in.connections[e];
  if(!c.source_id||c.property>=in.property_count||c.node[0]==c.node[1])
    return Error(ModelStatus::InvalidInput,"Invalid TYPE13 connection identity",ModelEntry::Connection,e);
  if(scratch.element_ids.First(c.source_id)!=e)
    return Error(ModelStatus::DuplicateIdentity,"Repeated TYPE13 source element",ModelEntry::Connection,e);
  for(unsigned local=0;local<3;++local)
    if(c.node[local]>=in.node_count||
       (local<2&&in.nodes[c.node[local]].global_node==SIZE_MAX))
      return Error(ModelStatus::InvalidInput,"TYPE13 endpoint lacks a real owner node",ModelEntry::Connection,e);
  return {};
}
ReferenceInput ReferenceFor(const ModelInput& in,const ModelConnection& c) {
  ReferenceInput result;
  for(unsigned local=0;local<3;++local)result.position[local]=in.nodes[c.node[local]].position_native;
  result.skew_x=c.skew_x;result.skew_y=c.skew_y;result.coordinate_noise=c.coordinate_noise;
  for(unsigned i=0;i<4;++i)result.endpoint_release[i]=c.endpoint_release[i];
  return result;
}
} // namespace tl::fea::type13::model_detail
