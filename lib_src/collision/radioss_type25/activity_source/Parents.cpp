// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::activity_source::detail {
TransactionReport Parents(const tl::fea::ShellPhysicalBinding& physical,Counts& counts,
    void* context,Visit visit) noexcept {
  if(!physical.prepared()||!physical.domain()||!physical.shells()||!physical.coefficients())
    return Fail(S::SourceMismatch,"Complete physical source is required");
  const auto& ledger=*physical.coefficients();const auto& shells=*physical.shells();
  const auto* map=ledger.shells();
  if(!map||!map->Matches(shells,*physical.domain())||!ledger.domain()->Matches(*physical.domain()))
    return Fail(S::SourceMismatch,"Physical source uses a different node domain");
  counts={};counts.nodes=physical.domain()->node_count();
  auto add=[&](Family family,std::size_t index,std::uint64_t id,const std::size_t* nodes,unsigned n) {
    if(!id||index>UINT32_MAX||!n||n>8)return false;
    ParentRow row;row.identity={family,static_cast<std::uint32_t>(index),id};
    for(unsigned k=0;k<n;++k) {
      if(nodes[k]>=counts.nodes||nodes[k]>UINT32_MAX)return false;
      const auto node=static_cast<std::uint32_t>(nodes[k]);
      if(std::find(row.nodes,row.nodes+row.count,node)==row.nodes+row.count)row.nodes[row.count++]=node;
    }
    if(counts.parents==UINT32_MAX||counts.incidence>UINT32_MAX-row.count)return false;
    ++counts.parents;++counts.families[static_cast<std::size_t>(family)];counts.incidence+=row.count;
    return !visit||visit(context,row);
  };
  auto shell=[&](Family family,std::size_t count,auto id,auto nodes,unsigned n) {
    for(std::size_t i=0;i<count;++i) {
      const auto local=nodes(i);std::size_t domain[4];
      for(unsigned k=0;k<n;++k)domain[k]=map->owner_index(local[k]);
      if(!add(family,i,id(i),domain,n))return false;
    }return true;
  };
  if(!shell(Family::Qeph,shells.qeph_count(),[&](auto i){return shells.qeph_source_id(i);},
        [&](auto i){return shells.qeph_nodes(i);},4)||
     !shell(Family::T3,shells.t3_count(),[&](auto i){return shells.t3_source_id(i);},
        [&](auto i){return shells.t3_nodes(i);},3)||
     !shell(Family::Qbat,shells.qbat_count(),[&](auto i){return shells.qbat_source_id(i);},
        [&](auto i){return shells.qbat_nodes(i);},4))
    return Fail(S::SourceMismatch,"Invalid complete shell support incidence");
  if(const auto* solids=ledger.solids()) {
    if(!solids->domain()||!solids->domain()->Matches(*physical.domain()))
      return Fail(S::SourceMismatch,"Solid support domain differs");
    std::array<std::size_t,5> index{};
    for(const auto& parent:solids->parents()) {
      Family family;std::size_t slot;
      switch(parent.family) {
        case tl::fea::SolidCoefficientFamily::Solid18:family=Family::Solid18;slot=0;break;
        case tl::fea::SolidCoefficientFamily::Solid24:family=Family::Solid24;slot=1;break;
        case tl::fea::SolidCoefficientFamily::Solid6z:family=Family::Solid6z;slot=2;break;
        case tl::fea::SolidCoefficientFamily::Solid18Law44:family=Family::Solid18Law44;slot=3;break;
        case tl::fea::SolidCoefficientFamily::Solid18Law90:family=Family::Solid18Law90;slot=4;break;
        default:return Fail(S::UnsupportedProfile,"Unknown physical solid support family");
      }
      if(parent.node_count!=(slot==2?6u:8u)||!add(family,index[slot]++,parent.source_element_id,parent.domain_node,parent.node_count))
        return Fail(S::SourceMismatch,"Invalid solid support incidence");
    }
  }
  if(const auto* beam=ledger.beam18()) {
    if(!beam->model()||!beam->domain()->Matches(*physical.domain()))return Fail(S::SourceMismatch,"Beam support domain differs");
    for(std::size_t i=0;i<beam->model()->parents().size();++i) {
      tl::fea::beam18::EndpointContribution endpoint[2];
      if(!beam->model()->Endpoint(i,0,endpoint[0])||!beam->model()->Endpoint(i,1,endpoint[1]))
        return Fail(S::SourceMismatch,"Beam endpoint support is unavailable",i);
      const std::size_t nodes[]{endpoint[0].global_node,endpoint[1].global_node};
      if(!add(Family::Beam18,i,endpoint[0].source_element_id,nodes,2))return Fail(S::SourceMismatch,"Invalid beam support",i);
    }
  }
  if(const auto* model=ledger.type25())for(std::size_t i=0;i<model->connection_count();++i) {
    const auto& row=model->connections()[i];
    if(!add(Family::Type25,i,row.source_element_id,row.global_node,2))return Fail(S::SourceMismatch,"Invalid TYPE25 spring support",i);
  }
  if(const auto* spring=ledger.type13()) {
    if(!spring->model()||!spring->domain()->Matches(*physical.domain()))return Fail(S::SourceMismatch,"TYPE13 support domain differs");
    for(std::size_t i=0;i<spring->model()->connection_count();++i) {
      tl::fea::type13::EndpointContribution endpoint[2];
      if(!spring->model()->Endpoint(i,0,endpoint[0])||!spring->model()->Endpoint(i,1,endpoint[1]))
        return Fail(S::SourceMismatch,"TYPE13 endpoint support is unavailable",i);
      const std::size_t nodes[]{endpoint[0].global_node,endpoint[1].global_node};
      if(!add(Family::Type13,i,endpoint[0].source_element_id,nodes,2))return Fail(S::SourceMismatch,"Invalid TYPE13 support",i);
    }
  }
  // Native TAGOFF support excludes bare element/nodal mass and constraint-only
  // membership. The named beam/spring models exclude orientation node N3;
  // native property12 third-node support is outside these source categories.
  return Ok();
}
}
