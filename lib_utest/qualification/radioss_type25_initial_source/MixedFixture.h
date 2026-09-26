// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "../radioss_type25_post_gapm/Fixture.h"
namespace initial_source_test {
struct MixedFixture {
  type25_post_gapm_test::Fixture topology;
  st::NativeResult native;
  std::vector<life::Node> nodes;
  std::vector<life::Main> mains;
  std::vector<life::Secondary> secondary;
  std::vector<search::Secondary> search_secondary;
  std::vector<double> main_gap;
  std::vector<std::uint32_t> main_nodes;
  std::vector<src::EightSlotSolid> solids;
  std::vector<src::InterfaceIdentity> interfaces;
  std::vector<tied::Interface> ties;
  explicit MixedFixture(unsigned mode=0,bool reverse=false):topology(type25_post_gapm_test::Blocks(mode,true),reverse),native(topology.native) {
    if(topology.report.status!=n::startup::Status::Ok)throw std::runtime_error("Genuine mixed Starter fixture rejected");
    const auto& top=topology.startup;const auto& mesh=topology.mesh;
    for(std::size_t i=0;i<mesh.ids.size();++i){nodes.push_back({mesh.ids[i],0,0});secondary.push_back({std::uint32_t(i),100.,.1,0});search_secondary.push_back({std::uint32_t(i),100.,.1});}
    mains.resize(top.main_count);main_gap.assign(top.main_count,5.);std::vector<bool> seen(nodes.size());
    for(std::size_t m=0;m<top.main_count;++m) {
      const auto& a=top.mains[m];auto& b=mains[m];b.global_id=a.global_id;b.segment_type=a.segment_type;
      b.coefficient=topology.coefficients[m];b.maximum_gap=5.;
      for(unsigned k=0;k<4;++k) {
        b.nodes[k]=a.nodes[k];b.normal_slot[k]=top.starter.face_normals[4*m+k];b.normal_reference[k]=a.normal_reference[k];b.neighbors[k]=a.neighbors[k];b.gap[k]=k==0?5.:.05;
        if(m<top.primary_count&&!seen[a.nodes[k]]){seen[a.nodes[k]]=true;main_nodes.push_back(a.nodes[k]);}
      }
    }
    for(const auto& solid:mesh.physical.solids) {
      src::EightSlotSolid value;value.native_source_id=solid.element_id;value.part_source_id=solid.part_id;
      std::copy_n(solid.nodes,8,value.nodes);solids.push_back(value);
    }
    interfaces={{100,1,src::InterfaceKind::Type25,src::InterfaceOrigin::OriginalDefinition},
      {200,2,src::InterfaceKind::Type25,src::InterfaceOrigin::DeclaredAdditionalInterface}};
  }
  src::Input Input() const {
    src::Input in;in.phase=src::Phase::StarterNormalsAndPreBucGaps;in.stamp={101,topology.input.source_generation,901};in.units={.001,1000,1};
    in.controls={1,1,5,1,1,0,1,1,0,0,2,1,0,4,0,false,double(.20f),0,0,8000000,0,128};
    in.interface_phase=src::InterfaceCensusPhase::CompleteOriginalAndDeclaredAdditions;
    in.interfaces=interfaces.data();in.interface_count=interfaces.size();in.native_interface_id=100;
    in.mesh=topology.input;in.starter=topology.startup;auto& c=in.contact;
    c.nodes=nodes.data();c.node_count=nodes.size();c.mains=mains.data();c.main_count=mains.size();
    c.secondary=secondary.data();c.secondary_count=secondary.size();c.normals=in.starter.starter.references;c.normal_count=in.starter.starter.reference_count;
    c.generation=in.mesh.source_generation;c.normal_to_main={in.starter.normal_offsets,c.normal_count+1,in.starter.normal_mains,in.starter.normal_incidence_count};
    in.main_nodes=main_nodes.data();in.main_node_count=main_nodes.size();in.main_search_gap=main_gap.data();in.main_search_gap_count=main_gap.size();
    in.global_search_gap=.1+5.;in.solid_scope=src::SolidScope::CompleteEightSlotModel;in.solids=solids.data();in.solid_count=solids.size();
    in.contributors={search::Census::CompleteDeclaredModel,nodes.size(),topology.mesh.physical.quads.size()+topology.mesh.physical.triangles.size(),0,0,0,1,0,0};
    return in;
  }
  search::Input Search() const {
    const auto in=Input();search::Input out;out.mesh=in.mesh;out.topology=in.starter;out.contributors=in.contributors;
    out.profile={1,1,2,5,0,0,0,1,search::Initialization::SerialNative,search::LoadCards::Absent};
    out.covered_type25_siblings=1;out.secondary=search_secondary.data();out.secondary_count=search_secondary.size();out.main_gaps=main_gap.data();out.main_count=main_gap.size();return out;
  }
  src::Limits Limits() const {
    src::Limits limits;limits.max_tasks=4096;limits.max_pairs=4096;limits.max_device_bytes=64u<<20;limits.max_host_bytes=64u<<20;
    limits.geometric.max_removals=mains.size()*secondary.size();limits.tied.search=limits.geometric;return limits;
  }
  InventoryInput Inventory(const type25_search_startup_test::NativeResult& geometry) const {
    InventoryInput p;p.multiplier=geometry.scalar[0];p.global_gap=Input().global_search_gap;
    p.removed_nodes.resize(mains.size());
    for(std::size_t i=0;i<nodes.size();++i) {
      const auto x=topology.input.positions.at(std::uint32_t(i));p.positions.push_back({x.x,x.y,x.z});p.codes.push_back(nodes[i].constraint);p.skews.push_back(nodes[i].skew);
    }
    for(auto node:main_nodes)p.main_nodes.push_back(int(node+1));
    for(const auto& row:secondary){p.secondary.push_back(int(row.node+1));p.secondary_coefficients.push_back(row.coefficient);p.secondary_gap.push_back(row.gap);}
    const auto solid_index=[&](std::uint64_t id) {for(std::size_t i=0;i<solids.size();++i)if(solids[i].native_source_id==id)return int(i+1);return 0;};
    for(const auto& solid:solids) {
      Solid value;value.source_id=int(solid.native_source_id);value.part_id=int(solid.part_source_id);
      for(unsigned k=0;k<8;++k)value.nodes[k]=int(solid.nodes[k]+1);p.solids.push_back(value);
    }
    for(std::size_t m=0;m<mains.size();++m) {
      const auto& a=native.mains[m];p.mains.push_back({int(a.nodes[0]+1),int(a.nodes[1]+1),int(a.nodes[2]+1),int(a.nodes[3]+1)});
      p.types.push_back(a.segment_type);p.coefficients.push_back(mains[m].coefficient);p.main_gap.push_back(main_gap[m]);
      p.corner_gaps.push_back({mains[m].gap[0],mains[m].gap[1],mains[m].gap[2],mains[m].gap[3]});
      const auto& support=topology.supports[m];p.support.push_back({support.first.kind==n::startup::PhysicalSupportKind::EightSlotSolid?
          solid_index(support.first.source_element_id):int(solids.size()+1),solid_index(support.second_solid_source_id)});
      if(!geometry.main_offsets.empty())for(auto j=geometry.main_offsets[m];j<geometry.main_offsets[m+1];++j)p.removed_nodes[m].push_back(int(geometry.removed_nodes[j]+1));
    }
    return p;
  }
  type25_search_startup_test::NativeResult Geometry() const {
    type25_search_startup_test::NativeResult empty;empty.scalar[0]=type25_search_startup_test::OracleMultiplier(int(nodes.size()));
    return NativeGeometry(Inventory(empty),topology.startup.primary_count);
  }
};
}
