// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/selection/lifecycle/Host.h"
#include "lib_utils/OrderedNodeIncidence.h"
#include <stdexcept>
#include <vector>
namespace type25_lifecycle_test {
namespace n=tlfea::contact::radioss_type25;
namespace l=n::selection::lifecycle;
struct Fixture {
  std::vector<l::Node> nodes;
  std::vector<double> positions,velocities;
  std::vector<l::Main> mains;
  std::vector<l::Secondary> secondary;
  std::vector<l::NormalReference> normals;
  std::vector<n::NativeGeometryHistory> accepted;
  std::vector<l::SpatialOccurrence> spatial;
  std::vector<std::uint32_t> normal_offsets,normal_entries,removed_offsets,removed_entries;
  std::vector<std::uint32_t> spatial_offsets,spatial_entries;
  l::Profile profile;
  l::Step step{0,.001};
  l::KinematicsUnits units=l::KinematicsUnits::Native;
  n::UnitScale scale{.001,1000,1};
  Fixture() {
    nodes.resize(7);for(unsigned i=0;i<nodes.size();++i)nodes[i].source_id=100+i;
    positions={0,0,0, 4,0,0, 4,4,0, 0,4,0, 8,0,0, 8,4,0, 2,1,.2};
    velocities.resize(positions.size());
    mains.resize(3);const unsigned connectivity[3][4]{{0,1,2,3},{1,0,3,2},{1,4,5,2}};
    const int references[3][4]{{1,2,3,4},{2,1,4,3},{2,5,6,3}};
    const n::StoredNormal normal[4]{{0,-1,0},{1,0,0},{0,1,0},{-1,0,0}};
    constexpr unsigned reversed[]{0,3,2,1};
    for(unsigned m=0;m<3;++m) {
      auto& main=mains[m];main.global_id=int(11*(m+1));main.coefficient=400;main.maximum_gap=.4;
      for(unsigned j=0;j<4;++j) {
        main.nodes[j]=connectivity[m][j];main.normal_reference[j]=references[m][j];
        main.normal_slot[j]=normal[m==1?reversed[j]:j];main.neighbors[j]=1;main.gap[j]=.4;
      }
    }
    mains[0].segment_type=2;mains[1].segment_type=-1;mains[2].segment_type=0;
    normals.resize(6);
    secondary.push_back({6,2,.2,0});
    accepted.resize(1);accepted[0].secondary_source_id=nodes[6].source_id;accepted[0].generation=7;
    spatial={{1,1},{1,3}};
    profile.selection={1,5,1,false,false,false};profile.geometry={1,1,5,1,false,false,false};
    profile.coefficient={4,0};profile.minimum_coefficient=0;profile.maximum_coefficient=1e30;
    profile.neighbor_removal=2;profile.optcd_response_precision=0;
    Rebuild();
  }
  void Rebuild() {
    normal_offsets.resize(normals.size()+1);std::vector<std::uint32_t> incidence(4*mains.size());
    if(!tl::util::BuildOrderedNodeIncidence<4>(mains.size(),normals.size(),
        [&](std::size_t m,unsigned j){return std::uint32_t(mains[m].normal_reference[j]-1);},
        normal_offsets.data(),normal_offsets.size(),incidence.data(),incidence.size()))
      throw std::runtime_error("Invalid fixture normal incidence");
    // I25NORM's source incidence omits the repeated fourth T3 node/reference.
    // Compact only those duplicate source slots, preserving main/slot order.
    const auto full_offsets=normal_offsets;normal_entries.clear();
    for(std::size_t ref=0;ref<normals.size();++ref) {
      normal_offsets[ref]=std::uint32_t(normal_entries.size());
      for(auto j=full_offsets[ref];j<full_offsets[ref+1];++j) {
        const auto encoded=incidence[j],main=encoded/4,slot=encoded%4;
        if(slot==3&&mains[main].nodes[2]==mains[main].nodes[3])continue;
        normal_entries.push_back(main+1);
      }
    }
    normal_offsets[normals.size()]=std::uint32_t(normal_entries.size());
    spatial_offsets.resize(secondary.size()+1);spatial_entries.resize(spatial.size());
    if(!spatial.empty()&&!tl::util::BuildOrderedNodeIncidence<1>(spatial.size(),secondary.size(),
        [&](std::size_t i,unsigned){return std::uint32_t(spatial[i].secondary-1);},
        spatial_offsets.data(),spatial_offsets.size(),spatial_entries.data(),spatial_entries.size()))
      throw std::runtime_error("Invalid fixture spatial incidence");
    if(spatial.empty())for(auto& offset:spatial_offsets)offset=0;
    removed_offsets.assign(secondary.size()+1,0);removed_entries.clear();
  }
  l::Input Input() const {
    l::Input out;out.profile=profile;out.step=step;
    auto& source=out.source;
    source.nodes=nodes.data();source.node_count=nodes.size();
    source.mains=mains.data();source.main_count=mains.size();
    source.secondary=secondary.data();source.secondary_count=secondary.size();
    source.normals=normals.data();source.normal_count=normals.size();source.generation=7;
    source.normal_to_main={normal_offsets.data(),normal_offsets.size(),normal_entries.data(),normal_entries.size()};
    source.removed_main_by_secondary={removed_offsets.data(),removed_offsets.size(),removed_entries.data(),removed_entries.size()};
    out.current.positions={positions.data(),std::uint32_t(nodes.size()),3,1};
    out.current.velocities={velocities.data(),std::uint32_t(nodes.size()),3,1};
    out.current.units=units;out.current.native_units=scale;
    out.accepted_rows=accepted.data();out.accepted_row_count=accepted.size();
    out.spatial=spatial.data();out.spatial_count=spatial.size();
    out.spatial_by_secondary={spatial_offsets.data(),spatial_offsets.size(),spatial_entries.data(),spatial_entries.size()};
    return out;
  }
  void TrianglePair() {
    mains[0].nodes[2]=mains[0].nodes[3]=3;
    mains[0].normal_reference[2]=mains[0].normal_reference[3]=4;
    mains[0].normal_slot[1]={0x1.6a09e6p-1f,0x1.6a09e6p-1f,0};
    mains[0].normal_slot[2]={}; // Native unused T3 NOD_NORMAL slot3.
    constexpr unsigned node_order[]{1,0,3,2},edge_order[]{0,3,2,1};
    for(unsigned k=0;k<4;++k) {
      mains[1].nodes[k]=mains[0].nodes[node_order[k]];
      mains[1].normal_reference[k]=mains[0].normal_reference[node_order[k]];
      mains[1].normal_slot[k]=mains[0].normal_slot[edge_order[k]];
    }
    positions[18]=1;positions[19]=2;Rebuild();
  }
  void AddSecondary(double x=2,double y=1,double z=.2) {
    const auto index=std::uint32_t(nodes.size());nodes.push_back({100+index,0,0});
    positions.insert(positions.end(),{x,y,z});velocities.insert(velocities.end(),{0,0,0});
    secondary.push_back({index,2,.2,0});accepted.push_back({});
    accepted.back().secondary_source_id=nodes.back().source_id;accepted.back().generation=7;
  }
  void Retained() {
    accepted[0].row.irtlm[0]=11;accepted[0].row.irtlm[1]=1;
    accepted[0].row.irtlm[2]=1;accepted[0].row.irtlm[3]=1;
    accepted[0].row.history.normal={.1,100,.2,200,.05};
    accepted[0].row.history.previous_force={.25,.5,.75};
    accepted[0].row.history.staged_force={1,2,3};
  }
  static l::Limits Limits(){return {8,128,512,1u<<20};}
};
} // namespace type25_lifecycle_test
