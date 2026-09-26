// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "../radioss_type25_fixed_main_startup/Fixture.h"
#include "lib_src/collision/radioss_type25/initial_state/Pair.h"
#include "lib_src/collision/radioss_type25/initial_state/Winner.h"
#include <algorithm>
namespace initial_state_test {
namespace st=type25_startup_test;
namespace init=n::initial_state;
struct Fixture {
  st::Case mesh;
  st::NativeResult topology;
  std::vector<n::search_startup::Secondary> secondary;
  std::vector<std::array<double,4>> gaps;
  std::vector<std::array<int,2>> candidates;
  explicit Fixture(unsigned mode=0,bool warped=false):mesh(st::Grid(2,1,mode)) {
    if(warped)for(std::size_t i=0;i<mesh.ids.size();++i)
      mesh.positions[3*i+2]=.04*mesh.positions[3*i]*mesh.positions[3*i+1];
    const n::Vector points[]{{.25,.25,.035},{.8,.4,-.04},{1.2,.7,.025},{1.75,.7,.4},{.001,.2,.001}};
    for(const auto& p:points) {
      secondary.push_back({std::uint32_t(mesh.ids.size()),210000.,.1});
      mesh.ids.push_back(mesh.ids.size()+1);mesh.positions.insert(mesh.positions.end(),{p.x,p.y,p.z});
    }
    topology=st::Oracle(mesh.Input(),mesh.coefficients.data(),mesh.coefficients.size());
    gaps.resize(topology.mains.size());
    for(std::size_t m=0;m<gaps.size();++m) {
      for(unsigned k=0;k<4;++k)gaps[m][k]=.05+.01*double(topology.mains[m].nodes[k]%3);
      for(std::size_t row=0;row<secondary.size();++row)candidates.push_back({int(row+1),int(m+1)});
    }
  }
  n::search_startup::Input Input() const {
    n::search_startup::Input in;in.mesh=mesh.Input();in.secondary=secondary.data();in.secondary_count=secondary.size();return in;
  }
  init::PairInput Pair(std::size_t occurrence,int sharp=1) const {
    const auto hit=candidates.at(occurrence);const auto row=std::size_t(hit[0]-1),m=std::size_t(hit[1]-1);
    const auto& main=topology.mains[m];const auto& source=secondary[row];
    init::PairInput out;out.expanded_main_count=int(topology.mains.size());out.profile.sharp=sharp;
    auto& in=out.geometry;in.key={mesh.ids[source.node],7,row,main.global_id};
    in.local_main=int(m+1);in.occurrence=occurrence;in.segment_type=main.segment_type;
    const auto secondary_position=mesh.Input().positions.at(source.node);
    in.secondary={secondary_position.x,secondary_position.y,secondary_position.z};in.secondary_gap=source.gap;
    in.main_gap_max=n::native_constant::ep20*n::native_constant::ep10;
    in.secondary_coefficient=source.stiffness;in.main_coefficient=mesh.coefficients[m];
    for(unsigned k=0;k<4;++k) {
      in.main_node_ids[k]=mesh.ids[main.nodes[k]];const auto position=mesh.Input().positions.at(main.nodes[k]);
      in.main_vertices[k]={position.x,position.y,position.z};
      in.normal_slot[k]=topology.starter_normals[4*m+k];in.neighbors[k]=main.neighbors[k];in.main_gap[k]=gaps[m][k];
      const auto& ref=topology.starter_references.at(main.normal_reference[k]-1);
      in.boundary_ids[k]=ref.boundary?std::uint64_t(main.normal_reference[k]):0;
      for(unsigned j=0;j<2;++j)in.vertex_bisector[k][j]=ref.bisector[j];
    }
    return out;
  }
};
}
