// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "../radioss_type25_initial_state/NativeOracle.h"
#include "../radioss_type25_search_startup/NativeOracle.h"
#include "../radioss_type25_tied_removal/NativeOracle.h"
#include "../radioss_type25_fixed_main_startup/Fixture.h"
#include "lib_src/collision/RadiossType25InitialState.h"
#include <algorithm>
#include <numeric>
namespace initial_source_test {
namespace n=tlfea::contact::radioss_type25;
namespace src=n::initial_source;
namespace life=n::selection::lifecycle;
namespace st=type25_startup_test;
namespace search=n::search_startup;
namespace tied=n::tied_removal;
struct Fixture {
  st::Case mesh;
  st::Built topology;
  st::NativeResult native;
  std::vector<life::Node> nodes;
  std::vector<life::Main> mains;
  std::vector<life::Secondary> secondary;
  std::vector<search::Secondary> search_secondary;
  std::vector<double> main_gap;
  std::vector<std::uint32_t> main_nodes;
  std::vector<src::InterfaceIdentity> interfaces;
  std::vector<src::EightSlotSolid> solids;
  std::vector<tied::Main> tied_mains;
  std::vector<tied::Row> tied_rows;
  std::vector<tied::Interface> ties;
  double global_gap=.1+.05;
  static st::Case Mesh(unsigned mode,bool warped) {
    auto out=st::Grid(3,2,mode);
    if(warped)for(std::size_t i=0;i<out.ids.size();++i)out.positions[3*i+2]=.035*out.positions[3*i]*out.positions[3*i+1];
    const n::Vector extra[]{{.25,.25,.035},{1.2,.75,-.04},{2.4,1.2,.08},{.4,.25,1.5}};
    for(const auto& x:extra){out.ids.push_back(out.ids.size()+1);out.positions.insert(out.positions.end(),{x.x,x.y,x.z});}
    return out;
  }
  static st::Case Distant(double distance) {
    st::Case out;const n::Vector points[]{{0,0,0},{1,0,0},{1,1,0},{0,1,0},
      {distance,0,0},{distance+1,0,0},{distance+1,1,0},{distance,1,0},{.4,.4,.02},{distance+.5,.5,.02}};
    for(const auto& x:points){out.ids.push_back(out.ids.size()+1);out.positions.insert(out.positions.end(),{x.x,x.y,x.z});}
    out.Add(n::ShellLayout::Quad4,0,1,2,3);out.Add(n::ShellLayout::Quad4,4,5,6,7);return out;
  }
  explicit Fixture(unsigned mode=0,bool warped=false,bool with_tied=false):Fixture(Mesh(mode,warped),with_tied){}
  explicit Fixture(st::Case value,bool with_tied=false):mesh(std::move(value)),topology(mesh),
      native(st::Oracle(mesh.Input(),mesh.coefficients.data(),mesh.coefficients.size())) {
    const auto n=mesh.ids.size(),g=topology.startup.main_count;
    for(std::size_t i=0;i<n;++i) {
      nodes.push_back({mesh.ids[i],0,0});secondary.push_back({std::uint32_t(i),210000.,.1,0});
      search_secondary.push_back({std::uint32_t(i),210000.,.1});
    }
    mains.resize(g);main_gap.assign(g,.05);std::vector<bool> seen(n);
    for(std::size_t m=0;m<g;++m) {
      const auto& a=topology.startup.mains[m];auto& b=mains[m];b.global_id=a.global_id;b.segment_type=a.segment_type;b.coefficient=210000.;b.maximum_gap=.05;
      for(unsigned k=0;k<4;++k) {
        b.nodes[k]=a.nodes[k];b.normal_slot[k]=topology.startup.starter.face_normals[4*m+k];
        b.normal_reference[k]=a.normal_reference[k];b.neighbors[k]=a.neighbors[k];b.gap[k]=.05;
        if(m<mesh.primary.size()&&!seen[a.nodes[k]]){seen[a.nodes[k]]=true;main_nodes.push_back(a.nodes[k]);}
      }
    }
    if(with_tied) {
      tied::Main main;std::copy_n(mesh.primary[0].nodes,4,main.nodes);tied_mains.push_back(main);
      tied_rows.push_back({std::uint32_t(n-4),1,0});tied_rows.push_back({std::uint32_t(n-3),1,1});
      ties.push_back({300,2,28,tied_mains.data(),tied_mains.size(),tied_rows.data(),tied_rows.size()});
      interfaces.push_back({300,2,src::InterfaceKind::Type2,src::InterfaceOrigin::OriginalDefinition});
    }
    interfaces.push_back({100,4,src::InterfaceKind::Type25,src::InterfaceOrigin::OriginalDefinition});
  }
  src::Input Input() const {
    src::Input in;in.phase=src::Phase::StarterNormalsAndPreBucGaps;in.stamp={101,7,901};in.units=mesh.scale;
    in.controls={1,1,5,1,1,0,1,1,0,0,2,1,0,4,0,false,double(.20f),0,0,8000000,0,128};
    in.interface_phase=src::InterfaceCensusPhase::CompleteOriginalAndDeclaredAdditions;
    in.interfaces=interfaces.data();in.interface_count=interfaces.size();in.native_interface_id=100;
    in.mesh=mesh.Input();in.starter=topology.startup;auto& c=in.contact;
    c.nodes=nodes.data();c.node_count=nodes.size();c.mains=mains.data();c.main_count=mains.size();
    c.secondary=secondary.data();c.secondary_count=secondary.size();c.normals=topology.startup.starter.references;
    c.normal_count=topology.startup.starter.reference_count;c.generation=7;
    c.normal_to_main={topology.startup.normal_offsets,c.normal_count+1,topology.startup.normal_mains,topology.startup.normal_incidence_count};
    in.main_nodes=main_nodes.data();in.main_node_count=main_nodes.size();in.main_search_gap=main_gap.data();in.main_search_gap_count=main_gap.size();
    in.global_search_gap=global_gap;in.solid_scope=solids.empty()?src::SolidScope::ExplicitNoSolids:src::SolidScope::CompleteEightSlotModel;
    in.solids=solids.empty()?nullptr:solids.data();in.solid_count=solids.size();
    in.contributors={search::Census::CompleteDeclaredModel,nodes.size(),mesh.primary.size(),ties.size(),0,0,interfaces.size()-ties.size()-1,0,0};
    for(const auto& tie:ties)for(std::size_t row=0;row<tie.row_count;++row)in.contributors.cin_links+=tie.rows[row].irupt==0;
    in.tied_phase=ties.empty()?tied::Finalization::Unspecified:tied::Finalization::CompactedAfterKinChk;
    in.tied_interfaces=ties.empty()?nullptr:ties.data();in.tied_interface_count=ties.size();return in;
  }
  search::Input Search() const {
    const auto in=Input();search::Input out;out.mesh=in.mesh;out.topology=in.starter;out.contributors=in.contributors;
    out.profile={1,1,2,5,0,0,0,1,search::Initialization::SerialNative,search::LoadCards::Absent};
    out.secondary=search_secondary.data();out.secondary_count=search_secondary.size();out.main_gaps=main_gap.data();out.main_count=mains.size();
    return out;
  }
  src::Limits Limits() const {
    src::Limits l;l.max_tasks=4096;l.max_pairs=4096;l.max_device_bytes=64u<<20;l.max_host_bytes=64u<<20;
    l.geometric.max_removals=mains.size()*secondary.size();l.tied.search=l.geometric;
    return l;
  }
  InventoryInput Inventory(const type25_search_startup_test::NativeResult& geometry) const {
    InventoryInput p;const auto in=Input();p.multiplier=geometry.scalar[0];p.global_gap=in.global_search_gap;
    p.removed_nodes.resize(mains.size());p.support.resize(mains.size(),{1,0});
    for(std::size_t i=0;i<nodes.size();++i) {
      p.positions.push_back({mesh.positions[3*i],mesh.positions[3*i+1],mesh.positions[3*i+2]});
      p.codes.push_back(nodes[i].constraint);p.skews.push_back(nodes[i].skew);
    }
    for(auto node:main_nodes)p.main_nodes.push_back(int(node+1));
    for(const auto& row:secondary){p.secondary.push_back(int(row.node+1));p.secondary_coefficients.push_back(row.coefficient);p.secondary_gap.push_back(row.gap);}
    for(std::size_t i=0;i<mains.size();++i) {
      const auto& a=native.mains[i];p.mains.push_back({int(a.nodes[0]+1),int(a.nodes[1]+1),int(a.nodes[2]+1),int(a.nodes[3]+1)});
      p.types.push_back(a.segment_type);p.coefficients.push_back(mains[i].coefficient);p.main_gap.push_back(main_gap[i]);
      p.corner_gaps.push_back({mains[i].gap[0],mains[i].gap[1],mains[i].gap[2],mains[i].gap[3]});
      for(auto k=geometry.main_offsets[i];k<geometry.main_offsets[i+1];++k)p.removed_nodes[i].push_back(int(geometry.removed_nodes[k]+1));
    }
    return p;
  }
};
}
