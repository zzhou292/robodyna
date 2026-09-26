// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "../radioss_type25_search_startup/Fixture.h"
#include <cstring>
namespace type25_tied_removal_test {
namespace search_test=type25_search_startup_test;
namespace s=tlfea::contact::radioss_type25::search_startup;
struct Fixture {
  search_test::Fixture geometry;
  std::vector<std::vector<t::Main>> mains{2};
  std::vector<std::vector<t::Row>> rows{2};
  std::vector<t::Interface> interfaces{2};
  std::vector<t::History> history;
  tl::util::HostArena out,scratch;
  s::GeometricSnapshot pending;
  explicit Fixture(unsigned mode=0):geometry(search_test::old::Grid(4,3,mode)) {
    geometry.Gaps(.6); // Genuine nonempty geometric removal prefix precedes TYPE2.
    const auto n=geometry.mesh.ids.size();
    for(unsigned i=0;i<2;++i) {
      const auto& face=geometry.mesh.primary[i?geometry.mesh.primary.size()-1:0];
      t::Main main;std::copy_n(face.nodes,4,main.nodes);mains[i].push_back(main);
      // Two actual distant ties; source interface IDs deliberately oppose
      // native interface order, so an ID sort cannot silently pass.
      if(i==0)rows[i]={{std::uint32_t(n-1),1,0},{std::uint32_t(n-2),1,1},{2,1,0}};
      else rows[i]={{0,1,0},{1,1,0}};
      interfaces[i]={std::uint64_t(200-100*i),std::uint32_t(2+3*i),28,mains[i].data(),mains[i].size(),rows[i].data(),rows[i].size()};
    }
    history.resize(n);
    for(std::size_t i=0;i<n;++i) {
      history[i].irtlm[0]=int(i%geometry.main_gaps.size()+1);
      history[i].irtlm[1]=100;history[i].irtlm[2]=-7;history[i].irtlm[3]=4;
      for(unsigned k=0;k<5;++k)history[i].penetration[k]=.125*(i+k+1);
      history[i].time[0]=-0.;history[i].time[1]=.25;
    }
    const auto input=Source();const auto f=s::Preflight(input);
    if(f.status!=s::Status::Ok||!out.Initialize(f.output_bytes)||!scratch.Initialize(f.scratch_bytes)||
      s::BuildGeometricBeforeTied(input,{},out,scratch,&pending).status!=s::Status::Ok)
      throw std::runtime_error("Genuine pending geometric fixture failed");
  }
  s::Input Source() const {
    auto in=geometry.Input();in.contributors.tied_interfaces=interfaces.size();
    for(const auto& interface:interfaces)for(std::size_t i=0;i<interface.row_count;++i)in.contributors.cin_links+=interface.rows[i].irupt==0;
    return in;
  }
  t::Input Input() const {
    t::Input in;in.source=Source();in.geometric=pending;in.finalization=t::Finalization::CompactedAfterKinChk;
    in.tied_removal=1;in.interfaces=interfaces.data();in.interface_count=interfaces.size();
    in.history=history.data();in.history_count=history.size();
    // Explicit numerical capacity packet: unused native allocation tail has no
    // geometric entries and must not become a relation or removal.
    in.native_removal_extent=pending.geometry.removal_count+11;return in;
  }
};
struct Built {
  tl::util::HostArena output,scratch;t::Snapshot result;t::Forecast plan;
  explicit Built(const t::Input& in,t::Limits limits={}) {
    plan=t::Preflight(in,limits);
    if(plan.status!=t::Status::Ok||!output.Initialize(plan.output_bytes)||!scratch.Initialize(plan.scratch_bytes))
      throw std::runtime_error("Tied output fixture allocation failed");
  }
  t::Report Run(const t::Input& in,t::Limits limits={}) {return t::Build(in,limits,output,scratch,&result);}
  std::vector<unsigned char> Bytes() const {
    auto* p=static_cast<const unsigned char*>(output.data());return {p,p+output.bytes()};
  }
};
}
