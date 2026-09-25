// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup::detail {
namespace {
bool Less(Edge a,Edge b) noexcept {
  if(a.low!=b.low) return a.low<b.low;
  if(a.high!=b.high) return a.high<b.high;
  if(a.main!=b.main) return a.main<b.main;
  return a.slot<b.slot;
}
}
Report Topology(const Input& in,Data data,Edge* edges,std::size_t& edge_count) noexcept {
  const auto g=2*in.primary_count; std::size_t count=0;
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      if(k==2 && main.nodes[2]==main.nodes[3]) continue;
      const auto a=main.nodes[k],b=main.nodes[(k+1)%4];
      edges[count++]={std::min(a,b),std::max(a,b),static_cast<std::uint32_t>(m),k};
    }
  }
  std::sort(edges,edges+count,Less);
  for(std::size_t first=0;first<count;) {
    std::size_t last=first+1;
    while(last<count && edges[last].low==edges[first].low && edges[last].high==edges[first].high) ++last;
    // Every physical primary contributes its two opposite sides. This first
    // source profile admits only boundary edges and two-primary manifold edges.
    if(last-first!=2 && last-first!=4)
      return {Status::UnsupportedTopology,data.expanded_to_primary[edges[first].main],edges[first].low};
    for(std::size_t i=first;i<last;++i) {
      const auto a=edges[i]; auto& main=data.mains[a.main];
      unsigned candidates=0;
      for(std::size_t j=first;j<last;++j) {
        const auto b=edges[j];
        if(data.expanded_to_primary[a.main]==data.expanded_to_primary[b.main]) continue;
        const auto& other=data.mains[b.main];
        if(main.nodes[a.slot]!=other.nodes[(b.slot+1)%4] || main.nodes[(a.slot+1)%4]!=other.nodes[b.slot]) continue;
        ++candidates;
        main.neighbors[a.slot]=static_cast<int>(b.main+1);
        main.neighbor_edges[a.slot]=static_cast<int>(b.slot+1);
      }
      // Under this admitted topology, native SEG_E/SEG_EN has exactly one
      // reversed-edge candidate (or none at a boundary), never its own partner.
      if(candidates!=(last-first==4?1u:0u))
        return {Status::UnsupportedTopology,data.expanded_to_primary[a.main],a.low};
    }
    first=last;
  }
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) if(main.neighbors[k]) {
      const auto other=std::size_t(main.neighbors[k]-1);
      const auto slot=unsigned(main.neighbor_edges[k]-1);
      if(slot>=4 || data.mains[other].neighbors[slot]!=static_cast<int>(m+1))
        return {Status::UnsupportedTopology,data.expanded_to_primary[m]};
      // A valid two-dimensional cell complex has at most one shared edge per
      // primary-face pair. This also excludes the native T3/quad NOK ambiguity.
      for(unsigned j=0;j<k;++j) if(main.neighbors[j] &&
          data.expanded_to_primary[std::size_t(main.neighbors[j]-1)]==data.expanded_to_primary[other])
        return {Status::UnsupportedTopology,data.expanded_to_primary[m]};
    }
  }
  edge_count=count; return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
