// SPDX-License-Identifier: AGPL-3.0-or-later
// Original I25NEIGH root-label order and PREPARE_SPLIT_I25 local CSR order.
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup::detail {
Report References(const Input& in,Data data,int* root,int* tags,std::uint32_t* node_refs,
    std::size_t& reference_count,std::size_t& incidence_count,const PostGapmTopology* post,
    const Edge* solid_edges,std::size_t solid_edge_count,int* solid_neighbor_ids) noexcept {
  const auto g=data.main_count; int vertices=0;
  for(std::size_t m=0;m<g;++m) {
    auto& main=data.mains[m];
    for(unsigned k=0;k<3;++k) { main.normal_reference[k]=++vertices; root[vertices-1]=vertices; }
    if(main.nodes[3]!=main.nodes[2]) { main.normal_reference[3]=++vertices; root[vertices-1]=vertices; }
    else main.normal_reference[3]=main.normal_reference[2];
  }
  auto merge=[root](int first,int second) noexcept {
    const int low=std::min(root[first-1],root[second-1]);
    const int high=std::max(root[first-1],root[second-1]);
    if(high!=low) root[high-1]=low;
  };
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m];
    std::size_t extra[5]{};
    const bool additional=post && post->final_solid_erosion==SolidErosion::Enabled && tags[m]>0 &&
        post->final_support[m].first.kind==PhysicalSupportKind::EightSlotSolid;
    if(additional && !SolidNeighborLists(data,solid_edges,solid_edge_count,m,solid_neighbor_ids,extra))
      return {Status::ResourceLimit,data.expanded_to_primary[m]};
    for(unsigned k=0;k<4;++k) {
      if(k==2 && main.nodes[2]==main.nodes[3]) continue;
      if(main.neighbors[k]) {
        const auto& other=data.mains[std::size_t(main.neighbors[k]-1)];
        const unsigned slot=unsigned(main.neighbor_edges[k]-1);
        merge(main.normal_reference[k],other.normal_reference[(slot+1)%4]);
        merge(main.normal_reference[(k+1)%4],other.normal_reference[slot]);
      }
      // Native order is ordinary neighbor first, THEN every extra solid list
      // entry at this edge. Extra lists do not write MVOISIN/EVOISIN.
      if(additional)for(auto at=extra[k];at<extra[k+1];++at) {
        const auto id=solid_neighbor_ids[at];
        if(!id || id==main.neighbors[k])continue;
        const auto& other=data.mains[std::size_t(id-1)];
        for(unsigned j=0;j<4;++j) {
          if(j==2 && other.nodes[2]==other.nodes[3])continue;
          if(main.nodes[(k+1)%4]==other.nodes[j] && main.nodes[k]==other.nodes[(j+1)%4]) {
            merge(main.normal_reference[k],other.normal_reference[(j+1)%4]);
            merge(main.normal_reference[(k+1)%4],other.normal_reference[j]);
          }
        }
      }
    }
    if(additional)std::fill_n(solid_neighbor_ids,extra[4],0);
  }
  // Keep the exact source's direct-parent update above and its later closure;
  // replacing it with another union heuristic could change reference numbering.
  for(int i=1;i<=vertices;++i) {
    int j=i;
    while(root[j-1]<j) j=root[j-1];
    root[i-1]=j;
  }
  std::fill_n(tags,std::size_t(vertices),0); int refs=0;
  for(int i=1;i<=vertices;++i) {
    const auto j=root[i-1]; if(!tags[j-1]) tags[j-1]=++refs;
  }
  for(std::size_t m=0;m<g;++m)
    for(auto& ref:data.mains[m].normal_reference) ref=tags[root[ref-1]-1];
  // root storage is dead after all references are mapped. Reuse it as the
  // reference-to-node check. General source topology retains native split fans.
  std::fill_n(root,std::size_t(refs),-1);
  std::fill_n(node_refs,in.node_count,0);
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m]; const unsigned corners=main.nodes[2]==main.nodes[3]?3:4;
    for(unsigned k=0;k<corners;++k) {
      const auto ref=std::size_t(main.normal_reference[k]-1); const int node=static_cast<int>(main.nodes[k]);
      if(root[ref]<0) { root[ref]=node; ++node_refs[std::size_t(node)]; }
      else if(root[ref]!=node) return {Status::UnsupportedTopology,data.expanded_to_primary[m],main.nodes[k]};
      ++data.normal_offsets[ref];
    }
  }
  for(std::size_t i=0;i<in.node_count;++i)
    if(in.topology==TopologyPolicy::ManifoldTwoSided && node_refs[i] && node_refs[i]!=2) return {Status::UnsupportedTopology,SIZE_MAX,i};
  std::uint32_t total=0;
  for(int i=0;i<refs;++i) {
    const auto count=data.normal_offsets[i]; data.normal_offsets[i]=total;
    tags[i]=static_cast<int>(total); total+=count;
  }
  data.normal_offsets[refs]=total;
  // Native PREPARE_SPLIT_I25 scans complete expanded mains, then corners1..3
  // and distinct4. For the admitted one-process source, TAG_SEGM2 is identity.
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m]; const unsigned corners=main.nodes[2]==main.nodes[3]?3:4;
    for(unsigned k=0;k<corners;++k) {
      const auto ref=std::size_t(main.normal_reference[k]-1);
      data.normal_mains[std::size_t(tags[ref]++)]=static_cast<std::uint32_t>(m+1);
    }
  }
  reference_count=std::size_t(refs); incidence_count=total; return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
