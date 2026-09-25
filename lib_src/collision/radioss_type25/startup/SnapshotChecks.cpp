// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "FloatNormals.h"
#include "../search/Ranges.h"
namespace tlfea::contact::radioss_type25::startup::detail {
Report CheckSnapshot(const Input& in,const Snapshot& view,const FixedMainInput& active,
    const tl::util::HostArena& output,const tl::util::HostArena& scratch,const FixedMainView* published) noexcept {
  namespace r=search::detail;
  const auto g=2*in.primary_count,n=view.starter.reference_count;
  if(view.primary_roles || view.primary_role_count || view.profile!=in.profile || view.topology!=in.topology || view.node_count!=in.node_count || view.primary_count!=in.primary_count || view.main_count!=g ||
      view.source_generation!=in.source_generation || !n || n>4*g || view.normal_incidence_count>4*g ||
      active.main_count!=g || !r::Span(active.main_coefficients,g) || !r::Span(view.mains,g) ||
      !r::Span(view.expanded_to_primary,g) || !r::Span(view.primary_to_partner,in.primary_count) ||
      !r::Span(view.normal_offsets,n+1) || !r::Span(view.normal_mains,view.normal_incidence_count) ||
      !r::Span(view.starter.face_normals,4*g) || !r::Span(view.starter.references,n))
    return {Status::InvalidInput};
  struct Range {const void* data;std::size_t bytes;};
  const Range reads[]{{&view,sizeof(view)},{&active,sizeof(active)},
      {active.main_coefficients,g*sizeof(double)},{view.mains,g*sizeof(Main)},
      {view.expanded_to_primary,g*sizeof(std::uint32_t)},
      {view.primary_to_partner,in.primary_count*sizeof(std::uint32_t)},
      {view.normal_offsets,(n+1)*sizeof(std::uint32_t)},
      {view.normal_mains,view.normal_incidence_count*sizeof(std::uint32_t)},
      {view.starter.face_normals,4*g*sizeof(StoredNormal)},
      {view.starter.references,n*sizeof(NormalReference)}};
  for(const auto& span:reads)
    if(!Disjoint(output.data(),output.bytes(),span.data,span.bytes) ||
        !Disjoint(scratch.data(),scratch.bytes(),span.data,span.bytes) ||
        !Disjoint(published,sizeof(FixedMainView),span.data,span.bytes)) return {Status::InvalidInput};
  constexpr unsigned reversed[]{1,0,3,2};
  for(std::size_t m=0;m<g;++m) {
    const auto primary=m<in.primary_count?m:m-in.primary_count;
    const auto& main=view.mains[m];const auto& original=in.primary[primary];
    if(!std::isfinite(active.main_coefficients[m]))return {Status::InvalidInput,primary};
    if(active.main_coefficients[m]<=0)return {Status::UnsupportedProfile,primary};
    if(view.expanded_to_primary[m]!=primary || main.source_id!=original.source_id ||
        main.global_id!=static_cast<int>(m+1) ||
        main.segment_type!=(m<in.primary_count?static_cast<int>(in.primary_count+m+1):-static_cast<int>(primary+1)))
      return {Status::InvalidInput,primary};
    if(main.nodes[2]==main.nodes[3] && main.normal_reference[2]!=main.normal_reference[3])
      return {Status::InvalidInput,primary};
    for(unsigned k=0;k<4;++k) {
      if(main.nodes[k]!=original.nodes[m<in.primary_count?k:reversed[k]] ||
          main.nodes[k]>=in.node_count || main.normal_reference[k]<=0 ||
          std::size_t(main.normal_reference[k])>n || main.neighbors[k]<0 ||
          std::size_t(main.neighbors[k])>g || main.neighbor_edges[k]<0 || main.neighbor_edges[k]>4 ||
          (main.neighbors[k]==0)!=(main.neighbor_edges[k]==0) || !fp::Finite(view.starter.face_normals[4*m+k]))
        return {Status::InvalidInput,primary};
      if(main.neighbors[k]) {
        const auto other=std::size_t(main.neighbors[k]-1);const auto edge=unsigned(main.neighbor_edges[k]-1);
        const auto& neighbor=view.mains[other];
        if(neighbor.nodes[edge]!=main.nodes[(k+1)%4] ||
            neighbor.nodes[(edge+1)%4]!=main.nodes[k] ||
            neighbor.neighbors[edge]!=static_cast<int>(m+1) ||
            neighbor.neighbor_edges[edge]!=static_cast<int>(k+1) ||
            neighbor.normal_reference[edge]!=main.normal_reference[(k+1)%4] ||
            neighbor.normal_reference[(edge+1)%4]!=main.normal_reference[k])
          return {Status::InvalidInput,primary};
      }
    }
  }
  for(std::size_t p=0;p<in.primary_count;++p)
    if(view.primary_to_partner[p]!=in.primary_count+p+1)return {Status::InvalidInput,p};
  if(view.normal_offsets[0]!=0 || view.normal_offsets[n]!=view.normal_incidence_count)return {Status::InvalidInput};
  for(std::size_t i=0;i<n;++i) {
    if(view.normal_offsets[i]>view.normal_offsets[i+1] || view.normal_offsets[i+1]>view.normal_incidence_count ||
        view.starter.references[i].boundary<0 || view.starter.references[i].boundary>1 ||
        !fp::Finite(view.starter.references[i].bisector[0]) || !fp::Finite(view.starter.references[i].bisector[1]))
      return {Status::InvalidInput};
  }
  for(std::size_t i=0;i<view.normal_incidence_count;++i)
    if(!view.normal_mains[i] || view.normal_mains[i]>g)return {Status::InvalidInput};
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
