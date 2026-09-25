// SPDX-License-Identifier: AGPL-3.0-or-later
// I7REMNODE_INIT, serial get_list_remnode and I25REMNOR, selected rawIGAP1.
#include "Internal.h"
namespace tlfea::contact::radioss_type25::search_startup::detail {
namespace {
constexpr double Large = native_constant::ep20*native_constant::ep10;
Report Incidence(const Input& in,Work w,bool& self,double& minimum) noexcept {
  const auto n=in.mesh.node_count,g=in.main_count;
  std::fill_n(w.node_offsets,n+1,0u);self=false;minimum=Large;
  for (std::size_t m=0;m<g;++m) {
    const auto& main=in.topology.mains[m];bool participates=false;
    for (unsigned k=0;k<4;++k)
      if(w.secondary_index[main.nodes[k]]!=UINT32_MAX)participates=true;
    if (participates) {
      self=true;
      for(unsigned k=0;k<Corners(main);++k)++w.node_offsets[main.nodes[k]];
    }
  }
  if (!self) return {Status::Ok};
  for (std::size_t m=0;m<g;++m) {
    const auto& main=in.topology.mains[m];const auto corners=Corners(main);
    for (unsigned k=0;k<corners;++k) {
      const auto a=w.points[main.nodes[k]],b=w.points[main.nodes[(k+1)%corners]];
      const double x=a.x-b.x,y=a.y-b.y,z=a.z-b.z;
      const double squared=x*x+y*y+z*z;
      if(!std::isfinite(squared))return {Status::NonfiniteResult,m};
      minimum=std::min(minimum,squared);
    }
  }
  minimum=std::sqrt(minimum);
  std::uint32_t total=0;
  for(std::size_t node=0;node<n;++node) {
    const auto count=w.node_offsets[node];w.node_offsets[node]=total;
    w.cursors[node]=total;total+=count;
  }
  w.node_offsets[n]=total;
  for (std::size_t m=0;m<g;++m) {
    const auto& main=in.topology.mains[m];bool participates=false;
    for(unsigned k=0;k<Corners(main);++k)
      if(w.secondary_index[main.nodes[k]]!=UINT32_MAX)participates=true;
    if(participates)for(unsigned k=0;k<Corners(main);++k)
      w.node_mains[w.cursors[main.nodes[k]]++]=static_cast<std::uint32_t>(m);
  }
  return {Status::Ok};
}
void Reset(const Input& in,Work w) noexcept {
  const auto n=in.mesh.node_count;
  std::fill_n(w.tag,n,0);std::fill_n(w.expanded,n,0);
  std::fill_n(w.distance,n,Large);std::fill_n(w.gap,n,0.);
  std::fill_n(w.segment_tag,in.main_count,0);
}
Report Traverse(const Input& in,Limits limits,Work w,Data data,double minimum,
    double maximum_secondary_gap,bool store,std::size_t& total) noexcept {
  Reset(in,w);total=0;std::size_t visits=0;
  const double root_two=std::sqrt(2.);
  for(std::size_t row=0;row<in.main_count;++row) {
    const double bound=root_two*std::max(in.main_gaps[row]+maximum_secondary_gap+0.,0.);
    if(!std::isfinite(bound))return {Status::NonfiniteResult,row};
    std::size_t discovered=0,current_count=1,visited=0;
    int level=1;double nearest=0;
    w.segment_tag[row]=level;w.current[0]=static_cast<std::uint32_t>(row);
    const auto& original=in.topology.mains[row];
    for(unsigned k=0;k<Corners(original);++k) {
      const auto node=original.nodes[k];w.tag[node]=1;w.distance[node]=0.;
    }
    while((nearest+minimum)<=bound && current_count!=0) {
      ++level;nearest=Large;std::size_t next_count=0;
      for(std::size_t i=0;i<current_count;++i) {
        const auto segment=w.current[i];const auto& main=in.topology.mains[segment];
        const auto corners=Corners(main);
        for(unsigned j=0;j<corners;++j)w.tag[main.nodes[j]]=2;
        for(unsigned j=0;j<corners;++j) {
          const auto node=main.nodes[j];if(w.expanded[node])continue;
          w.expanded[node]=1;
          for(auto k=w.node_offsets[node];k<w.node_offsets[node+1];++k) {
            if(++visits>limits.max_neighbor_visits)return {Status::ResourceLimit,row};
            const auto other=w.node_mains[k];const auto& adjacent=in.topology.mains[other];
            if(w.segment_tag[other]!=0 && w.segment_tag[other]!=level)continue;
            if(w.segment_tag[other]==0) {
              if(next_count>=in.main_count)return {Status::InvalidInput,row};
              w.next[next_count++]=other;
            }
            w.segment_tag[other]=level;
            for(unsigned l=0;l<Corners(adjacent);++l) {
              const auto candidate=adjacent.nodes[l];
              if(w.secondary_index[candidate]==UINT32_MAX || w.tag[candidate]==2)continue;
              const double distance=w.distance[node]+Distance(w.points[candidate],w.points[node]);
              if(!std::isfinite(distance))return {Status::NonfiniteResult,row,candidate};
              w.distance[candidate]=std::min(w.distance[candidate],distance);
              nearest=std::min(nearest,w.distance[candidate]);
              if(w.tag[candidate]==0) {
                if(discovered>=in.mesh.node_count)return {Status::InvalidInput,row};
                w.tag[candidate]=1;w.discovered[discovered++]=candidate;
              }
              double gap=w.secondary_gap[candidate]+in.main_gaps[row];
              if(!std::isfinite(gap))return {Status::NonfiniteResult,row,candidate};
              gap=std::min(Large,gap);gap=std::max(0.,gap);gap=std::max(0.,gap+0.);
              w.gap[candidate]=gap;
            }
          }
        }
        for(unsigned j=0;j<4;++j)w.tag[main.nodes[j]]=1;
      }
      for(std::size_t i=0;i<current_count;++i) {
        const auto& main=in.topology.mains[w.current[i]];
        for(unsigned j=0;j<Corners(main);++j)w.expanded[main.nodes[j]]=0;
      }
      current_count=next_count;
      if(current_count==0)break;
      if(current_count>in.main_count-visited)return {Status::InvalidInput,row};
      for(std::size_t j=0;j<current_count;++j) {
        w.current[j]=w.next[j];w.next[j]=0;w.visited[visited+j]=w.current[j];
      }
      visited+=current_count;
    }
    if(level==1) {
      if(!store)data.main_offsets[row+1]=0;
      // Source intentionally leaves worker-local tag/distance scratch here.
      // Preserve it; generic mixed-worker equivalence is not claimed.
      continue;
    }
    for(unsigned k=0;k<4;++k)w.distance[original.nodes[k]]=Large;
    std::size_t row_count=0;
    for(std::size_t j=0;j<discovered;++j) {
      const auto node=w.discovered[j];
      const double bound_here=root_two*w.gap[node];
      if(!std::isfinite(bound_here))return {Status::NonfiniteResult,row,node};
      if(w.distance[node]<=bound_here) {
        if(store) {
          if(row_count>=data.main_offsets[row+1]-data.main_offsets[row])return {Status::InvalidInput,row};
          data.removed_nodes[data.main_offsets[row]+row_count]=node;
        }
        ++row_count;
      }
    }
    if(store && row_count!=data.main_offsets[row+1]-data.main_offsets[row])
      return {Status::InvalidInput,row};
    if(!store)data.main_offsets[row+1]=static_cast<std::uint32_t>(row_count);
    total+=row_count;
    for(std::size_t j=0;j<discovered;++j)w.distance[w.discovered[j]]=Large;
    for(std::size_t j=0;j<visited;++j) {
      const auto segment=w.visited[j];const auto& main=in.topology.mains[segment];
      for(unsigned k=0;k<4;++k)w.tag[main.nodes[k]]=0;
      w.segment_tag[segment]=0;w.visited[j]=0;
    }
    for(unsigned k=0;k<4;++k)w.tag[original.nodes[k]]=0;
    w.segment_tag[row]=0;
  }
  return {Status::Ok,SIZE_MAX,SIZE_MAX,total,true};
}
}
Report Removals(const Input& in,Limits limits,Work w,Data data,std::size_t capacity,
    double maximum_secondary_gap) noexcept {
  bool self=false;double minimum=0;
  auto report=Incidence(in,w,self,minimum);if(report.status!=Status::Ok)return report;
  if(!self)return {Status::Ok,SIZE_MAX,SIZE_MAX,0,true};
  if(in.profile.initialization==Initialization::InvariantNoExpansion) {
    const double root_two=std::sqrt(2.);
    for(std::size_t i=0;i<in.main_count;++i) {
      const double bound=root_two*std::max(in.main_gaps[i]+maximum_secondary_gap+0.,0.);
      if(!std::isfinite(bound))return {Status::NonfiniteResult,i};
      if(minimum<=bound)return {Status::UnsupportedProfile,i};
    }
    // Every source loop remains level1, independently of worker assignment.
    return {Status::Ok,SIZE_MAX,SIZE_MAX,0,true};
  }
  std::size_t total=0;
  report=Traverse(in,limits,w,data,minimum,maximum_secondary_gap,false,total);
  if(report.status!=Status::Ok)return report;
  if(total>capacity)return {Status::ResourceLimit,SIZE_MAX,SIZE_MAX,total,true};
  for(std::size_t i=0;i<in.main_count;++i)data.main_offsets[i+1]+=data.main_offsets[i];
  std::size_t repeated=0;
  report=Traverse(in,limits,w,data,minimum,maximum_secondary_gap,true,repeated);
  if(report.status!=Status::Ok)return report;
  if(repeated!=total)return {Status::InvalidInput};
  // Exact I25REMNOR count/scan/fill: expanded-main order, then native node-list
  // order. Output entries retain one-based local main IDs.
  for(std::size_t i=0;i<total;++i)++data.secondary_offsets[w.secondary_index[data.removed_nodes[i]]+1];
  for(std::size_t s=0;s<in.secondary_count;++s) {
    data.secondary_offsets[s+1]+=data.secondary_offsets[s];w.cursors[s]=data.secondary_offsets[s];
  }
  for(std::size_t m=0;m<in.main_count;++m)
    for(auto j=data.main_offsets[m];j<data.main_offsets[m+1];++j) {
      const auto s=w.secondary_index[data.removed_nodes[j]];
      data.removed_mains[w.cursors[s]++]=static_cast<std::uint32_t>(m+1);
    }
  return {Status::Ok,SIZE_MAX,SIZE_MAX,total,true};
}
} // namespace tlfea::contact::radioss_type25::search_startup::detail
