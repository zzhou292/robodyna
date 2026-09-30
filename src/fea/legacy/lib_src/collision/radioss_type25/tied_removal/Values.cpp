// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
#include <climits>
namespace tlfea::contact::radioss_type25::tied_removal::detail {
Report Evaluate(const Input& in,Limits limits,const Layout& layout,Work w,Data out) noexcept {
  const auto& old=in.geometric.geometry;
  const auto g=in.source.main_count,s=in.source.secondary_count;
  const auto capacity=layout.forecast.removal_capacity;
  std::size_t visits=0,total=0,added=0;
  out.main_offsets[0]=0;
  for(std::size_t m=0;m<g;++m) {
    const auto first=old.main_offsets[m],last=old.main_offsets[m+1];
    for(std::size_t j=first;j<last;++j)w.seen[old.removed_nodes[j]]=static_cast<std::uint32_t>(m);
    std::size_t discovered=0;
    const auto append=[&](std::uint32_t node) {
      if(visits==limits.max_relation_visits)return false;
      ++visits;
      if(w.secondary[node]!=UINT32_MAX&&w.seen[node]!=m) {
        w.seen[node]=static_cast<std::uint32_t>(m);w.discovered[discovered++]=node;
      }
      return true;
    };
    const auto& main=in.source.topology.mains[m];
    const unsigned corners=main.nodes[2]==main.nodes[3]?3:4;
    for(unsigned k=0;k<corners;++k) {
      const auto node=main.nodes[k];
      for(std::size_t q=w.offsets[node];q<w.offsets[node+1];++q) {
        if(visits==limits.max_relation_visits)return {Status::ResourceLimit,SIZE_MAX,SIZE_MAX,m};
        ++visits;const auto relation=w.relations[q];
        const auto& f=in.interfaces[relation.interface];const auto& row=f.rows[relation.row];
        if(!relation.secondary) {
          if(!append(row.node))return {Status::ResourceLimit,SIZE_MAX,SIZE_MAX,m};
        } else {
          const auto& tied_main=f.mains[row.local_main-1];
          for(unsigned c=0;c<Corners(tied_main);++c)
            if(!append(tied_main.nodes[c]))return {Status::ResourceLimit,SIZE_MAX,SIZE_MAX,m};
        }
      }
    }
    const auto count=std::size_t(last-first)+discovered;
    if(count>std::size_t(INT_MAX)-total)return {Status::ResourceLimit,SIZE_MAX,SIZE_MAX,m};
    // Native insertion address is after the old entries of this main; only
    // the new per-main discovery sequence is reversed. Never sort by node ID.
    if(total+count<=capacity) {
      if(last>first)std::copy_n(old.removed_nodes+first,last-first,out.nodes+total);
      for(std::size_t j=0;j<discovered;++j)out.nodes[total+(last-first)+j]=w.discovered[discovered-1-j];
    }
    total+=count;added+=discovered;out.main_offsets[m+1]=static_cast<std::uint32_t>(total);
  }
  Report report{Status::Ok,SIZE_MAX,SIZE_MAX,SIZE_MAX,total,added,0,true};
  if(total>capacity||added>std::size_t(INT_MAX)-in.native_removal_extent) {
    report.status=Status::ResourceLimit;return report;
  }
  std::copy_n(in.history,s,out.history);
  if(!added) {
    // Native does not call UPGRADE_REMNODE2 or perform its history reset when
    // NNREM(new additions)==0. Keep even an already-forbidden retained marker.
    std::copy_n(old.secondary_offsets,s+1,out.secondary_offsets);
    if(total)std::copy_n(old.removed_mains,total,out.mains);
    return report;
  }
  // UPGRADE_REMNODE2 clears the inverse before REMN_I2OP transposes all used
  // entries. Padding from native allocated REMNODE is not part of this loop.
  for(std::size_t i=0;i<total;++i)++out.secondary_offsets[w.secondary[out.nodes[i]]+1];
  for(std::size_t i=0;i<s;++i) {
    out.secondary_offsets[i+1]+=out.secondary_offsets[i];w.cursors[i]=out.secondary_offsets[i];
  }
  for(std::size_t m=0;m<g;++m)for(std::size_t i=out.main_offsets[m];i<out.main_offsets[m+1];++i)
    out.mains[w.cursors[w.secondary[out.nodes[i]]]++]=static_cast<std::uint32_t>(m+1);
  for(std::size_t row=0;row<s;++row)for(std::size_t j=out.secondary_offsets[row];j<out.secondary_offsets[row+1];++j) {
    const auto main=out.mains[j]-1;
    if(out.history[row].irtlm[0]==in.source.topology.mains[main].global_id) {
      // Literal comparison: a negative stored marker is never converted to abs.
      out.history[row]=History{};++report.reset_rows;break;
    }
  }
  return report;
}
} // namespace tlfea::contact::radioss_type25::tied_removal::detail
