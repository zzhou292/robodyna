// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace normal_activation_test {
extern "C" void na_masks(const int*,const int*,const int*,const int*,const int*,const double*,const double*,
    const int*,const int*,const int*,const int*,const int*,const int*,int*,int*,int*,int*);
namespace {
void Need(bool value){if(!value)throw std::invalid_argument("Native activation reference input exceeds its declared domain");}
std::vector<int> Offsets(const activation::lifecycle::Csr& csr,std::size_t rows,std::size_t mains) {
  Need(csr.offsets&&csr.offset_count==rows+1&&csr.entry_count<=32768);
  Need(csr.offsets[0]==0&&csr.offsets[rows]==csr.entry_count&&(!csr.entry_count||csr.entries));
  std::vector<int> result(rows+1);
  for(std::size_t i=0;i<=rows;++i){Need(csr.offsets[i]<=csr.entry_count&&(i==0||csr.offsets[i]>=csr.offsets[i-1]));result[i]=int(csr.offsets[i]);}
  for(std::size_t i=0;i<csr.entry_count;++i)Need(csr.entries[i]>0&&csr.entries[i]<=mains);
  return result;
}
std::vector<int> Entries(const activation::lifecycle::Csr& csr) {
  std::vector<int> out(std::max(std::size_t{1},csr.entry_count));
  for(std::size_t i=0;i<csr.entry_count;++i)out[i]=int(csr.entries[i]);return out;
}
}
Result Oracle(const activation::Input& input) {
  const auto& s=input.source;const auto& p=input.profile;
  const auto n=s.node_count,g=s.main_count,ns=s.secondary_count,nr=s.normal_count;
  Need(p.edge_mode==0&&p.foreign_rows==0&&p.partitions==1&&p.neighbor_removal==2&&p.local_processor==1&&
      p.free_roster==activation::FreeRosterPolicy::FreshComplete);
  Need(n&&n<=4096&&g&&g<=4096&&nr&&nr<=16384&&ns<=4096&&input.row_count==ns);
  Need(s.mains&&s.nodes&&(!ns||(s.secondary&&input.rows))&&input.optimized_count<=32768&&(!input.optimized_count||input.optimized_main_ids));
  auto normal_offsets=Offsets(s.normal_to_main,nr,g),removal_offsets=Offsets(s.removed_main_by_secondary,ns,g);
  auto normal_mains=Entries(s.normal_to_main),removed_mains=Entries(s.removed_main_by_secondary);
  const int counts[]{int(n),int(g),int(ns),int(nr),int(s.normal_to_main.entry_count),int(s.removed_main_by_secondary.entry_count),int(input.optimized_count)};
  std::vector<int> irect(4*g),role(g),neighbor(4*g),admsr(4*g),history(4*std::max(ns,std::size_t{1}));
  std::vector<double> main_stiffness(g),secondary_stiffness(std::max(ns,std::size_t{1}));
  for(std::size_t i=0;i<g;++i) {
    const auto& main=s.mains[i];Need(std::isfinite(main.coefficient));main_stiffness[i]=main.coefficient;role[i]=main.segment_type;
    Need(std::int64_t(role[i])>=-2*std::int64_t(g)&&std::int64_t(role[i])<=2*std::int64_t(g));
    for(unsigned k=0;k<4;++k) {
      Need(main.nodes[k]<n&&main.normal_reference[k]>0&&std::size_t(main.normal_reference[k])<=nr&&main.neighbors[k]>=0&&std::size_t(main.neighbors[k])<=g);
      irect[4*i+k]=int(main.nodes[k]+1);admsr[4*i+k]=main.normal_reference[k];neighbor[4*i+k]=main.neighbors[k];
    }
  }
  for(std::size_t i=0;i<ns;++i) {
    Need(std::isfinite(s.secondary[i].coefficient));secondary_stiffness[i]=s.secondary[i].coefficient;
    const auto& row=input.rows[i].stage.value.history.row;
    for(unsigned k=0;k<4;++k)history[4*i+k]=row.irtlm[k];
    if(row.irtlm[0]>0&&secondary_stiffness[i]!=0&&row.irtlm[3]==1)Need(row.irtlm[2]>0&&std::size_t(row.irtlm[2])<=g);
  }
  std::vector<int> optimized(std::max(input.optimized_count,std::size_t{1}));
  for(std::size_t i=0;i<input.optimized_count;++i){Need(input.optimized_main_ids[i]>0&&input.optimized_main_ids[i]<=g);optimized[i]=int(input.optimized_main_ids[i]);}
  std::vector<int> active(g),tags(n),free_ids(g);int free_count=0;
  na_masks(counts,irect.data(),role.data(),neighbor.data(),admsr.data(),main_stiffness.data(),secondary_stiffness.data(),
      history.data(),normal_offsets.data(),normal_mains.data(),removal_offsets.data(),removed_mains.data(),optimized.data(),
      active.data(),tags.data(),&free_count,free_ids.data());
  Need(free_count>=0&&std::size_t(free_count)<=g);
  for(int v:active)Need(v==0||v==1);for(int v:tags)Need(v==0||v==1);
  Result out;out.main_active.assign(active.begin(),active.end());out.node_tag.assign(tags.begin(),tags.end());
  out.free_main_ids.assign(free_ids.begin(),free_ids.begin()+free_count);return out;
}
} // namespace normal_activation_test
