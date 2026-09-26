// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <array>
#include <climits>
#include <mutex>
#include <stdexcept>
namespace type25_tied_removal_test {
extern "C" void type25_tied_removal_reference(const int*,const int*,const int*,const int*,const int*,const int*,
 const int*,const int*,const int*,const int*,const int*,const double*,int*,int*,int*,int*,int*,double*,int*);
namespace {
void Require(bool value) {if(!value)throw std::runtime_error("Native tied-removal boundary admission");}
template<class T>void Span(const T* p,std::size_t n) {
  Require(n==0?p==nullptr:p&&reinterpret_cast<std::uintptr_t>(p)%alignof(T)==0&&n<=SIZE_MAX/sizeof(T)&&
    n*sizeof(T)<=UINTPTR_MAX-reinterpret_cast<std::uintptr_t>(p));
}
}
NativeResult Oracle(const t::Input& in) {
  static std::mutex mutex;const std::lock_guard<std::mutex> lock(mutex);
  const auto& source=in.source;const auto& old=in.geometric.geometry;
  const auto n=source.mesh.node_count,g=source.main_count,s=source.secondary_count;
  Require(n>0&&n<=256&&g>0&&g<=128&&s>0&&s<=128&&in.interface_count>0&&in.interface_count<=16&&
    old.native_model_nodes>=n&&old.native_model_nodes<=256&&in.native_removal_extent>=old.removal_count&&
    in.native_removal_extent<=16384&&in.history_count==s);
  Span(in.interfaces,in.interface_count);Span(source.topology.mains,g);Span(source.secondary,s);Span(in.history,s);
  Span(old.main_offsets,g+1);Span(old.secondary_offsets,s+1);Span(old.removed_nodes,old.removal_count);Span(old.removed_mains,old.removal_count);
  Require(old.main_offsets[0]==0&&old.secondary_offsets[0]==0&&old.main_offsets[g]==old.removal_count&&old.secondary_offsets[s]==old.removal_count);
  std::vector<int> interfaces,mains,rows,target,secondary,ko(g+1),rn,kn(s+1),rm,history(4*s);
  std::vector<double> values(7*s);
  std::size_t tm=0,tr=0;
  for(std::size_t i=0;i<in.interface_count;++i) {
    const auto& f=in.interfaces[i];Require(f.main_count<=128-tm&&f.row_count<=128-tr&&f.native_ordinal>0&&f.native_ordinal<=32&&
      (!i||f.native_ordinal>in.interfaces[i-1].native_ordinal)&&f.source_id>0&&f.source_id<=INT_MAX&&f.level==28);
    Span(f.mains,f.main_count);Span(f.rows,f.row_count);tm+=f.main_count;tr+=f.row_count;
    interfaces.insert(interfaces.end(),{int(f.native_ordinal),int(f.source_id),int(f.main_count),int(f.row_count)});
    for(std::size_t m=0;m<f.main_count;++m)for(auto node:f.mains[m].nodes) {Require(node<n);mains.push_back(int(node+1));}
    for(std::size_t r=0;r<f.row_count;++r) {Require(f.rows[r].node<n&&f.rows[r].local_main>0&&std::size_t(f.rows[r].local_main)<=f.main_count);
      rows.insert(rows.end(),{int(f.rows[r].node+1),f.rows[r].local_main});}
  }
  for(std::size_t m=0;m<g;++m) {
    for(auto node:source.topology.mains[m].nodes) {Require(node<n);target.push_back(int(node+1));}
    target.push_back(source.topology.mains[m].global_id);
    Require(old.main_offsets[m]<=old.main_offsets[m+1]&&old.main_offsets[m+1]<=old.removal_count);ko[m]=int(old.main_offsets[m]);
  }
  ko[g]=int(old.removal_count);kn[s]=int(old.removal_count);
  for(std::size_t r=0;r<s;++r) {
    Require(source.secondary[r].node<n&&old.secondary_offsets[r]<=old.secondary_offsets[r+1]&&old.secondary_offsets[r+1]<=old.removal_count);
    secondary.push_back(int(source.secondary[r].node+1));kn[r]=int(old.secondary_offsets[r]);
    for(std::size_t prior=0;prior<r;++prior)Require(source.secondary[prior].node!=source.secondary[r].node);
    for(unsigned k=0;k<4;++k)history[4*r+k]=in.history[r].irtlm[k];
    for(unsigned k=0;k<5;++k)values[7*r+k]=in.history[r].penetration[k];
    for(unsigned k=0;k<2;++k)values[7*r+5+k]=in.history[r].time[k];
  }
  for(std::size_t i=0;i<old.removal_count;++i) {
    Require(old.removed_nodes[i]<n&&old.removed_mains[i]>0&&old.removed_mains[i]<=g);
    rn.push_back(int(old.removed_nodes[i]+1));rm.push_back(int(old.removed_mains[i]));
    bool present=false;for(std::size_t r=0;r<s;++r)present|=source.secondary[r].node==old.removed_nodes[i];Require(present);
  }
  for(std::size_t m=0;m<g;++m)for(std::size_t j=old.main_offsets[m];j<old.main_offsets[m+1];++j)
    for(std::size_t k=old.main_offsets[m];k<j;++k)Require(old.removed_nodes[j]!=old.removed_nodes[k]);
  const int counts[]{int(old.native_model_nodes),int(g),int(s),int(in.interface_count),int(tm),int(tr),int(in.native_removal_extent)};
  std::vector<int> out_k(g+1),out_n(g*s),out_s(s+1),out_m(g*s),out_h(4*s);std::vector<double> out_v(7*s);int info[3]{};
  type25_tied_removal_reference(counts,interfaces.data(),mains.data(),rows.data(),target.data(),secondary.data(),
    ko.data(),rn.data(),kn.data(),rm.data(),history.data(),values.data(),out_k.data(),out_n.data(),out_s.data(),out_m.data(),out_h.data(),out_v.data(),info);
  Require(info[0]>=0&&std::size_t(info[0])<=g*s&&info[1]>=info[0]);
  NativeResult result;result.main_offsets.assign(out_k.begin(),out_k.end());result.secondary_offsets.assign(out_s.begin(),out_s.end());
  for(int i=0;i<info[0];++i) {Require(out_n[i]>0&&std::size_t(out_n[i])<=n);result.nodes.push_back(out_n[i]-1);result.mains.push_back(out_m[i]);}
  result.native_extent=std::size_t(info[1]);result.history.resize(s);
  for(std::size_t r=0;r<s;++r) {
    for(unsigned k=0;k<4;++k)result.history[r].irtlm[k]=out_h[4*r+k];
    for(unsigned k=0;k<5;++k)result.history[r].penetration[k]=out_v[7*r+k];
    for(unsigned k=0;k<2;++k)result.history[r].time[k]=out_v[7*r+5+k];
  }
  return result;
}
}
