// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <cstdint>
#include <limits>
#include <stdexcept>
namespace type25_current_normals_test {
extern "C" void rd_current_main_normals(const int*,const double*,const int*,const int*,const int*,const int*,
    const int*,const double*,const int*,const int*,const int*,const float*,float*,float*,int*,float*,int*,int*,int*,int*);
namespace {
void Need(bool value){if(!value)throw std::invalid_argument("Unsupported or malformed serial current-normal native packet");}
template<class T> bool Span(const T* ptr,std::size_t count) {
  if(count>SIZE_MAX/sizeof(T))return false;
  const auto address=reinterpret_cast<std::uintptr_t>(ptr),bytes=count*sizeof(T);
  return (!count||ptr)&&address%alignof(T)==0&&address<=UINTPTR_MAX-bytes;
}
bool Finite(n::StoredNormal v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
}
NativeResult Oracle(const c::Input& in) {
  static_assert(sizeof(int)==4&&sizeof(float)==4&&sizeof(double)==8&&std::numeric_limits<float>::is_iec559);
  const auto& t=in.topology;const auto N=t.nodes,P=t.primary_count,G=t.main_count,R=t.references;
  const bool resolved=in.profile==c::Profile::ResolvedShellSidesLocal;
  Need((resolved||in.profile==c::Profile::OrdinaryShellLocal)&&in.free_roster==n::normal_activation::FreeRosterPolicy::FreshComplete);
  Need(N&&N<=256&&P&&P<=160&&G==2*P&&R&&R<=4*G&&in.free_count<=G);
  if(resolved) {
    Need(t.source_profile==n::startup::Profile::ResolvedShellSides&&t.source_topology==n::startup::TopologyPolicy::NativeResolvedShellSides);
    Need(t.primary_role_count==P&&Span(t.primary_roles,P));
    for(std::size_t i=0;i<P;++i)Need(t.primary_roles[i]==n::startup::ShellSideRole::Ordinary||
        t.primary_roles[i]==n::startup::ShellSideRole::CoatingForward||t.primary_roles[i]==n::startup::ShellSideRole::CoatingReversed);
  } else Need(!t.primary_roles&&!t.primary_role_count);
  Need(in.coefficient_count==G&&in.active_count==G&&in.tag_count==N&&in.prior_count==4*G);
  Need(Span(t.mains,G)&&Span(in.main_coefficients,G)&&Span(in.main_active,G)&&Span(in.node_tag,N)&&
      Span(in.prior_normals,4*G)&&Span(in.free_main_ids,in.free_count));
  Need(in.positions.valid()&&in.positions.node_count==N);
  const auto last=(N-1)*in.positions.node_stride+2*in.positions.component_stride;
  Need(last<SIZE_MAX&&Span(in.positions.data,std::size_t(last)+1));
  const auto& csr=t.normal_to_main;
  Need(csr.offset_count==R+1&&csr.entry_count<=4*G&&Span(csr.offsets,R+1)&&Span(csr.entries,csr.entry_count));
  Need(csr.offsets&&csr.offsets[0]==0&&csr.offsets[R]==csr.entry_count);
  for(std::size_t r=0;r<R;++r)Need(csr.offsets[r]<=csr.offsets[r+1]&&csr.offsets[r+1]<=csr.entry_count);
  for(std::size_t i=0;i<csr.entry_count;++i)Need(csr.entries[i]>0&&csr.entries[i]<=G);
  double length=1;
  if(in.coordinates==n::startup::Coordinates::Si){length=in.units.length_m;Need(std::isfinite(length)&&length>0);}
  else Need(in.coordinates==n::startup::Coordinates::Native);
  std::vector<double> x(3*N);std::vector<int> connectivity(4*G),roles(G),neighbors(4*G),edges(4*G),references(4*G),active(G),tag(N),free(std::max<std::size_t>(1,in.free_count));
  std::vector<float> prior(12*G),first(12*G),last_normals(12*G),bisectors(6*R);
  std::vector<int> bound(R),skip(P),free_edges(16*G);
  for(std::size_t node=0;node<N;++node){const auto v=in.positions.at(std::uint32_t(node));
    x[3*node]=v.x/length;x[3*node+1]=v.y/length;x[3*node+2]=v.z/length;
    for(unsigned k=0;k<3;++k)Need(std::isfinite(x[3*node+k]));
    Need(in.node_tag[node]<=1);tag[node]=int(in.node_tag[node]);}
  for(std::size_t main=0;main<G;++main){const auto& m=t.mains[main];
    Need(std::int64_t(m.segment_type)>=-2*std::int64_t(G)&&std::int64_t(m.segment_type)<=2*std::int64_t(G));
    roles[main]=m.segment_type;Need(std::isfinite(in.main_coefficients[main])&&in.main_active[main]<=1);active[main]=int(in.main_active[main]);
    for(unsigned k=0;k<4;++k){Need(m.nodes[k]<N&&m.normal_reference[k]>0&&std::size_t(m.normal_reference[k])<=R&&
        m.neighbors[k]>=0&&std::size_t(m.neighbors[k])<=G&&(!m.neighbors[k]||(m.neighbor_edges[k]>0&&m.neighbor_edges[k]<=4)));
      const auto i=4*main+k;connectivity[i]=int(m.nodes[k]+1);references[i]=m.normal_reference[k];neighbors[i]=m.neighbors[k];edges[i]=m.neighbor_edges[k];
      const auto v=in.prior_normals[i];Need(Finite(v));prior[3*i]=v.x;prior[3*i+1]=v.y;prior[3*i+2]=v.z;}}
  for(std::size_t i=0;i<in.free_count;++i){Need(in.free_main_ids[i]>0&&in.free_main_ids[i]<=G);free[i]=int(in.free_main_ids[i]);}
  const int counts[]{int(N),int(P),int(G),int(R),int(in.free_count)};int edge_count=0,status=0;
  rd_current_main_normals(counts,x.data(),connectivity.data(),roles.data(),neighbors.data(),edges.data(),references.data(),
      in.main_coefficients,active.data(),tag.data(),free.data(),prior.data(),first.data(),last_normals.data(),bound.data(),
      bisectors.data(),skip.data(),free_edges.data(),&edge_count,&status);
  Need(status==0&&edge_count>=0&&std::size_t(edge_count)<=4*G);
  NativeResult out;out.flag1_normals.resize(4*G);out.normals.resize(4*G);out.references.resize(R);out.primary_skip=std::move(skip);
  for(std::size_t i=0;i<4*G;++i){out.flag1_normals[i]={first[3*i],first[3*i+1],first[3*i+2]};
    out.normals[i]={last_normals[3*i],last_normals[3*i+1],last_normals[3*i+2]};out.finite&=Finite(out.flag1_normals[i])&&Finite(out.normals[i]);}
  for(std::size_t i=0;i<R;++i){auto& r=out.references[i];r.boundary=bound[i];Need(bound[i]>=0);
    for(unsigned k=0;k<2;++k){const auto j=6*i+3*k;r.bisector[k]={bisectors[j],bisectors[j+1],bisectors[j+2]};out.finite&=Finite(r.bisector[k]);}}
  for(int i=0;i<edge_count;++i)out.free_edges.push_back({free_edges[4*i],free_edges[4*i+1],free_edges[4*i+2],free_edges[4*i+3]});
  return out;
}
}
