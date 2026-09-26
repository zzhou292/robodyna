// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../search_startup/Internal.h"
#include <climits>
namespace tlfea::contact::radioss_type25::tied_removal::detail {
namespace {
bool Disjoint(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept {
  if(!an||!bn)return true;
  const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
  return a&&b&&an<=UINTPTR_MAX-x&&bn<=UINTPTR_MAX-y&&(x+an<=y||y+bn<=x);
}
bool Same(const search_startup::Contributors& a,const search_startup::Contributors& b) noexcept {
  return a.census==b.census&&a.physical_nodes==b.physical_nodes&&a.physical_shells==b.physical_shells&&
      a.tied_interfaces==b.tied_interfaces&&a.rigid_bodies==b.rigid_bodies&&a.cin_links==b.cin_links&&
      a.other_interfaces==b.other_interfaces&&a.unsupported_elements==b.unsupported_elements&&
      a.native_auxiliary_nodes==b.native_auxiliary_nodes;
}
}
Report Admit(const Input& in,Limits limits,const Layout&,const tl::util::HostArena& out,
    const tl::util::HostArena& scratch,const Snapshot* result,bool mixed) noexcept {
  namespace r=search::detail;namespace ss=search_startup;
  std::size_t native_nodes=0;
  const auto context=ss::detail::ResolveContext(in.source,limits.search,mixed?ss::detail::Context::ComposedBeforeTied:ss::detail::Context::BeforeTied,native_nodes);
  if(context.status!=Status::Ok)return {context.status};
  const auto source=(mixed&&in.source.mesh.profile==startup::Profile::MixedSurface)?ss::detail::AdmitMixed(in.source,{},limits.search,out,scratch,result,sizeof(*result)):
      ss::detail::Admit(in.source,{},limits.search,out,scratch,result,sizeof(*result),mixed);
  if(source.status!=Status::Ok)return {source.status,SIZE_MAX,SIZE_MAX,source.main};
  if(in.finalization!=Finalization::CompactedAfterKinChk||in.tied_removal!=1)
    return {Status::UnsupportedProfile};
  const auto& g=in.geometric.geometry;
  const auto population=ss::detail::ResolvedPopulation(in.source,native_nodes);
  const bool exact=population.policy==ss::NativePopulationPolicy::ExactDeclaredAuxiliaryIds;
  const auto p=in.source.mesh.primary_count,m=in.source.main_count,s=in.source.secondary_count;
  if(in.geometric.covered_type25_siblings!=in.source.covered_type25_siblings||
      !Same(in.geometric.contributors,in.source.contributors)||in.interface_count!=in.source.contributors.tied_interfaces||
      g.primary_count!=p||g.main_count!=m||g.secondary_count!=s||g.source_generation!=in.source.mesh.source_generation||
      g.native_model_nodes_exact!=exact||g.native_model_nodes!=(exact?native_nodes:0)||
      g.native_population.policy!=population.policy||g.native_population.lower!=population.lower||g.native_population.upper!=population.upper||
      in.history_count!=s||g.removal_count>limits.search.max_removals||
      in.native_removal_extent<g.removal_count||in.native_removal_extent>std::size_t(INT_MAX)||
      !r::Span(g.primary_extent,p)||!r::Span(g.main_offsets,m+1)||!r::Span(g.secondary_offsets,s+1)||
      !r::Span(g.removed_nodes,g.removal_count)||!r::Span(g.removed_mains,g.removal_count)||
      !r::Span(g.initial_contact,s)||!r::Span(in.history,s)||!r::Span(result,1))return {Status::InvalidInput};
  const auto safe=[&](const void* read,std::size_t bytes) {
    return Disjoint(out.data(),out.bytes(),read,bytes)&&Disjoint(scratch.data(),scratch.bytes(),read,bytes)&&
        Disjoint(result,sizeof(*result),read,bytes);
  };
  if(!safe(&in,sizeof(in))||!safe(in.interfaces,in.interface_count*sizeof(Interface))||
      !safe(g.primary_extent,p*sizeof(double))||!safe(g.main_offsets,(m+1)*sizeof(std::uint32_t))||
      !safe(g.secondary_offsets,(s+1)*sizeof(std::uint32_t))||!safe(g.removed_nodes,g.removal_count*sizeof(std::uint32_t))||
      !safe(g.removed_mains,g.removal_count*sizeof(std::uint32_t))||!safe(g.initial_contact,s*sizeof(int))||
      !safe(in.history,s*sizeof(History)))return {Status::InvalidInput};
  for(std::size_t i=0;i<in.interface_count;++i) {
    const auto& f=in.interfaces[i];
    if(!safe(f.mains,f.main_count*sizeof(Main))||!safe(f.rows,f.row_count*sizeof(Row)))return {Status::InvalidInput,i};
    if(f.level!=28)return {Status::UnsupportedProfile,i};
    if(!f.source_id||f.source_id>std::uint64_t(INT_MAX)||!f.native_ordinal||f.native_ordinal>std::uint32_t(INT_MAX)||
        (i&&f.native_ordinal<=in.interfaces[i-1].native_ordinal))return {Status::InvalidInput,i};
  }
  for(double value:{g.multiplier,g.mean_length,g.margin,g.maximum_extent})
    if(!std::isfinite(value)||value<0)return {Status::InvalidInput};
  for(std::size_t i=0;i<p;++i)if(!std::isfinite(g.primary_extent[i])||g.primary_extent[i]<0)return {Status::InvalidInput};
  if(g.main_offsets[0]||g.secondary_offsets[0]||g.main_offsets[m]!=g.removal_count||g.secondary_offsets[s]!=g.removal_count)
    return {Status::InvalidInput};
  for(std::size_t i=0;i<m;++i)if(g.main_offsets[i]>g.main_offsets[i+1])return {Status::InvalidInput,SIZE_MAX,SIZE_MAX,i};
  for(std::size_t i=0;i<s;++i)if(g.secondary_offsets[i]>g.secondary_offsets[i+1])return {Status::InvalidInput,SIZE_MAX,i};
  return {Status::Ok};
}
Report Prepare(const Input& in,Work w) noexcept {
  search_startup::detail::Work auxiliary{};auxiliary.auxiliary_ids=w.auxiliary_ids;
  const auto context=search_startup::detail::CheckAuxiliaryIds(in.source,auxiliary);
  if(context.status!=Status::Ok)return {context.status};
  const auto n=in.source.mesh.node_count,s=in.source.secondary_count,g=in.source.main_count;
  std::fill_n(w.secondary,n,UINT32_MAX);std::fill_n(w.seen,n,UINT32_MAX);
  for(std::size_t i=0;i<s;++i) {
    const auto node=in.source.secondary[i].node;
    if(node>=n||w.secondary[node]!=UINT32_MAX)return {Status::InvalidInput,SIZE_MAX,i};
    w.secondary[node]=static_cast<std::uint32_t>(i);
    w.cursors[i]=in.geometric.geometry.secondary_offsets[i];
  }
  const auto& old=in.geometric.geometry;
  // Authenticate both complete CSR directions, including exact ascending-main
  // inverse order. No invalid padding is inspected.
  for(std::size_t m=0;m<g;++m)for(std::size_t j=old.main_offsets[m];j<old.main_offsets[m+1];++j) {
    const auto node=old.removed_nodes[j];
    if(node>=n||w.secondary[node]==UINT32_MAX||w.seen[node]==m)return {Status::InvalidInput,SIZE_MAX,SIZE_MAX,m};
    w.seen[node]=static_cast<std::uint32_t>(m);const auto row=w.secondary[node];
    if(w.cursors[row]>=old.secondary_offsets[row+1]||old.removed_mains[w.cursors[row]++]!=m+1)
      return {Status::InvalidInput,SIZE_MAX,row,m};
  }
  for(std::size_t i=0;i<s;++i)if(w.cursors[i]!=old.secondary_offsets[i+1])return {Status::InvalidInput,SIZE_MAX,i};
  std::fill_n(w.seen,n,UINT32_MAX);
  std::size_t cin=0;
  for(std::size_t i=0;i<in.interface_count;++i) {
    const auto& f=in.interfaces[i];w.ids[i]=f.source_id;
    for(std::size_t m=0;m<f.main_count;++m) {
      const auto& a=f.mains[m].nodes;
      if(a[0]>=n||a[1]>=n||a[2]>=n||a[3]>=n||a[0]==a[1]||a[0]==a[2]||a[1]==a[2]||a[3]==a[0]||a[3]==a[1])
        return {Status::InvalidInput,i,SIZE_MAX,m};
    }
    for(std::size_t j=0;j<f.row_count;++j) {
      const auto& row=f.rows[j];
      if(row.node>=n||row.local_main<=0||std::size_t(row.local_main)>f.main_count||w.seen[row.node]==i)
        return {Status::InvalidInput,i,j};
      w.seen[row.node]=static_cast<std::uint32_t>(i);cin+=row.irupt==0;
      const auto& main=f.mains[row.local_main-1];
      for(unsigned k=0;k<Corners(main);++k)++w.offsets[main.nodes[k]+1];
      ++w.offsets[row.node+1];
    }
  }
  std::sort(w.ids,w.ids+in.interface_count);
  for(std::size_t i=1;i<in.interface_count;++i)if(w.ids[i]==w.ids[i-1])return {Status::InvalidInput};
  if(cin!=in.source.contributors.cin_links)return {Status::InvalidInput};
  for(std::size_t i=0;i<n;++i) {w.offsets[i+1]+=w.offsets[i];w.cursors[i]=w.offsets[i];}
  // Counting scatter is the stable node-only MY_ORDERS grouping. Within each
  // node the native interface, secondary row and corner insertion order stays.
  for(std::size_t i=0;i<in.interface_count;++i)for(std::size_t j=0;j<in.interfaces[i].row_count;++j) {
    const auto& row=in.interfaces[i].rows[j];const auto& main=in.interfaces[i].mains[row.local_main-1];
    const Relation positive{static_cast<std::uint32_t>(i),static_cast<std::uint32_t>(j),false};
    for(unsigned k=0;k<Corners(main);++k)w.relations[w.cursors[main.nodes[k]]++]=positive;
    w.relations[w.cursors[row.node]++]={positive.interface,positive.row,true};
  }
  std::fill_n(w.seen,n,UINT32_MAX);return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::tied_removal::detail
