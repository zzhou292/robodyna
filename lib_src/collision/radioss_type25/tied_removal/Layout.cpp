// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
#include <climits>
namespace tlfea::contact::radioss_type25::tied_removal::detail {
namespace {
bool Append(tl::util::BoundedArenaLayout& a,std::size_t p,std::size_t g,std::size_t s,
    std::size_t cap,DataLayout& out) noexcept {
  return a.Append<double>(p,out.extent)&&a.Append<std::uint32_t>(g+1,out.main_offsets)&&
      a.Append<std::uint32_t>(s+1,out.secondary_offsets)&&a.Append<std::uint32_t>(cap,out.nodes)&&
      a.Append<std::uint32_t>(cap,out.mains)&&a.Append<int>(s,out.contact)&&a.Append<History>(s,out.history);
}
}
Report Plan(const Input& in,Limits limits,Layout& out,bool mixed) noexcept {
  namespace r=search::detail;
  const Limits hard;
  const auto n=in.source.mesh.node_count,p=in.source.mesh.primary_count,s=in.source.secondary_count;
  const bool rich=mixed&&in.source.mesh.profile==startup::Profile::MixedSurface;
  const auto g=mixed?in.source.main_count:2*p;
  if(rich&&(in.source.mesh.topology!=startup::TopologyPolicy::NativeMixedSurface||
      in.source.mesh.shell_primary_count>p||g!=p+in.source.mesh.shell_primary_count))return {Status::UnsupportedProfile};
  if(mixed&&!rich&&g!=2*p)return {Status::InvalidInput};
  if(!n||!p||!s||!in.interface_count)return {Status::InvalidInput};
  if(n>limits.search.max_nodes||n>hard.search.max_nodes||(!mixed&&p>hard.search.max_mains/2)||g>hard.search.max_mains||
      g>limits.search.max_mains||s>limits.search.max_secondaries||s>hard.search.max_secondaries||
      in.interface_count>limits.max_interfaces||in.interface_count>hard.max_interfaces||
      limits.max_tied_mains>hard.max_tied_mains||limits.max_tied_rows>hard.max_tied_rows||
      limits.max_relations>hard.max_relations||limits.max_relation_visits>hard.max_relation_visits||
      !limits.max_relation_visits||limits.search.max_removals>hard.search.max_removals||
      !limits.search.max_output_bytes||!limits.search.max_scratch_bytes||
      in.source.auxiliary_rigid_primary_count>hard.search.max_native_model_nodes)return {Status::ResourceLimit};
  if(!r::Span(in.interfaces,in.interface_count))return {Status::InvalidInput};
  std::size_t rows=0,mains=0;
  for(std::size_t i=0;i<in.interface_count;++i) {
    const auto& f=in.interfaces[i];
    if(f.main_count>limits.max_tied_mains-mains||f.row_count>limits.max_tied_rows-rows)
      return {Status::ResourceLimit,i};
    mains+=f.main_count;rows+=f.row_count;
    if(!r::Span(f.mains,f.main_count)||!r::Span(f.rows,f.row_count))return {Status::InvalidInput,i};
  }
  if(rows>limits.max_relations/5)return {Status::ResourceLimit};
  if(g>SIZE_MAX/s)return {Status::ResourceLimit};
  const auto capacity=std::min(g*s,limits.search.max_removals),relations=5*rows;
  Layout next;tl::util::BoundedArenaLayout output(SIZE_MAX),scratch(SIZE_MAX);
  if(!Append(output,p,g,s,capacity,next.output)||!Append(scratch,p,g,s,capacity,next.staged)||
      !scratch.Append<std::uint32_t>(n+1,next.offsets)||!scratch.Append<std::uint32_t>(n,next.cursors)||
      !scratch.Append<std::uint32_t>(n,next.secondary)||!scratch.Append<std::uint32_t>(n,next.seen)||
      !scratch.Append<std::uint32_t>(s,next.discovered)||!scratch.Append<Relation>(relations,next.relations)||
      !scratch.Append<std::uint64_t>(in.interface_count,next.ids)||
      !scratch.Append<std::uint64_t>(in.source.auxiliary_rigid_primary_count,next.auxiliary_ids))return {Status::ResourceLimit};
  next.forecast={Status::Ok,output.bytes(),scratch.bytes(),capacity,relations};
  if(output.bytes()>limits.search.max_output_bytes||scratch.bytes()>limits.search.max_scratch_bytes)
    next.forecast.status=Status::ResourceLimit;
  out=next;return {next.forecast.status};
}
Data Construct(tl::util::HostArena& arena,const DataLayout& l) noexcept {
  return {arena.Construct<double>(l.extent),arena.Construct<std::uint32_t>(l.main_offsets),
      arena.Construct<std::uint32_t>(l.secondary_offsets),arena.Construct<std::uint32_t>(l.nodes),
      arena.Construct<std::uint32_t>(l.mains),arena.Construct<int>(l.contact),arena.Construct<History>(l.history)};
}
Work ConstructWork(tl::util::HostArena& arena,const Layout& l) noexcept {
  return {arena.Construct<std::uint32_t>(l.offsets),arena.Construct<std::uint32_t>(l.cursors),
      arena.Construct<std::uint32_t>(l.secondary),arena.Construct<std::uint32_t>(l.seen),
      arena.Construct<std::uint32_t>(l.discovered),arena.Construct<Relation>(l.relations),arena.Construct<std::uint64_t>(l.ids),
      arena.Construct<std::uint64_t>(l.auxiliary_ids)};
}
}
namespace tlfea::contact::radioss_type25::tied_removal {
Forecast Preflight(const Input& in,Limits limits) noexcept {
  detail::Layout layout;const auto result=detail::Plan(in,limits,layout);
  if(layout.forecast.output_bytes)return layout.forecast;
  Forecast out;out.status=result.status;return out;
}
Forecast PreflightComposed(const Input& in,Limits limits) noexcept {
  detail::Layout layout;const auto result=detail::Plan(in,limits,layout,true);
  if(layout.forecast.output_bytes)return layout.forecast;
  Forecast out;out.status=result.status;return out;
}

}
