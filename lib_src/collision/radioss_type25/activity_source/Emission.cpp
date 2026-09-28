// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::activity_source::detail {
namespace {
bool Emitter(Family family){return family==Family::Qeph||family==Family::T3;}
unsigned Corners(const std::uint32_t* nodes){return nodes[2]==nodes[3]?3:4;}
struct ScratchLayout {
  tl::util::ArenaRegion offsets,cursor,mains;
  std::size_t bytes=0,incidence=0;
};
TransactionReport MakeEmissionLayout(Source source,std::size_t nodes,std::size_t cap,ScratchLayout& out) {
  for(std::size_t i=0;i<source.mains();++i) {
    const auto additional=Corners(source.nodes(i));
    if(out.incidence>UINT32_MAX-additional)return Fail(S::ResourceLimit,"Registered-main incidence overflows");
    out.incidence+=additional;
  }
  tl::util::BoundedArenaLayout layout(cap);
  if(!layout.Append<std::uint32_t>(nodes+1,out.offsets)||!layout.Append<std::uint32_t>(nodes,out.cursor)||
      !layout.Append<std::uint32_t>(out.incidence,out.mains))
    return Fail(S::ResourceLimit,"Complete registered-main discovery scratch exceeds cap");
  out.bytes=layout.bytes();return Ok();
}
struct Counter { const std::uint32_t* degree;std::size_t cap,used=0;bool exhausted=false; };
bool Count(void* opaque,const ParentRow& row) {
  auto& counter=*static_cast<Counter*>(opaque);if(!Emitter(row.identity.family))return true;
  const auto additional=counter.degree[row.nodes[0]];
  if(additional>counter.cap-counter.used){counter.exhausted=true;return false;}
  counter.used+=additional;return true;
}
struct Writer {
  Source source;const std::uint32_t* node_offsets;const std::uint32_t* registered_mains;
  std::uint32_t* offsets;std::uint32_t* mains;std::size_t capacity,parent=0,used=0;
  bool exhausted=false;
};
bool Write(void* opaque,const ParentRow& row) {
  auto& out=*static_cast<Writer*>(opaque);
  if(Emitter(row.identity.family)) {
    const auto node=row.nodes[0];
    for(auto at=out.node_offsets[node];at<out.node_offsets[node+1];++at) {
      const auto main=out.registered_mains[at];const auto* nodes=out.source.nodes(main-1);
      bool contains=true;
      for(unsigned k=0;k<row.count;++k)
        contains=contains&&std::find(nodes,nodes+Corners(nodes),row.nodes[k])!=nodes+Corners(nodes);
      if(!contains)continue;
      if(out.used==out.capacity){out.exhausted=true;return false;}
      out.mains[out.used++]=main;
    }
  }
  out.offsets[++out.parent]=static_cast<std::uint32_t>(out.used);return true;
}
}
TransactionReport ForecastEmissions(PhysicalSources physical,Source source,Counts& counts,
    Limits limits,std::size_t& scratch_bytes) noexcept {
  ScratchLayout layout;auto report=MakeEmissionLayout(source,counts.nodes,limits.startup_bytes,layout);
  if(report.status!=S::Ok)return report;
  tl::util::HostArena arena;
  if(!arena.Initialize(counts.nodes*sizeof(std::uint32_t)))return Fail(S::ResourceLimit,"Emission count allocation failed");
  auto* degree=static_cast<std::uint32_t*>(arena.data());std::fill_n(degree,counts.nodes,0);
  for(std::size_t i=0;i<counts.mains;++i) {
    const auto* nodes=source.nodes(i);
    for(unsigned k=0;k<Corners(nodes);++k)++degree[nodes[k]];
  }
  Counter counter{degree,limits.emitting_mains};Counts observed;
  report=Parents(physical,observed,&counter,Count);
  if(counter.exhausted)return Fail(S::ResourceLimit,"Complete emitting-parent discovery capacity exceeds cap");
  if(report.status!=S::Ok)return report;
  counts.emitting_capacity=counter.used;scratch_bytes=layout.bytes+256;return Ok();
}
TransactionReport Emit(PhysicalSources physical,Source source,const Counts& counts,Limits limits,
    std::uint32_t* offsets,std::uint32_t* mains,std::size_t& used) noexcept {
  ScratchLayout layout;auto report=MakeEmissionLayout(source,counts.nodes,limits.startup_bytes,layout);
  if(report.status!=S::Ok)return report;
  tl::util::HostArena arena;
  if(!arena.Initialize(layout.bytes))return Fail(S::ResourceLimit,"Registered-main discovery allocation failed");
  auto* node_offsets=arena.Construct<std::uint32_t>(layout.offsets);
  auto* cursor=arena.Construct<std::uint32_t>(layout.cursor);
  auto* registered=arena.Construct<std::uint32_t>(layout.mains);
  for(std::size_t main=0;main<counts.mains;++main) {
    const auto* nodes=source.nodes(main);
    for(unsigned k=0;k<Corners(nodes);++k)++node_offsets[nodes[k]+1];
  }
  for(std::size_t i=1;i<=counts.nodes;++i)node_offsets[i]+=node_offsets[i-1];
  std::copy_n(node_offsets,counts.nodes,cursor);
  // INIT_NODAL_STATE registers every expanded main in order and skips a
  // repeated T3 fourth corner. Each per-node list is therefore a sorted set.
  for(std::size_t main=0;main<counts.mains;++main) {
    const auto* nodes=source.nodes(main);
    for(unsigned k=0;k<Corners(nodes);++k)registered[cursor[nodes[k]]++]=static_cast<std::uint32_t>(main+1);
  }
  offsets[0]=0;Writer writer{source,node_offsets,registered,offsets,mains,counts.emitting_capacity};Counts observed;
  report=Parents(physical,observed,&writer,Write);
  if(writer.exhausted)return Fail(S::ResourceLimit,"Complete emitting-parent discovery exceeds admitted capacity");
  if(report.status!=S::Ok)return report;
  if(writer.parent!=counts.parents)return Fail(S::SourceMismatch,"Emission parent roster changed during construction");
  used=writer.used;return Ok();
}
}
