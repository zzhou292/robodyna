// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
#include <new>
namespace tlfea::contact::radioss_type25::activity_source::detail {
namespace {
bool CountNodes(void* opaque,const ParentRow& row) {
  auto* counts=static_cast<std::uint32_t*>(opaque);
  for(unsigned i=0;i<row.count;++i)++counts[row.nodes[i]];
  return true;
}
struct WriteContext {
  ParentIdentity* parents;std::uint32_t* cursor;std::uint32_t* entries;std::uint32_t next=0;
};
bool WriteParent(void* opaque,const ParentRow& row) {
  auto& out=*static_cast<WriteContext*>(opaque);out.parents[out.next]=row.identity;
  for(unsigned i=0;i<row.count;++i)out.entries[out.cursor[row.nodes[i]]++]=out.next;
  ++out.next;return true;
}
bool Contains(const std::uint32_t* offsets,const std::uint32_t* parents,
    const std::uint32_t* nodes,unsigned n,std::uint32_t parent) {
  for(unsigned k=0;k<n;++k)
    if(!std::binary_search(parents+offsets[nodes[k]],parents+offsets[nodes[k]+1],parent))return false;
  return true;
}
}
Forecast Preflight(const tl::fea::ShellPhysicalBinding& physical,Source source,Controls controls,
    Limits limits,Layout* output) noexcept try {
  Forecast result;auto reject=[&](S status,const char* message){result.report=Fail(status,message);return result;};
  if((source.ordinary!=nullptr)==(source.mixed!=nullptr)||!limits.output_bytes||!limits.startup_bytes||limits.containing_parents>=UINT32_MAX)
    return reject(S::InvalidInput,"Exactly one complete contact source is required");
  auto status=Parents(physical,result.counts,nullptr,nullptr);if(status.status!=S::Ok){result.report=status;return result;}
  auto& count=result.counts;count.mains=source.mains();count.origins=source.origins();count.primaries=source.primaries();
  if(!count.nodes||count.nodes>=UINT32_MAX||count.nodes>limits.nodes||count.parents>limits.parents||
      count.incidence>limits.incidence||count.mains>limits.mains||count.origins>limits.origins||
      !count.primaries||count.primaries>count.mains||count.mains>=UINT32_MAX||count.origins>=UINT32_MAX)
    return reject(S::ResourceLimit,"Complete activity-source counts exceed the declared caps");
  const auto index=pm::Index::Preflight(physical,limits.startup_bytes);
  if(index.report.status!=S::Ok){result.report=index.report;return result;}
  status=CheckSource(physical,source,controls,limits.startup_bytes);
  if(status.status!=S::Ok){result.report=status;return result;}
  tl::util::BoundedArenaLayout scratch_layout(limits.startup_bytes);tl::util::ArenaRegion node_region;
  if(!scratch_layout.Append<std::uint32_t>(count.nodes,node_region))
    return reject(S::ResourceLimit,"Activity incidence count scratch exceeds cap");
  tl::util::HostArena scratch;if(!scratch.Initialize(scratch_layout.bytes()))
    return reject(S::ResourceLimit,"Activity incidence count allocation failed");
  auto* node_counts=scratch.Construct<std::uint32_t>(node_region);std::fill_n(node_counts,count.nodes,0);
  Counts inspected;status=Parents(physical,inspected,node_counts,CountNodes);
  if(status.status!=S::Ok){result.report=status;return result;}
  for(std::size_t i=0;i<count.primaries;++i) {
    const auto* nodes=source.nodes(i);const auto additional=node_counts[nodes[0]];
    if(additional>limits.containing_parents-count.containing_capacity)
      return reject(S::ResourceLimit,"Complete containing-parent capacity exceeds cap");
    count.containing_capacity+=additional;
  }
  Layout layout;tl::util::BoundedArenaLayout storage(limits.output_bytes);
  if(!storage.Append<ParentIdentity>(count.parents,layout.parents)||
      !storage.Append<std::uint32_t>(count.nodes+1,layout.offsets)||
      !storage.Append<std::uint32_t>(count.incidence,layout.incidence)||
      !storage.Append<MainSupport>(count.mains,layout.mains)||
      !storage.Append<Origin>(count.origins,layout.origins)||
      !storage.Append<std::uint32_t>(count.mains,layout.main_to_primary)||
      !storage.Append<std::uint32_t>(count.primaries+1,layout.containing_offsets)||
      !storage.Append<std::uint32_t>(count.containing_capacity,layout.containing_parents))
    return reject(S::ResourceLimit,"Complete activity-source retained storage exceeds cap");
  layout.bytes=storage.bytes();
  tl::util::BoundedArenaLayout owned(limits.output_bytes);tl::util::ArenaRegion ignored;
  if(!owned.Append<std::byte>(sizeof(Storage)+64,ignored)||!owned.Append<std::byte>(layout.bytes,ignored))
    return reject(S::ResourceLimit,"Activity-source handles and payload exceed cap");
  result.output_bytes=owned.bytes();
  // Covers retained output, physical source index, CSR cursor, and the existing
  // mixed-origin validator's count array. Phases are charged together safely.
  tl::util::BoundedArenaLayout peak(limits.startup_bytes);
  if(!peak.Append<std::byte>(result.output_bytes,ignored)||!peak.Append<std::byte>(index.bytes,ignored)||
      !peak.Append<std::uint32_t>(count.nodes,ignored)||
      !peak.Append<tl::util::SourceIdentityIndex<0>::Entry>(count.primaries,ignored)||
      !peak.Append<std::size_t>(count.primaries,ignored)||
      !peak.Append<std::byte>(512,ignored))
    return reject(S::ResourceLimit,"Complete activity-source startup peak exceeds cap");
  result.startup_bytes=peak.bytes();result.report=Ok();if(output)*output=layout;return result;
} catch(const std::bad_alloc&) {Forecast result;result.report=Fail(S::ResourceLimit,"Activity-source preflight allocation failed");return result;}
TransactionReport Build(const tl::fea::ShellPhysicalBinding& physical,Source source,Controls controls,
    Limits limits,std::unique_ptr<Storage>& output) noexcept try {
  if(output)return Fail(S::AlreadyInitialized,"Activity source plan is immutable");
  Layout layout;const auto forecast=Preflight(physical,source,controls,limits,&layout);
  if(forecast.report.status!=S::Ok)return forecast.report;
  auto next=std::make_unique<Storage>(physical);next->forecast=forecast;
  if(!next->arena.Initialize(layout.bytes))return Fail(S::ResourceLimit,"Activity-source retained allocation failed");
  const auto& count=forecast.counts;
  auto* parents=next->arena.Construct<ParentIdentity>(layout.parents);
  auto* offsets=next->arena.Construct<std::uint32_t>(layout.offsets);
  auto* entries=next->arena.Construct<std::uint32_t>(layout.incidence);
  std::fill_n(offsets,count.nodes+1,0);Counts inspected;
  auto status=Parents(physical,inspected,offsets+1,CountNodes);if(status.status!=S::Ok)return status;
  for(std::size_t i=1;i<=count.nodes;++i)offsets[i]+=offsets[i-1];
  if(offsets[count.nodes]!=count.incidence)return Fail(S::SourceMismatch,"Physical incidence count changed");
  tl::util::HostArena scratch;if(!scratch.Initialize(count.nodes*sizeof(std::uint32_t)))
    return Fail(S::ResourceLimit,"Activity-source cursor allocation failed");
  auto* cursor=static_cast<std::uint32_t*>(scratch.data());std::copy_n(offsets,count.nodes,cursor);
  WriteContext writer{parents,cursor,entries};status=Parents(physical,inspected,&writer,WriteParent);
  if(status.status!=S::Ok)return status;
  auto* main_support=next->arena.Construct<MainSupport>(layout.mains);
  auto* origins=next->arena.Construct<Origin>(layout.origins);
  pm::Index index;status=index.Initialize(physical,limits.startup_bytes);if(status.status!=S::Ok)return status;
  const auto shells=count.families[0]+count.families[1]+count.families[2];
  status=WriteMains(index,source,shells,main_support,origins);if(status.status!=S::Ok)return status;
  auto* mapping=next->arena.Construct<std::uint32_t>(layout.main_to_primary);
  for(std::size_t i=0;i<count.mains;++i)mapping[i]=source.ordinary?i%count.primaries:source.mixed->expanded_to_primary[i];
  auto* containing_offsets=next->arena.Construct<std::uint32_t>(layout.containing_offsets);
  auto* containing=next->arena.Construct<std::uint32_t>(layout.containing_parents);std::size_t used=0;
  for(std::size_t i=0;i<count.primaries;++i) {
    containing_offsets[i]=static_cast<std::uint32_t>(used);const auto* nodes=source.nodes(i);
    const unsigned n=nodes[2]==nodes[3]?3:4;
    for(auto at=offsets[nodes[0]];at<offsets[nodes[0]+1];++at)
      if(Contains(offsets,entries,nodes,n,entries[at])) {
        if(used>=count.containing_capacity)return Fail(S::ResourceLimit,"Containing-parent forecast was exceeded",i);
        containing[used++]=entries[at];
      }
    if(containing_offsets[i]==used)return Fail(S::SourceMismatch,"Contact main has no actual containing source parent",i);
  }
  containing_offsets[count.primaries]=static_cast<std::uint32_t>(used);
  next->view={{parents,count.parents},{offsets,count.nodes+1},{entries,count.incidence},
      {main_support,count.mains},{mapping,count.mains},{containing_offsets,count.primaries+1},
      {containing,used},{origins,count.origins},controls,source.generation()};
  output=std::move(next);return Ok();
} catch(const std::bad_alloc&) {return Fail(S::ResourceLimit,"Activity-source allocation failed");}
}
namespace tlfea::contact::radioss_type25::activity_source {
Plan::Plan()=default;Plan::~Plan()=default;
bool Plan::initialized()const noexcept{return bool(storage_);}
bool Plan::Matches(const tl::fea::ShellPhysicalBinding& p)const noexcept{return storage_&&storage_->physical.Matches(p);}
View Plan::view()const noexcept{return storage_?storage_->view:View{};}
Forecast Plan::forecast()const noexcept{return storage_?storage_->forecast:Forecast{};}
Forecast Plan::Preflight(const tl::fea::ShellPhysicalBinding& p,const ContactSourceInput& s,Controls c,Limits l)noexcept {
  return detail::Preflight(p,{&s,nullptr,nullptr},c,l);
}
Forecast Plan::Preflight(const tl::fea::ShellPhysicalBinding& p,const startup::MixedSidesSnapshot& s,
    const startup::PostGapmTopology& post,Controls c,Limits l)noexcept{return detail::Preflight(p,{nullptr,&s,&post},c,l);}
Forecast Plan::Preflight(const tl::fea::ShellPhysicalBinding& p,const startup::Snapshot& s,Controls c,Limits l)noexcept {
  if(s.profile!=startup::Profile::MixedSurface||s.topology!=startup::TopologyPolicy::NativeMixedSurface||
      s.primary_identity_count!=s.primary_count||!s.post_gapm){Forecast f;f.report=detail::Fail(TransactionStatus::InvalidInput,"Post-GAPM source is missing");return f;}
  const auto mixed=detail::Mixed(s);return Preflight(p,mixed,*s.post_gapm,c,l);
}
TransactionReport Plan::Initialize(const tl::fea::ShellPhysicalBinding& p,const ContactSourceInput& s,Controls c,Limits l)noexcept {
  return detail::Build(p,{&s,nullptr,nullptr},c,l,storage_);
}
TransactionReport Plan::Initialize(const tl::fea::ShellPhysicalBinding& p,const startup::MixedSidesSnapshot& s,
    const startup::PostGapmTopology& post,Controls c,Limits l)noexcept{return detail::Build(p,{nullptr,&s,&post},c,l,storage_);}
TransactionReport Plan::Initialize(const tl::fea::ShellPhysicalBinding& p,const startup::Snapshot& s,Controls c,Limits l)noexcept {
  if(s.profile!=startup::Profile::MixedSurface||s.topology!=startup::TopologyPolicy::NativeMixedSurface||
      s.primary_identity_count!=s.primary_count||!s.post_gapm)return detail::Fail(TransactionStatus::InvalidInput,"Post-GAPM source is missing");
  const auto mixed=detail::Mixed(s);return Initialize(p,mixed,*s.post_gapm,c,l);
}
}
