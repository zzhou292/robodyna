// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TiedClassificationInternal.h"
#include <algorithm>

namespace tl::constraints::tied_shell::classification_detail {
namespace {
bool Count(std::size_t n,std::size_t cap,std::size_t& total) noexcept {
  cap=std::min(cap,NativeCountCap);
  if(n>cap-total) return false;
  total+=n;
  return true;
}
bool Root(const ClassificationContext& c,ClassificationLimits l) noexcept {
  return c.source_instance_id && c.nodes.count && c.nodes.count<=l.max_nodes &&
         c.nodes.count<=NativeCountCap && Span(c.nodes);
}
Report Budget(const ClassificationContext& c,std::size_t interfaces,
              std::size_t index_count,const ClassificationResult& old,
              ClassificationLimits limits,std::size_t data_bytes,Counts& counts) noexcept {
  tl::util::BoundedArenaLayout layout(limits.max_host_bytes);
  tl::util::ArenaRegion region;
  if(!layout.Append<unsigned char>(data_bytes,region) ||
     !layout.Append<ClassificationNode>(c.nodes.count,region) ||
     !layout.Append<ClassifiedInterface>(interfaces,region) ||
     !layout.Append<std::int32_t>(counts.slaves,region) ||
     !layout.Append<std::uint32_t>(counts.slaves,region)) return Fail(Status::ResourceLimit);
  counts.owned=layout.bytes();
  if(!layout.Append<unsigned char>(old.owned_payload_bytes(),region) ||
     !layout.Append<NativeKinematics>(c.nodes.count,region) ||
     !layout.Append<std::int32_t>(3*c.nodes.count,region) ||
     !layout.Append<Index::Entry>(index_count,region) ||
     !layout.Append<unsigned char>(sizeof(Index),region)) return Fail(Status::ResourceLimit);
  counts.startup=layout.bytes();
  return {};
}
template<class T> bool RootView(ClassificationView<T> v,std::size_t cap) noexcept {
  return v.count<=cap && v.count<=NativeCountCap && Span(v);
}
}

Report Preflight(const ClassificationInput& in,const ClassificationResult& old,
                 ClassificationLimits limits,std::size_t data_bytes,Counts& counts) noexcept {
  // Root count caps precede every borrowed metadata/payload read.
  if(in.context.nodes.count>limits.max_nodes || in.interfaces.count>limits.max_interfaces ||
     in.sections.count>limits.max_roles || in.tetra_edges.count>limits.max_roles ||
     in.rbe2_nodes.count>limits.max_roles || in.rbe3_members.count>limits.max_roles ||
     in.cyclic_tags.count>limits.max_nodes || in.tetra_tags.count>limits.max_nodes)
    return Fail(Status::ResourceLimit);
  if(!Root(in.context,limits) || !in.interfaces.count ||
     !RootView(in.interfaces,limits.max_interfaces) || !RootView(in.sections,limits.max_roles) ||
     !RootView(in.tetra_edges,limits.max_roles) || !RootView(in.rbe2_nodes,limits.max_roles) ||
     !RootView(in.rbe3_members,limits.max_roles) || !Span(in.cyclic_tags) || !Span(in.tetra_tags) ||
     (in.cyclic_tags.count && in.cyclic_tags.count!=in.context.nodes.count) ||
     in.tetra_tags.count!=(in.tetra_edges.count ? in.context.nodes.count : 0))
    return Fail(Status::InvalidInput);
  // A too-small basic byte cap must reject before even role metadata is read.
  auto report=Budget(in.context,in.interfaces.count,
                    std::max(in.context.nodes.count,in.interfaces.count),old,limits,data_bytes,counts);
  if(!report) return report;
  for(std::size_t i=0;i<in.interfaces.count;++i) {
    const auto& role=in.interfaces.data[i];
    if(!Count(role.slaves.count,limits.max_occurrences,counts.occurrences) ||
       !Count(role.masters.count,limits.max_occurrences,counts.occurrences))
      return Fail(Status::ResourceLimit,i);
    if(!Span(role.slaves) || !Span(role.masters)) return Fail(Status::InvalidInput,i);
    counts.slaves+=role.slaves.count;
  }
  for(std::size_t i=0;i<in.sections.count;++i) {
    const auto nodes=in.sections.data[i].nodes;
    if(!Count(nodes.count,limits.max_occurrences,counts.occurrences)) return Fail(Status::ResourceLimit,i);
    if(!Span(nodes)) return Fail(Status::InvalidInput,i);
  }
  for(std::size_t i=0;i<in.rbe3_members.count;++i) {
    const auto nodes=in.rbe3_members.data[i];
    if(!Count(nodes.count,limits.max_occurrences,counts.occurrences)) return Fail(Status::ResourceLimit,i);
    if(!Span(nodes)) return Fail(Status::InvalidInput,i);
  }
  if(!Count(in.rbe2_nodes.count,limits.max_occurrences,counts.occurrences))
    return Fail(Status::ResourceLimit);
  return Budget(in.context,in.interfaces.count,
                std::max(in.context.nodes.count,in.interfaces.count),old,limits,data_bytes,counts);
}

Report Preflight(const RigidRegistrationInput& in,const ClassificationResult& old,
                 ClassificationLimits limits,std::size_t data_bytes,Counts& counts) noexcept {
  if(in.context.nodes.count>limits.max_nodes || in.groups.count>limits.max_roles)
    return Fail(Status::ResourceLimit);
  if(!Root(in.context,limits) || !RootView(in.groups,limits.max_roles) || in.native_iddlevel<0)
    return Fail(Status::InvalidInput);
  auto report=Budget(in.context,0,std::max(in.context.nodes.count,in.groups.count),old,limits,data_bytes,counts);
  if(!report) return report;
  for(std::size_t i=0;i<in.groups.count;++i) {
    const auto nodes=in.groups.data[i].members;
    if(!Count(nodes.count,limits.max_occurrences,counts.occurrences)) return Fail(Status::ResourceLimit,i);
    if(!Span(nodes)) return Fail(Status::InvalidInput,i);
  }
  return {};
}
} // namespace tl::constraints::tied_shell::classification_detail
