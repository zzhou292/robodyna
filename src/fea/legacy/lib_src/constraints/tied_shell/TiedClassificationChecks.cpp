// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TiedClassificationInternal.h"

namespace tl::constraints::tied_shell::classification_detail {
namespace {
Report Nodes(ClassificationNodes view,std::size_t count,std::size_t row) noexcept {
  for(std::size_t i=0;i<view.count;++i)
    if(view.data[i]>=count) return Fail(Status::InvalidInput,row,i);
  return {};
}
template<class T> Report Identities(ClassificationView<T> view) {
  Index index;
  for(std::size_t i=0;i<view.count;++i)
    if(!view.data[i].source_id || view.data[i].source_id>INT_MAX)
      return Fail(Status::InvalidInput,i);
  index.Prepare(view.count,[&](std::size_t i){ return view.data[i].source_id; });
  for(std::size_t i=0;i<view.count;++i)
    if(index.First(view.data[i].source_id)!=i) return Fail(Status::DuplicateIdentity,i);
  return {};
}
}
Report CheckContext(const ClassificationContext& in) {
  for(std::size_t i=0;i<in.nodes.count;++i) {
    const auto& k=in.nodes.data[i].kinematics;
    if(!Mask(k.conditions) || !Mask(k.duplicate_conditions) || !Mask(k.incompatible_conditions) ||
       !Directions(k.translation) || !Directions(k.rotation)) return Fail(Status::NativeDomain,i);
  }
  return Identities(in.nodes);
}
Report CheckRoles(const ClassificationInput& in) noexcept {
  const auto count=in.context.nodes.count;
  // The allocating source-ID checks are performed separately by the public
  // operation after the complete preflight; this pass has no allocation.
  for(std::size_t i=0;i<in.interfaces.count;++i) {
    const auto& r=in.interfaces.data[i];
    if(r.native_type<0) return Fail(Status::InvalidInput,i);
    auto report=Nodes(r.slaves,count,i);
    if(!report) return report;
    report=Nodes(r.masters,count,i);
    if(!report) return report;
  }
  for(std::size_t i=0;i<in.sections.count;++i) {
    if(in.sections.data[i].native_type<0) return Fail(Status::InvalidInput,i);
    auto report=Nodes(in.sections.data[i].nodes,count,i);
    if(!report) return report;
  }
  for(std::size_t i=0;i<in.cyclic_tags.count;++i)
    if(in.cyclic_tags.data[i]<0 || in.cyclic_tags.data[i]>INT_MAX-13)
      return Fail(Status::NativeDomain,i);
  for(std::size_t i=0;i<in.tetra_edges.count;++i) {
    const auto e=in.tetra_edges.data[i];
    if(e.midpoint>=count || e.first_corner>=count || e.second_corner>=count)
      return Fail(Status::InvalidInput,i);
  }
  for(std::size_t i=0;i<in.tetra_tags.count;++i) {
    const auto value=static_cast<std::int64_t>(in.tetra_tags.data[i]);
    const auto size=static_cast<std::int64_t>(in.tetra_edges.count);
    if(value < -size || value > size) return Fail(Status::InvalidInput,i);
  }
  auto report=Nodes(in.rbe2_nodes,count,0);
  if(!report) return report;
  for(std::size_t i=0;i<in.rbe3_members.count;++i) {
    report=Nodes(in.rbe3_members.data[i],count,i);
    if(!report) return report;
  }
  return {};
}
Report CheckRoles(const RigidRegistrationInput& in) noexcept {
  for(std::size_t i=0;i<in.groups.count;++i) {
    auto report=Nodes(in.groups.data[i].members,in.context.nodes.count,i);
    if(!report) return report;
  }
  return {};
}
Report CheckInterfaceIdentities(const ClassificationInput& in) { return Identities(in.interfaces); }
Report CheckGroupIdentities(const RigidRegistrationInput& in) { return Identities(in.groups); }
} // namespace tl::constraints::tied_shell::classification_detail
