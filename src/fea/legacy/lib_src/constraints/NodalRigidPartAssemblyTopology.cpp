// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidPartAssemblyInternal.h"

namespace tl::fea::rigid::part_assembly_detail {
Report Clone(const NodalRigidPartTopology& t,NodalRigidPartTopology& out) {
  auto parts=std::make_unique<PartTopologyPartInput[]>(t.part_count());
  std::unique_ptr<PartTopologyExtraInput[]> extras;
  if(t.extra_count()) extras=std::make_unique<PartTopologyExtraInput[]>(t.extra_count());
  for(std::size_t p=0;p<t.part_count();++p) {
    const auto& row=t.parts()[p];
    parts[p]={row.source_part_id,t.original_members()+row.member_offset,row.member_count};
  }
  for(std::size_t e=0;e<t.extra_count();++e) {
    const auto& row=t.extras()[e];
    extras[e]={t.parts()[row.part_index].source_part_id,row.source_node_set_id,
               t.original_members()+row.member_offset,row.member_count};
  }
  PartTopologyInput input;
  input.source_instance_id=t.source_instance_id();
  input.parts=parts.get();input.part_count=t.part_count();
  input.extras=extras.get();input.extra_count=t.extra_count();
  input.merges=t.merges();input.merge_count=t.merge_count();
  input.expected_members=t.expected_members();input.expected_member_count=t.member_count();
  input.other_rigid_members=t.other_rigid_members();
  input.other_rigid_member_count=t.other_rigid_member_count();
  // This is the same already admitted topology, including its original peak.
  input.limits.max_host_bytes=t.startup_payload_bytes();
  const auto report=out.Initialize(input);
  if(!report) return Fail(report.status==PartTopologyStatus::ResourceLimit?S::ResourceLimit:S::InvalidInput,
                          report.message,report.row,report.member);
  return {};
}
bool SameTopology(const NodalRigidPartTopology& a,const NodalRigidPartTopology& b) noexcept {
  if(!a.prepared()||!b.prepared()) return false;
  if(&a==&b) return true;
  if(a.source_instance_id()!=b.source_instance_id()||a.part_count()!=b.part_count()||
      a.extra_count()!=b.extra_count()||a.merge_count()!=b.merge_count()||
      a.member_count()!=b.member_count()||a.other_rigid_member_count()!=b.other_rigid_member_count()) return false;
  for(std::size_t p=0;p<a.part_count();++p) {
    const auto& x=a.parts()[p];const auto& y=b.parts()[p];
    if(x.source_part_id!=y.source_part_id||x.member_count!=y.member_count) return false;
  }
  for(std::size_t e=0;e<a.extra_count();++e) {
    const auto& x=a.extras()[e];const auto& y=b.extras()[e];
    if(x.source_node_set_id!=y.source_node_set_id||x.part_index!=y.part_index||
        x.member_count!=y.member_count) return false;
  }
  for(std::size_t m=0;m<a.merge_count();++m)
    if(a.merges()[m].parent_part_id!=b.merges()[m].parent_part_id||
        a.merges()[m].child_part_id!=b.merges()[m].child_part_id) return false;
  for(std::size_t n=0;n<a.member_count();++n)
    if(a.original_members()[n]!=b.original_members()[n]||a.expected_members()[n]!=b.expected_members()[n]) return false;
  for(std::size_t n=0;n<a.other_rigid_member_count();++n)
    if(a.other_rigid_members()[n]!=b.other_rigid_members()[n]) return false;
  return true;
}
} // namespace tl::fea::rigid::part_assembly_detail
