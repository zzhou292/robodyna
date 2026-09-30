// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidPartTopologyInternal.h"

namespace tl::fea::rigid::topology_detail {
Report Preflight(const PartTopologyInput& in,std::size_t impl_bytes,Counts& out) noexcept {
  const auto& l=in.limits;
  if(!in.source_instance_id||!l.max_parts||!l.max_members||!l.max_other_rigid_members||!l.max_host_bytes)
    return Fail(Status::InvalidInput,"Rigid PART topology requires source identity and explicit bounds");
  if(l.max_parts>1024||l.max_members>16384||l.max_other_rigid_members>16384||
      !in.part_count||in.part_count>l.max_parts||in.extra_count>in.part_count||
      in.merge_count>in.part_count/2||in.expected_member_count>l.max_members||
      in.other_rigid_member_count>l.max_other_rigid_members)
    return Fail(Status::ResourceLimit,"Rigid PART topology count exceeds its admitted domain");
  if(!Span(in.parts,in.part_count)||!Span(in.extras,in.extra_count)||!Span(in.merges,in.merge_count)||
      !in.expected_member_count||!Span(in.expected_members,in.expected_member_count)||
      !Span(in.other_rigid_members,in.other_rigid_member_count))
    return Fail(Status::InvalidInput,"Rigid PART topology has missing or overflowing borrowed ranges");
  std::size_t count=0;
  for(std::size_t i=0;i<in.part_count;++i) {
    const auto& p=in.parts[i];
    if(p.node_count<2||p.node_count>l.max_members-count)
      return Fail(Status::ResourceLimit,"PART membership count exceeds admitted domain",i);
    if(!Span(p.nodes,p.node_count))return Fail(Status::InvalidInput,"Invalid PART member range",i);
    count+=p.node_count;
  }
  for(std::size_t i=0;i<in.extra_count;++i) {
    const auto& x=in.extras[i];
    if(!x.node_count||x.node_count>l.max_members-count)
      return Fail(Status::ResourceLimit,"Extra-node count exceeds admitted domain",i);
    if(!Span(x.nodes,x.node_count))return Fail(Status::InvalidInput,"Invalid extra-node range",i);
    count+=x.node_count;
  }
  if(count!=in.expected_member_count)return Fail(Status::MissingMember,"Complete expected rigid membership count differs");
  std::size_t owned=impl_bytes;
  if(!AddBytes(in.part_count,sizeof(PartTopologyPart),owned)||
      !AddBytes(in.extra_count,sizeof(PartTopologyExtra),owned)||
      !AddBytes(in.merge_count,sizeof(PartTopologyMerge),owned)||
      !AddBytes(in.part_count-in.merge_count,sizeof(PartTopologyRoot),owned)||
      !AddBytes(count,3*sizeof(SourceNodeId),owned)||
      !AddBytes(in.other_rigid_member_count,sizeof(SourceNodeId),owned))
    return Fail(Status::ResourceLimit,"Rigid PART topology owned size overflows");
  std::size_t startup=owned;
  if(!AddBytes(1,Index::Bytes(in.part_count),startup)||
      !AddBytes(1,Index::Bytes(in.extra_count),startup)||
      !AddBytes(2,Index::Bytes(count),startup)||
      !AddBytes(1,Index::Bytes(in.other_rigid_member_count),startup)||
      !AddBytes(in.part_count,2*sizeof(std::size_t),startup)||startup>l.max_host_bytes)
    return Fail(Status::ResourceLimit,"Rigid PART topology startup exceeds explicit byte budget");
  out={count,owned,startup};return {};
}
Report CheckMembers(const PartTopologyInput& in,const SourceNodeId* original,
                    const Index& members,Index& expected,Index& other) {
  for(std::size_t i=0;i<in.expected_member_count;++i) {
    if(!original[i])return Fail(Status::InvalidInput,"Zero original rigid member ID",SIZE_MAX,i);
    if(members.First(original[i])!=i)return Fail(Status::DuplicateMembership,"Original member occurs in multiple PART/extra slots",SIZE_MAX,i);
  }
  expected.Prepare(in.expected_member_count,[&](std::size_t i){return in.expected_members[i];});
  for(std::size_t i=0;i<in.expected_member_count;++i) {
    const auto id=in.expected_members[i];
    if(!id||expected.First(id)!=i)return Fail(Status::DuplicateIdentity,"Expected member inventory is not unique and positive",SIZE_MAX,i);
    if(members.First(id)==SIZE_MAX)return Fail(Status::MissingMember,"Expected source node has no PART/extra assignment",SIZE_MAX,i);
  }
  other.Prepare(in.other_rigid_member_count,[&](std::size_t i){return in.other_rigid_members[i];});
  for(std::size_t i=0;i<in.other_rigid_member_count;++i) {
    const auto id=in.other_rigid_members[i];
    if(!id||other.First(id)!=i)return Fail(Status::DuplicateIdentity,"Other rigid inventory is not unique and positive",SIZE_MAX,i);
    if(members.First(id)!=SIZE_MAX)return Fail(Status::DuplicateMembership,"PART/extra member overlaps a declared plain-rigid group",SIZE_MAX,i);
  }
  return {};
}
} // namespace tl::fea::rigid::topology_detail
