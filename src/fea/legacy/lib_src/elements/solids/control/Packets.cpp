// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tl::fea::solids::control::detail {
Report BindPackets(Input in,Storage& out,Scratch& scratch) noexcept {
  std::size_t packet_cursor=0,member_cursor=0;
  for(std::size_t part=0;part<in.partitions.size();++part) {
    const auto& partition=in.partitions[part];
    if(partition.partition_id!=part)
      return {Status::SourceMismatch,"Complete zero-based native partition roster must include empty partitions"};
    if(partition.packet_begin!=packet_cursor||partition.member_begin!=member_cursor||
       partition.packet_count>in.packets.size()-packet_cursor||partition.member_count>in.ordered_element_ids.size()-member_cursor||
       ((!partition.packet_count)!=( !partition.member_count)))
      return {Status::SourceMismatch,"Native partition roster has missing or overlapping extents"};
    std::size_t native_cursor=0;std::uint64_t previous_group=0;
    for(std::size_t j=0;j<partition.packet_count;++j) {
      const auto at=packet_cursor+j;const auto& packet=in.packets[at];
      if(!packet.group_id||packet.group_id<=previous_group)
        return {Status::DuplicateIdentity,"Native packet group IDs must preserve unique source order",SIZE_MAX,at};
      previous_group=packet.group_id;
      if(packet.native_first!=native_cursor||packet.member_begin!=member_cursor+native_cursor||
         !packet.member_count||packet.member_count>partition.member_count-native_cursor||
         packet.member_count>in.native_nvsiz||packet.member_count>in.compiled_mvsiz)
        return {Status::SourceMismatch,"Native NFT/NEL roster has an unexplained gap, overlap or vector overrun",SIZE_MAX,at};
      if(static_cast<unsigned>(packet.family)>=5||!packet.material_id||!packet.native_property_id||packet.icontrol>1)
        return {Status::InvalidInput,"Invalid native packet semantic declaration",SIZE_MAX,at};
      if(packet.icontrol&&!Supported(packet.family))return {Status::UnsupportedProfile,"Unsupported controlled native packet family",SIZE_MAX,at};
      out.packets[at]={packet,part};
      for(std::size_t k=0;k<packet.member_count;++k) {
        const auto ordinal=packet.member_begin+k;const auto eid=in.ordered_element_ids[ordinal];
        const auto parent=scratch.identities.First(eid);
        if(parent==SIZE_MAX)return {Status::SourceMismatch,"Native packet contains element absent from model",SIZE_MAX,at};
        if(scratch.seen[parent])return {Status::DuplicateIdentity,"Element occurs in more than one native packet position",parent,at};
        auto& row=out.parents[parent];
        if(row.family!=packet.family||row.source.material_id!=packet.material_id||
           row.source.native_property_id!=packet.native_property_id||row.source.icontrol!=packet.icontrol)
          return {Status::SourceMismatch,"Native packet mixes family, material, property or control semantics",parent,at};
        out.members[ordinal]={eid,row.family,row.family_index,parent};
        row.packet_index=at;row.packet_slot=k;scratch.seen[parent]=1;
      }
      native_cursor+=packet.member_count;
    }
    if(native_cursor!=partition.member_count)return {Status::SourceMismatch,"Native partition has omitted solid packet members"};
    out.partitions[part]=partition;packet_cursor+=partition.packet_count;member_cursor+=partition.member_count;
  }
  if(packet_cursor!=in.packets.size()||member_cursor!=in.ordered_element_ids.size())
    return {Status::SourceMismatch,"Native solid roster contains unclaimed trailing packets or elements"};
  for(std::size_t i=0;i<in.parents.size();++i)if(!scratch.seen[i])
    return {Status::SourceMismatch,"Native solid roster omitted model element",i};
  return {};
}
} // namespace tl::fea::solids::control::detail
