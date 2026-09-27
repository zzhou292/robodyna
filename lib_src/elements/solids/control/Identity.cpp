// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Impl.h"
namespace tl::fea::solids::control {
bool Selection::Matches(const Selection& other)const noexcept {
  if(!prepared()||!other.prepared())return false;
  if(impl_==other.impl_)return true;
  if(profile()!=other.profile()||source_instance_id()!=other.source_instance_id()||
     units().length_m!=other.units().length_m||units().mass_kg!=other.units().mass_kg||units().time_s!=other.units().time_s||
     native_nvsiz()!=other.native_nvsiz()||compiled_mvsiz()!=other.compiled_mvsiz()||
     controlled_count()!=other.controlled_count()||parents().size()!=other.parents().size()||
     partitions().size()!=other.partitions().size()||packets().size()!=other.packets().size()||members().size()!=other.members().size())return false;
  for(std::size_t i=0;i<parents().size();++i) {
    const auto& a=parents()[i];const auto& b=other.parents()[i];const auto& x=a.source;const auto& y=b.source;
    if(x.element_id!=y.element_id||x.part_id!=y.part_id||x.section_id!=y.section_id||x.material_id!=y.material_id||
       x.native_property_id!=y.native_property_id||x.icontrol!=y.icontrol||a.family!=b.family||a.family_index!=b.family_index||
       a.packet_index!=b.packet_index||a.packet_slot!=b.packet_slot)return false;
  }
  for(std::size_t i=0;i<partitions().size();++i) {
    const auto& a=partitions()[i];const auto& b=other.partitions()[i];
    if(a.partition_id!=b.partition_id||a.packet_begin!=b.packet_begin||a.packet_count!=b.packet_count||
       a.member_begin!=b.member_begin||a.member_count!=b.member_count)return false;
  }
  for(std::size_t i=0;i<packets().size();++i) {
    const auto& a=packets()[i];const auto& b=other.packets()[i];const auto& x=a.source;const auto& y=b.source;
    if(a.partition_index!=b.partition_index||x.group_id!=y.group_id||x.native_first!=y.native_first||
       x.member_begin!=y.member_begin||x.member_count!=y.member_count||x.family!=y.family||x.material_id!=y.material_id||
       x.native_property_id!=y.native_property_id||x.icontrol!=y.icontrol)return false;
  }
  for(std::size_t i=0;i<members().size();++i) {
    const auto& a=members()[i];const auto& b=other.members()[i];
    if(a.element_id!=b.element_id||a.family!=b.family||a.family_index!=b.family_index||a.parent_index!=b.parent_index)return false;
  }
  return true;
}
} // namespace tl::fea::solids::control
