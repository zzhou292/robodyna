// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include "Values.h"
namespace tlfea::contact::radioss_type25::source_gaps::detail {
Report Rows(const Input& in,unsigned char* tags) noexcept {
  namespace c=coefficient_detail;
  bool triangles=false;
  for(std::size_t i=0;i<in.shell_count;++i) {
    const auto& row=in.shells[i];const auto slots=Slots(row.layout);
    if(!slots || (triangles&&slots==4)) return {Status::UnsupportedProfile,Field::Shell,i};
    triangles=triangles||slots==3;
    if(!row.source_element_id || !c::Finite(row.part_contact_thickness) ||
        !c::Finite(row.element_thickness) || !c::Finite(row.property_thickness) ||
        (slots==3&&row.nodes[2]!=row.nodes[3])) return {Status::InvalidInput,Field::Shell,i};
    for(unsigned k=0;k<slots;++k) {
      if(row.nodes[k]>=in.node_count) return {Status::InvalidInput,Field::Shell,i};
      for(unsigned j=0;j<k;++j) if(row.nodes[j]==row.nodes[k]) return {Status::InvalidInput,Field::Shell,i};
    }
  }
  for(unsigned family=0;family<2;++family) {
    const auto* rows=family?in.beams:in.trusses;
    const auto count=family?in.beam_count:in.truss_count;
    const auto field=family?Field::Beam:Field::Truss;
    for(std::size_t i=0;i<count;++i) {
      const auto& row=rows[i];
      if(!row.source_element_id || row.nodes[0]>=in.node_count || row.nodes[1]>=in.node_count ||
          !c::Finite(row.part_contact_thickness) ||
          (!(row.part_contact_thickness>0)&&!c::Nonnegative(row.native_area)))
        return {Status::InvalidInput,field,i};
    }
  }
  for(std::size_t i=0;i<in.spring_count;++i) {
    const auto& row=in.springs[i];
    if(row.property_type!=13&&row.property_type!=25&&row.property_type!=45)
      return {Status::UnsupportedProfile,Field::Spring,i};
    if(!row.native_element_id || !c::Finite(row.part_contact_thickness) ||
        row.nodes[0]>=in.node_count || row.nodes[1]>=in.node_count)
      return {Status::InvalidInput,Field::Spring,i};
  }
  for(std::size_t i=0;i<in.main_count;++i) for(const auto node:in.mains[i].nodes) {
    if(node>=in.node_count) return {Status::InvalidInput,Field::Main,i};
    tags[node]|=1;
    if(i<in.primary_count&&in.mains[i].segment_type!=0)tags[node]|=4;
  }
  for(std::size_t i=0;i<in.main_node_count;++i) {
    const auto node=in.main_nodes[i];
    if(node>=in.node_count || !(tags[node]&1) || (tags[node]&2))
      return {Status::InvalidInput,Field::MainRoster,i};
    tags[node]|=2;
  }
  for(std::size_t node=0;node<in.node_count;++node)
    if((tags[node]&1)&&!(tags[node]&2)) return {Status::InvalidInput,Field::Node,node};
  for(std::size_t i=0;i<in.secondary_count;++i) {
    const auto node=in.secondary_nodes[i];
    if(node>=in.node_count || (tags[node]&8)) return {Status::InvalidInput,Field::SecondaryRoster,i};
    tags[node]|=8;
  }
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::source_gaps::detail
