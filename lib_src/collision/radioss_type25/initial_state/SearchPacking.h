// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../candidates/PenetrationFilter.h"
namespace tlfea::contact::radioss_type25::initial_state {
// Selected Starter COR3T operands. Complete BUC/TRIVOX eligibility is upstream;
// this value function grants no search/source completeness authority.
struct SearchPair {
  std::uint64_t nodes[4]{};
  Vector vertices[4]{},secondary{};
  int constraint_codes[5]{}; // Exact original ICODE/512 translations, main1..4 then NSV.
  int segment_type=0,expanded_main_count=0;
  double secondary_gap=0,main_gap=0,margin=0,edge_length=0,drad=0,gap_load=0;
};
TL_MATH_HOST_DEVICE inline candidates::Status PackSearch(const SearchPair& in,candidates::PackedRow* output) {
  namespace d=candidates::detail;
  if(!output||in.expanded_main_count<=0||!d::Nonnegative(in.secondary_gap)||!d::Nonnegative(in.main_gap)||
      !d::Nonnegative(in.margin)||!d::Nonnegative(in.edge_length)||in.drad!=0||in.gap_load!=0||
      !tl::math::fixed3::Finite(in.secondary))return candidates::Status::InvalidInput;
  candidates::PackedRow next;next.secondary=in.secondary;next.segment_type=in.segment_type;
  next.main_count=in.expanded_main_count;next.symmetry=in.constraint_codes[4];
  if(next.symmetry<0||next.symmetry>7)return candidates::Status::InvalidInput;
  for(unsigned k=0;k<4;++k) {
    if(!in.nodes[k]||!tl::math::fixed3::Finite(in.vertices[k])||in.constraint_codes[k]<0||in.constraint_codes[k]>7)
      return candidates::Status::InvalidInput;
    for(unsigned j=0;j<k;++j)if(in.nodes[j]==in.nodes[k]&&!d::SameVector(in.vertices[j],in.vertices[k]))
      return candidates::Status::InvalidInput;
    next.nodes[k]=in.nodes[k];next.vertices[k]=in.vertices[k];next.symmetry&=in.constraint_codes[k];
  }
  double zone=in.secondary_gap+in.main_gap;
  if(in.segment_type==0||in.segment_type>in.expanded_main_count)
    zone=d::Max(zone+in.margin,zone+in.edge_length);
  else zone=zone+in.margin;
  next.gap=d::Max(zone+in.gap_load,in.drad);next.margin=0;
  if(!d::Nonnegative(next.gap))return candidates::Status::NonfiniteResult;
  *output=next;return candidates::Status::Ok;
}
TL_MATH_HOST_DEVICE inline candidates::Status EvaluateSearchPair(const SearchPair& in,candidates::FilterResult* output) {
  if(!output)return candidates::Status::InvalidInput;
  candidates::PackedRow row;const auto status=PackSearch(in,&row);
  if(status!=candidates::Status::Ok)return status;
  // Whole I25PEN3A and qualified I25PEN3 have identical numerical bodies after
  // ZONEINF and the solid/coating count operand are resolved. Here margin0's
  // addition cannot alter finite nonnegative zone squared, including signed0.
  return candidates::EvaluatePacked(row,output);
}
} // namespace tlfea::contact::radioss_type25::initial_state
