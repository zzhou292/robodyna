// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Values.h"
#include "../selection/lifecycle/Admission.h"
namespace tlfea::contact::radioss_type25::normal_activation::detail {
namespace ld=lifecycle::detail;
namespace gd=geometry_detail;
template<class T> TL_MATH_HOST_DEVICE inline gd::Range Range(const T* p,std::size_t n) {
  return {p,n*sizeof(T),alignof(T)}; // Called after the shared typed Span check.
}
TL_MATH_HOST_DEVICE inline Status Validate(const Input& in,Limits cap,Output out) {
  const auto& p=in.profile;const auto& s=in.source;
  if(p.edge_mode!=0||p.foreign_rows!=0||p.partitions!=1||p.neighbor_removal!=2||
     p.local_processor!=1||p.free_roster!=FreeRosterPolicy::FreshComplete)return Status::UnsupportedProfile;
  if(!s.node_count||!s.main_count||!s.normal_count||s.node_count>UINT32_MAX||s.main_count>INT_MAX||
     s.normal_count>INT_MAX||s.secondary_count>INT_MAX||!s.generation||in.row_count!=s.secondary_count||
     out.node_count!=s.node_count||out.main_count!=s.main_count)return Status::InvalidInput;
  if(s.node_count>cap.nodes||s.main_count>cap.mains||s.secondary_count>cap.secondaries||
     s.normal_count>cap.references||s.normal_to_main.entry_count>cap.incidences||
     s.removed_main_by_secondary.entry_count>cap.removals||in.optimized_count>cap.optimized||
     in.free_count>s.main_count)return Status::CapacityExceeded;
  if(!ld::Span(s.nodes,s.node_count)||!ld::Span(s.mains,s.main_count)||
     !ld::Span(s.secondary,s.secondary_count)||!ld::Span(s.normals,s.normal_count)||
     !ld::Span(in.rows,in.row_count)||!ld::Span(in.optimized_main_ids,in.optimized_count)||
     !ld::Span(in.free_main_ids,in.free_count)||!ld::Span(out.main_active,out.main_count)||
     !ld::Span(out.node_tag,out.node_count)||
     !ld::CsrValid(s.normal_to_main,s.normal_count,s.main_count,true)||
     !ld::CsrValid(s.removed_main_by_secondary,s.secondary_count,s.main_count,true))return Status::InvalidInput;
  const gd::Range inputs[]{Range(&in,1),Range(s.nodes,s.node_count),Range(s.mains,s.main_count),
      Range(s.secondary,s.secondary_count),Range(s.normals,s.normal_count),Range(in.rows,in.row_count),
      Range(in.optimized_main_ids,in.optimized_count),Range(in.free_main_ids,in.free_count),
      Range(s.normal_to_main.offsets,s.normal_to_main.offset_count),Range(s.normal_to_main.entries,s.normal_to_main.entry_count),
      Range(s.removed_main_by_secondary.offsets,s.removed_main_by_secondary.offset_count),
      Range(s.removed_main_by_secondary.entries,s.removed_main_by_secondary.entry_count)};
  const auto main_output=Range(out.main_active,out.main_count),node_output=Range(out.node_tag,out.node_count);
  if(!gd::Disjoint(main_output,node_output))return Status::InvalidInput;
  for(const auto& range:inputs)
    if(!gd::Disjoint(main_output,range)||!gd::Disjoint(node_output,range))return Status::InvalidInput;
  for(std::size_t i=0;i<s.main_count;++i) {
    const auto& main=s.mains[i];
    if(main.global_id<=0||main.segment_type==INT_MIN||!tl::math::Finite(main.coefficient)||
       std::int64_t(main.segment_type)<-2*std::int64_t(s.main_count)||
       std::int64_t(main.segment_type)>2*std::int64_t(s.main_count))return Status::InvalidInput;
    for(unsigned j=0;j<4;++j)
      if(main.nodes[j]>=s.node_count||main.normal_reference[j]<=0||
         std::size_t(main.normal_reference[j])>s.normal_count||main.neighbors[j]<0||
         std::size_t(main.neighbors[j])>s.main_count)return Status::InvalidInput;
  }
  std::size_t optimized=0;
  for(std::size_t row=0;row<in.row_count;++row) {
    const auto& stage=in.rows[row].stage;const auto& secondary=s.secondary[row];
    if(stage.report.status!=Status::Ok||stage.report.stage!=lifecycle::Stage::Optimize||
       stage.report.count_complete||stage.report.required_candidates!=0||secondary.node>=s.node_count||
       !tl::math::Finite(secondary.coefficient)||
       stage.value.history.secondary_source_id!=s.nodes[secondary.node].source_id||
       stage.value.history.generation!=s.generation)return Status::InvalidInput;
    if(stage.value.optimized_count>in.optimized_count-optimized)return Status::InvalidInput;
    optimized+=stage.value.optimized_count;
    const auto& h=stage.value.history.row;
    if(h.irtlm[0]>0&&secondary.coefficient!=0&&h.irtlm[3]==p.local_processor) {
      if(h.irtlm[2]<=0||std::size_t(h.irtlm[2])>s.main_count||
         s.mains[h.irtlm[2]-1].global_id!=h.irtlm[0])return Status::InvalidInput;
    }
  }
  if(optimized!=in.optimized_count)return Status::InvalidInput;
  for(std::size_t i=0;i<in.optimized_count;++i)
    if(!in.optimized_main_ids[i]||in.optimized_main_ids[i]>s.main_count)return Status::InvalidInput;
  // Authenticate the complete fresh FREE_IRECT_ID roster, including original
  // increasing main order. Do not silently regenerate or fill a missing list.
  std::size_t free=0;
  for(std::size_t i=0;i<s.main_count;++i)if(FreeMain(s.mains[i])) {
    if(free>=in.free_count||in.free_main_ids[free]!=i+1)return Status::InvalidInput;
    ++free;
  }
  return free==in.free_count?Status::Ok:Status::InvalidInput;
}
} // namespace tlfea::contact::radioss_type25::normal_activation::detail
