// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Pair.h"
#include "Winner.h"
#include "../selection/lifecycle/Types.h"
#include "../candidates/InventoryTypes.h"
namespace tlfea::contact::radioss_type25::initial_state {
// Shared bounded gather/row value stage. Host facade is qualification only;
// source-sized execution calls it once per secondary from GPU threads. Complete
// Starter inventory/phase authority belongs to the producer, not these views.
struct RowInput {
  Profile profile;
  selection::lifecycle::SourceView source;
  VectorView native_positions;
  const candidates::Pair* pairs=nullptr;
  std::size_t pair_count=0;
  const std::uint64_t* row_offsets=nullptr;
  std::size_t offset_count=0;
};
struct RowReport {Status status=Status::InvalidInput;std::size_t occurrence=SIZE_MAX;};
TL_MATH_HOST_DEVICE inline Status GatherPair(const RowInput& in,std::size_t row,
    std::size_t occurrence,PairInput& output) {
  const auto& src=in.source;
  if(row>=src.secondary_count||occurrence>=in.pair_count||!in.pairs||!src.nodes||!src.mains||!src.secondary||
      !src.normals||!in.native_positions.valid()||in.native_positions.node_count!=src.node_count||
      src.main_count>1073741823)return Status::InvalidInput;
  const auto pair=in.pairs[occurrence];
  if(pair.secondary_row!=row||pair.main_occurrence>=src.main_count)return Status::InvalidInput;
  const auto& secondary=src.secondary[row];const auto& main=src.mains[pair.main_occurrence];
  if(secondary.node>=src.node_count)return Status::InvalidInput;
  PairInput next;next.profile=in.profile;next.expanded_main_count=int(src.main_count);
  auto& p=next.geometry;p.key={src.nodes[secondary.node].source_id,src.generation,row,main.global_id};
  p.local_main=int(pair.main_occurrence+1);p.occurrence=occurrence;p.segment_type=main.segment_type;
  const auto secondary_position=in.native_positions.at(secondary.node);
  p.secondary={secondary_position.x,secondary_position.y,secondary_position.z};p.secondary_gap=secondary.gap;
  // Original Starter COR3's literal IGAP!=3 assignment. This is not maximum
  // physical corner gap or a caller-adjusted clearance.
  p.main_gap_max=native_constant::ep20*native_constant::ep10;
  p.main_coefficient=main.coefficient;p.secondary_coefficient=secondary.coefficient;
  for(unsigned k=0;k<4;++k) {
    if(main.nodes[k]>=src.node_count||main.normal_reference[k]<=0||
        std::size_t(main.normal_reference[k])>src.normal_count)return Status::InvalidInput;
    p.main_node_ids[k]=src.nodes[main.nodes[k]].source_id;const auto position=in.native_positions.at(main.nodes[k]);
    p.main_vertices[k]={position.x,position.y,position.z};
    p.normal_slot[k]=main.normal_slot[k];p.neighbors[k]=main.neighbors[k];p.main_gap[k]=main.gap[k];
    const auto& ref=src.normals[main.normal_reference[k]-1];
    if(ref.boundary<0)return Status::InvalidInput;
    p.boundary_ids[k]=ref.boundary?std::uint64_t(main.normal_reference[k]):0;
    if(ref.boundary)for(unsigned j=0;j<2;++j)p.vertex_bisector[k][j]=ref.bisector[j];
  }
  output=next;return Status::Ok;
}
TL_MATH_HOST_DEVICE inline RowReport ReduceRow(const RowInput& in,std::size_t row,Winner* output) {
  if(!Supported(in.profile))return {Status::UnsupportedProfile,SIZE_MAX};
  if(!output||in.source.secondary_count>UINT32_MAX||row>=in.source.secondary_count||!in.row_offsets||
      in.offset_count!=in.source.secondary_count+1)return {};
  const auto first=in.row_offsets[row],last=in.row_offsets[row+1];
  if(last<first||last>in.pair_count)return {};
  Winner next;
  for(auto occurrence=first;occurrence<last;++occurrence) {
    PairInput pair;auto status=GatherPair(in,row,std::size_t(occurrence),pair);
    if(status!=Status::Ok)return {status,std::size_t(occurrence)};
    PairResult value;status=EvaluatePair(pair,&value);
    if(status!=Status::Ok)return {status,std::size_t(occurrence)};
    status=Consider(value,next);
    if(status!=Status::Ok)return {status,std::size_t(occurrence)};
  }
  // PWR is separate so owning tests can observe both native source phases.
  *output=next;return {Status::Ok,SIZE_MAX};
}
} // namespace tlfea::contact::radioss_type25::initial_state
