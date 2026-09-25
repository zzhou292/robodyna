// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Optimized.h"
namespace tlfea::contact::radioss_type25::selection::lifecycle::detail {
TL_MATH_HOST_DEVICE inline Status PrepareSliding(const Main& main,
    const CandidateCache& cache,RowResult& row) {
  if(row.history.row.irtlm[1]>0)return Status::Ok;
  for(const auto& sector:cache.sector)
    if((sector.defined&(FarDefined|PenetrationDefined))!=(FarDefined|PenetrationDefined))
      return Status::UndefinedNativeInput;
  const bool triangle=main.nodes[2]==main.nodes[3];
  if(!triangle) {
    const auto old=(-std::int64_t(row.history.row.irtlm[1]))/5;
    if(old<1||old>4)return Status::UndefinedNativeInput;
    if(cache.sector[old-1].far==2) {
      bool tagged[4]{};
      for(unsigned i=0;i<4;++i)if(cache.sector[i].far==2){tagged[i]=true;tagged[(i+1)%4]=true;}
      // The native NSLIDE increments for EVERY slot, not just tagged entries.
      for(unsigned i=0;i<4;++i)if(tagged[i])row.sliding_reference[i]=main.normal_reference[i];
    } else if(cache.sector[old-1].penetration==0) {
      for(unsigned i=0;i<4;++i)row.sliding_reference[i]=main.normal_reference[i];
    } else row.history.row.selection_metric[0]=cache.sector[old-1].penetration;
  } else {
    if(cache.sector[0].far==2||cache.sector[1].far==2||cache.sector[2].far==2) {
      if(cache.sector[0].far==2){row.sliding_reference[0]=main.normal_reference[0];row.sliding_reference[1]=main.normal_reference[1];}
      if(cache.sector[1].far==2){row.sliding_reference[1]=main.normal_reference[1];row.sliding_reference[2]=main.normal_reference[2];}
      if(cache.sector[2].far==2){row.sliding_reference[2]=main.normal_reference[2];row.sliding_reference[0]=main.normal_reference[0];}
    } else if(cache.sector[0].penetration==0) {
      for(unsigned i=0;i<3;++i)row.sliding_reference[i]=main.normal_reference[i];
    } else row.history.row.selection_metric[0]=cache.sector[0].penetration;
  }
  return Status::Ok;
}
TL_MATH_HOST_DEVICE inline bool Contains(const std::uint32_t* entries,
    std::size_t begin,std::size_t end,std::uint32_t value) {
  for(std::size_t i=begin;i<end;++i)if(entries[i]==value)return true;
  return false;
}
TL_MATH_HOST_DEVICE inline bool SlidingCandidate(const Input& input,std::size_t row,
    int local_main,const RowResult& state) {
  const auto& scene=input.source;const auto& main=scene.mains[local_main-1];
  std::int64_t symmetric=main.segment_type;if(symmetric<0)symmetric=-symmetric;
  if(symmetric>std::int64_t(scene.main_count))symmetric-=std::int64_t(scene.main_count);
  if(symmetric!=0&&scene.mains[symmetric-1].global_id==state.history.row.irtlm[0])return false;
  if(main.coefficient<=0||main.global_id==state.history.row.irtlm[0])return false;
  const auto node=scene.secondary[row].node;
  for(auto corner:main.nodes)if(corner==node)return false;
  if(input.profile.neighbor_removal==2) {
    const auto& csr=scene.removed_main_by_secondary;
    if(Contains(csr.entries,csr.offsets[row],csr.offsets[row+1],std::uint32_t(local_main)))return false;
  }
  return true;
}
// Scratch holds every potential incidence before final candidate admission.
// It is independently forecast; a smaller output cap never truncates counting.
TL_MATH_HOST_DEVICE inline Status SlidingMains(const Input& input,std::size_t row,
    RowResult& state,int* scratch,std::size_t capacity) {
  std::size_t required_scratch=0;
  const auto& csr=input.source.normal_to_main;
  for(int reference:state.sliding_reference)if(reference!=0) {
    const auto count=std::size_t(csr.offsets[reference]-csr.offsets[reference-1]);
    if(count>SIZE_MAX-required_scratch)return Status::CapacityExceeded;
    required_scratch+=count;
  }
  if(required_scratch>capacity||required_scratch&& !scratch)return Status::CapacityExceeded;
  std::size_t count=0;
  for(int reference:state.sliding_reference)if(reference!=0)
    for(std::size_t i=csr.offsets[reference-1];i<csr.offsets[reference];++i) {
      const int main=int(csr.entries[i]);
      if(!SlidingCandidate(input,row,main,state))continue;
      bool seen=false;for(std::size_t j=0;j<count;++j)if(scratch[j]==main){seen=true;break;}
      if(!seen)scratch[count++]=main;
    }
  state.sliding_count=count;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::selection::lifecycle::detail
