// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Channels.h"
#include "../Measurement.h"
namespace tl::fea::qbat::mapped::measurement {
// One64-lane block: independent scalar channels keep their own complete order.
// Only the leader mutates Control; every gate is uniform across both warps.
__device__ inline void Finalize(batch_detail::Storage& state,const NodalPreparedView& view,
    BatchDiagnostics identity,unsigned blocks,Tile& tile) noexcept {
  const unsigned lane=threadIdx.x;
  batch_detail::Control next{};
  if(!lane) {
    next=BeginMeasurement(state,identity);
    tile.proceed=next.status==BatchStatus::Success;
    if(!tile.proceed)state.control=next;
    else next.diagnostics.element_count=state.model.config.element_count;
  }
  __syncthreads();
  if(!tile.proceed)return;
  for(std::size_t first=0;first<state.model.config.element_count;first+=Threads) {
    const auto remaining=state.model.config.element_count-first;
    const unsigned count=remaining<Threads?unsigned(remaining):Threads;
    if(lane<count)Read(tile,lane,state.assembly.measurement[first+lane]);
    const unsigned mask=__ballot_sync(0xffffffffu,lane<count && tile.valid[lane]!=1);
    if((lane&31)==0)tile.invalid[lane/32]=mask;
    if(!lane)SeedChannels(tile,next.diagnostics);
    __syncthreads();
    const bool serial=tile.invalid[0] || tile.invalid[1];
    if(!serial) {
      if(lane<8)ScalarChannel(tile,lane,count);
      if(lane>=8 && lane<11)MinimumChannel(tile,lane-8,first,count);
      if(lane==11)MaximumChannel(tile,count);
      if(lane==12 || lane==13)CountChannel(tile,lane==13,count);
      if(lane==32 || lane==33)WorkChannel(tile,lane-24,count,state.model.config.usage==BatchUsage::CoupledForces);
    }
    __syncthreads();
    if(!lane) {
      if(serial)tile.proceed=ReplayTile(state.model,tile,first,count,next.diagnostics);
      else StoreChannels(tile,next.diagnostics);
    }
    __syncthreads();
    if(!tile.proceed)break;
  }
  if(!lane) {
    if(!tile.proceed || !FinishMeasurement(state,view,next.diagnostics,blocks))next.status=BatchStatus::NonfiniteResult;
    else next.diagnostics.valid=true;
    state.control=next;
  }
}
} // namespace tl::fea::qbat::mapped::measurement
