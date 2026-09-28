// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Read.h"
namespace tl::fea::type13::batch_detail::measurement {
// Exactly one Threads-wide block. All lanes use the shared barrier/return gate.
// The complete status pass precedes every payload read and floating addition.
__device__ inline void Finalize(Storage& state,BatchDiagnostics identity,Tile& tile) noexcept {
  const unsigned lane=threadIdx.x;
  Control next;
  if(!lane) {next.diagnostics=identity;tile.proceed=true;}
  __syncthreads();
  for(std::size_t first=0;first<state.model.element_count;first+=Threads) {
    const auto remaining=state.model.element_count-first;
    const unsigned count=remaining<Threads ? unsigned(remaining) : Threads;
    if(lane<count) tile.status[lane]=state.candidate_status[first+lane];
    __syncthreads();
    if(!lane) for(unsigned i=0;i<count;++i) if(tile.status[i]!=Status::Success) {
      next.status=BatchStatus::ElementFailure;next.element=first+i;next.element_status=tile.status[i];
      tile.proceed=false;break;
    }
    __syncthreads();
    if(!tile.proceed) break;
  }
  if(!tile.proceed) {if(!lane)state.control=next;return;}
  if(!lane) {
    next.diagnostics.element_count=state.model.element_count;
    next.diagnostics.minimum_native_dt_s=state.measurement[0].native_dt;
  }
  for(std::size_t first=0;first<state.model.element_count;first+=Threads) {
    const auto remaining=state.model.element_count-first;
    const unsigned count=remaining<Threads ? unsigned(remaining) : Threads;
    if(lane<count) Read(tile,lane,state.measurement[first+lane]);
    __syncthreads();
    if(!lane) for(unsigned i=0;i<count;++i) {
      const auto row=Load(tile,i);AccumulatePrepared(row,next.diagnostics);
    }
    __syncthreads();
  }
  if(!lane) {
    if(!FinishPrepared(next.diagnostics))next.status=BatchStatus::NonfiniteResult;
    else next.diagnostics.valid=true;
    state.control=next;
  }
}
}
