// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Read.h"
#include "../Measurement.h"
namespace tl::fea::qbat::mapped::measurement {
// Called by exactly one Threads-wide block. Only the leader mutates diagnostics
// or publishes Control. All barrier/return decisions use the shared gate.
__device__ inline void Finalize(batch_detail::Storage& state,const NodalPreparedView& view,
    BatchDiagnostics identity,unsigned blocks,Tile& tile) noexcept {
  const unsigned lane=threadIdx.x;
  batch_detail::Control next{};
  if(!lane) {
    next=BeginMeasurement(state,identity);
    tile.proceed=next.status==BatchStatus::Success;
    if(!tile.proceed) state.control=next;
    else next.diagnostics.element_count=state.model.config.element_count;
  }
  __syncthreads();
  if(!tile.proceed) return;
  for(std::size_t first=0;first<state.model.config.element_count;first+=Threads) {
    const auto remaining=state.model.config.element_count-first;
    const unsigned count=remaining<Threads ? unsigned(remaining) : Threads;
    if(lane<count) Read(tile,lane,state.assembly.measurement[first+lane]);
    __syncthreads();
    if(!lane) for(unsigned local=0;local<count;++local) {
      if(tile.valid[local]!=1) { tile.proceed=false; break; }
      const auto value=Load(tile,local);
      AccumulateMeasurementParent(state.model,value,first+local,next.diagnostics);
    }
    __syncthreads();
    if(!tile.proceed) break;
  }
  if(!lane) {
    if(!tile.proceed || !FinishMeasurement(state,view,next.diagnostics,blocks))
      next.status=BatchStatus::NonfiniteResult;
    else next.diagnostics.valid=true;
    state.control=next;
  }
}
} // namespace tl::fea::qbat::mapped::measurement
