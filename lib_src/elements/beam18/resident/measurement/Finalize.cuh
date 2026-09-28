// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Read.h"
#include <cfloat>
namespace tl::fea::beam18::batch_detail::measurement {
// Every lane follows the same shared gate. Status/validity remains per-parent,
// not a global prescan: earlier numerical failure wins over later bad status.
__device__ inline void Finalize(Storage& state,unsigned accepted,unsigned trial,
    NodalPreparedView view,BatchDiagnostics identity,bool initial,Tile& tile) noexcept {
  const unsigned lane=threadIdx.x;Control next;
  if(!lane) {
    if(initial) {
      identity.source_instance_id=state.source_instance_id;identity.owner_id=state.config.owner.owner_id;
      identity.configuration_id=state.config.configuration_id;identity.qualification_id=state.config.qualification_id;
      identity.phase=BatchPhase::Accepted;
    }
    identity.minimum_native_dt_s=DBL_MAX;next.diagnostics=identity;
    next.diagnostics.parent_count=state.count;tile.proceed=true;
  }
  __syncthreads();
  const auto* prepared=initial ? nullptr : &view;
  for(std::size_t first=0;first<state.count;first+=Threads) {
    const auto remaining=state.count-first;const unsigned count=remaining<Threads ? unsigned(remaining) : Threads;
    if(lane<count)Read(tile,lane,state,trial,first+lane,identity.time,identity.epoch);
    __syncthreads();
    if(!lane)for(unsigned i=0;i<count;++i) {
      if(tile.status[i]||tile.valid[i]!=1) {
        next.status=tile.status[i] ? BatchStatus::ElementFailure : BatchStatus::NonfiniteResult;
        next.parent=first+i;next.element_status=tile.status[i];tile.proceed=false;break;
      }
      const auto row=Load(tile,i);
      if(!AccumulateMeasurementParent(next,state,accepted,first+i,row,prepared)) {tile.proceed=false;break;}
    }
    __syncthreads();
    if(!tile.proceed)break;
  }
  if(!lane) {if(tile.proceed)next.diagnostics.valid=true;state.control=next;}
}
}
