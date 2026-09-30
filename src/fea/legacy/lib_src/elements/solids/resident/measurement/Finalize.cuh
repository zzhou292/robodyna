// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Begin.h"
#include "Family.cuh"
namespace tl::fea::solids::batch_detail::measurement {
// Exactly one Threads-wide block. Each family returns the same shared decision
// on every lane, so short circuiting never leaves a lane inside a barrier.
__device__ inline void Finalize(Storage& state,NodalPreparedView view,
    BatchDiagnostics identity,bool initial,Tile& tile) noexcept {
  Control next{};
  if(!threadIdx.x) next=Begin(state,identity,initial);
  const auto* prepared=initial ? nullptr : &view;
  if(Measure<Traits18>(state,next,0,prepared,tile) &&
      Measure<Traits24>(state,next,1,prepared,tile) &&
      Measure<Traits6z>(state,next,2,prepared,tile) &&
      Measure<Traits18Law44>(state,next,3,prepared,tile) &&
      Measure<Traits18Law90>(state,next,4,prepared,tile)) {
    if(!threadIdx.x) next.diagnostics.valid=true;
  }
  if(!threadIdx.x) state.control=next;
}
} // namespace tl::fea::solids::batch_detail::measurement
