// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ObserverSums.h"

namespace tl::fea::mapped_shell {
__device__ inline void ReduceObserverBlock(ObserverSummary* values) {
  for (unsigned offset = ObserverThreads / 2; offset; offset /= 2) {
    __syncthreads();
    if (threadIdx.x < offset) MergeObservations(values[threadIdx.x], values[threadIdx.x + offset]);
  }
  __syncthreads();
}
} // namespace tl::fea::mapped_shell
