// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchStorage.h"
#include "Type25BatchMeasurement.h"
namespace tl::fea::type25::batch_detail {
namespace {
__global__ void PrepareMeasurementParents(Storage* state, const Slab* accepted,
    const Slab* trial, NodalPreparedView view) {
  for (std::size_t e = blockIdx.x * blockDim.x + threadIdx.x;
       e < state->model.config.element_count; e += blockDim.x * gridDim.x)
    state->measurement[e] = PrepareMeasurement(*state, *accepted, *trial, view, e);
}
}
void LaunchMeasurement(Storage* state, const Slab* accepted, const Slab* trial,
    NodalPreparedView view, std::size_t count) {
  constexpr unsigned threads = 64;
  const unsigned blocks = 1 + static_cast<unsigned>((count - 1) / threads);
  PrepareMeasurementParents<<<blocks, threads, 0, view.stream>>>(state, accepted, trial, view);
}
} // namespace tl::fea::type25::batch_detail
