// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Kinematics.h"
#include "Measure.h"
#include "../Type13Math.h"

namespace tl::fea::type13::batch_detail {
namespace {
__global__ void EvaluateElements(Storage* storage, unsigned accepted,
                                 unsigned trial, NodalPreparedView view) {
  auto& state = *storage;
  const std::size_t first = blockIdx.x * blockDim.x + threadIdx.x;
  const std::size_t stride = gridDim.x * blockDim.x;
  for (std::size_t e = first; e < state.model.element_count; e += stride) {
    const auto& element = state.model.elements[e];
    NativeEndpointKinematics nodes[2];
    bool finite = true;
    for (unsigned local = 0; local < 2; ++local) {
      const auto node = element.nodes[local];
      finite = FromSI(state.model.units,
          shell_batch_fields::ReadVector(view.kinematics.position_xyz, node),
          shell_batch_fields::ReadVector(view.kinematics.velocity_xyz, node),
          shell_batch_fields::ReadVector(view.kinematics.angular_velocity_xyz, node),
          nodes[local]) && finite;
    }
    const double dt = state.model.config.owner.fixed_dt / state.model.units.time_to_s;
    state.candidate_status[e] = finite
        ? Evaluate(state.model.properties[element.property], element.reference,
                   state.slab[accepted][e].native_history, nodes, dt, state.slab[trial][e])
        : Status::NonfiniteResult;
  }
}

__global__ void Finalize(Storage* storage, unsigned accepted, unsigned trial,
                         NodalPreparedView view, BatchDiagnostics identity) {
  auto& state = *storage;
  state.control = {};
  state.control.diagnostics = identity;
  for (std::size_t e = 0; e < state.model.element_count; ++e) {
    if (state.candidate_status[e] != Status::Success) {
      state.control.status = BatchStatus::ElementFailure;
      state.control.element = e;
      state.control.element_status = state.candidate_status[e];
      return;
    }
  }
  if (!Measure(state.model, state.slab[accepted], state.slab[trial], view,
               state.control.diagnostics)) {
    state.control.status = BatchStatus::NonfiniteResult;
    return;
  }
  state.control.diagnostics.valid = true;
}
} // namespace

void LaunchCandidate(Storage* storage, unsigned accepted, unsigned trial,
                      NodalPreparedView view, BatchDiagnostics identity,
                      std::size_t count) {
  constexpr unsigned threads = 64;
  const unsigned blocks = 1 + static_cast<unsigned>((count - 1) / threads);
  EvaluateElements<<<blocks, threads, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) {
    return;
  }
  Finalize<<<1, 1, 0, view.stream>>>(storage, accepted, trial, view, identity);
}
} // namespace tl::fea::type13::batch_detail
