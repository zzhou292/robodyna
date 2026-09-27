// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "MeasurementValues.h"
#include "measurement/Finalize.cuh"
#include <cfloat>

namespace tl::fea::solids::batch_detail {
namespace {
template<class Traits> __global__ void Initialize(Storage* storage) {
  auto& family = FamilyStorage<Traits>(*storage);
  for (std::size_t p = threadIdx.x; p < family.count; p += blockDim.x) {
    if(ControlledIndex<Traits>(*storage,p)!=SIZE_MAX)continue;
    const auto& parent = family.parents[p];
    if constexpr (std::is_same_v<Traits, Traits18>) {
      family.status[p] = InitializeState18(parent,
          MaterialAt<Traits>(*storage, parent.material_index),
          storage->config.startup.uniform_velocity, storage->scratch18[threadIdx.x],
          family.slab[0][p]);
    } else if constexpr (std::is_same_v<Traits, Traits18Law44> ||
        std::is_same_v<Traits, Traits18Law90>) {
      family.status[p] = InitializeExtendedState<Traits>(parent,
          MaterialAt<Traits>(*storage, parent.material_index),
          storage->config.startup.uniform_velocity, Scratch<Traits>(*storage, threadIdx.x),
          family.slab[0][p]);
    } else {
      family.status[p] = InitializeState<Traits>(parent,
          MaterialAt<Traits>(*storage, parent.material_index), storage->config.startup.uniform_velocity,
          family.slab[0][p]);
    }
    if (!family.status[p]) family.slab[1][p] = family.slab[0][p];
  }
}
template<class Traits> __global__ void Evaluate(Storage* storage, unsigned accepted,
    unsigned trial, NodalPreparedView view) {
  auto& family = FamilyStorage<Traits>(*storage);
  const auto first = blockIdx.x * blockDim.x + threadIdx.x;
  const auto stride = gridDim.x * blockDim.x;
  for (std::size_t p = first; p < family.count; p += stride) {
    if(ControlledIndex<Traits>(*storage,p)!=SIZE_MAX)continue;
    const auto& parent = family.parents[p];
    auto interval = Traits::Phase(view.base_time, storage->config.owner.fixed_dt,
        view.kinematics.base_epoch);
    for (unsigned n = 0; n < Traits::nodes; ++n) {
      const auto node = parent.domain_nodes[n];
      Traits::Node(interval, n, shell_batch_fields::ReadVector(view.kinematics.position_xyz, node),
          shell_batch_fields::ReadVector(view.kinematics.velocity_xyz, node));
    }
    if constexpr (std::is_same_v<Traits, Traits18>) {
      family.status[p] = UpdateState18(parent,
          MaterialAt<Traits>(*storage, parent.material_index), family.slab[accepted][p],
          interval, storage->scratch18[first], family.slab[trial][p]);
    } else if constexpr (std::is_same_v<Traits, Traits18Law44> ||
        std::is_same_v<Traits, Traits18Law90>) {
      family.status[p] = UpdateExtendedState<Traits>(parent,
          MaterialAt<Traits>(*storage, parent.material_index), family.slab[accepted][p],
          interval, Scratch<Traits>(*storage, first), family.slab[trial][p]);
    } else {
      family.status[p] = UpdateState<Traits>(parent,
          MaterialAt<Traits>(*storage, parent.material_index), family.slab[accepted][p],
          interval, family.slab[trial][p]);
    }
  }
}
__global__ void Finalize(Storage* storage, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity, bool initial, bool operands_prepared = false) {
  if (operands_prepared) {
    __shared__ measurement::Tile tile;
    measurement::Finalize(*storage,view,identity,initial,tile);
    return;
  }
  if (threadIdx.x) return;
  auto& state = *storage;
  auto next=measurement::Begin(state,identity,initial);
  const auto* prepared = initial ? nullptr : &view;
  if (MeasureFinalFamily<Traits18>(state, next, accepted, trial, 0, prepared, operands_prepared) &&
      MeasureFinalFamily<Traits24>(state, next, accepted, trial, 1, prepared, operands_prepared) &&
      MeasureFinalFamily<Traits6z>(state, next, accepted, trial, 2, prepared, operands_prepared) &&
      MeasureFinalFamily<Traits18Law44>(state, next, accepted, trial, 3, prepared, operands_prepared) &&
      MeasureFinalFamily<Traits18Law90>(state, next, accepted, trial, 4, prepared, operands_prepared)) {
    next.diagnostics.valid = true;
  }
  state.control = next;
}
} // namespace
void LaunchInitialize(Storage* storage, cudaStream_t stream) {
  Initialize<Traits18><<<1, candidate_threads, 0, stream>>>(storage);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Initialize<Traits24><<<1, candidate_threads, 0, stream>>>(storage);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Initialize<Traits6z><<<1, candidate_threads, 0, stream>>>(storage);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Initialize<Traits18Law44><<<1, candidate_threads, 0, stream>>>(storage);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Initialize<Traits18Law90><<<1, candidate_threads, 0, stream>>>(storage);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  LaunchControlledInitialize(storage,stream);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  LaunchMeasurementValidation(storage, 0, 0, {}, 0, 0, true, stream);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Finalize<<<1, measurement::Threads, 0, stream>>>(storage, 0, 0, {}, {}, true, true);
}
void LaunchCandidate(Storage* storage, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity) {
  Evaluate<Traits18><<<candidate_blocks, candidate_threads, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Evaluate<Traits24><<<candidate_blocks, candidate_threads, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Evaluate<Traits6z><<<candidate_blocks, candidate_threads, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Evaluate<Traits18Law44><<<candidate_blocks, candidate_threads, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Evaluate<Traits18Law90><<<candidate_blocks, candidate_threads, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  LaunchControlledCandidate(storage,accepted,trial,view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  LaunchMeasurementValidation(storage, accepted, trial, view, identity.time, identity.epoch, false, view.stream);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Finalize<<<1, measurement::Threads, 0, view.stream>>>(storage, accepted, trial, view, identity, false, true);
}
} // namespace tl::fea::solids::batch_detail
