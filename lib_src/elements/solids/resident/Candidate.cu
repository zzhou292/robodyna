// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Measure.h"
#include <limits>

namespace tl::fea::solids::batch_detail {
namespace {
template<class Traits> __global__ void Initialize(Storage* storage) {
  auto& family = FamilyStorage<Traits>(*storage);
  for (std::size_t p = threadIdx.x; p < family.count; p += blockDim.x) {
    const auto& parent = family.parents[p];
    family.status[p] = InitializeState<Traits>(parent,
        MaterialAt<Traits>(*storage, parent.material_index), storage->config.startup.uniform_velocity,
        family.slab[0][p]);
    if (!family.status[p]) family.slab[1][p] = family.slab[0][p];
  }
}
template<class Traits> __global__ void Evaluate(Storage* storage, unsigned accepted,
    unsigned trial, NodalPreparedView view) {
  auto& family = FamilyStorage<Traits>(*storage);
  const auto first = blockIdx.x * blockDim.x + threadIdx.x;
  const auto stride = gridDim.x * blockDim.x;
  for (std::size_t p = first; p < family.count; p += stride) {
    const auto& parent = family.parents[p];
    auto interval = Traits::Phase(view.base_time, storage->config.owner.fixed_dt,
        view.kinematics.base_epoch);
    for (unsigned n = 0; n < Traits::nodes; ++n) {
      const auto node = parent.domain_nodes[n];
      Traits::Node(interval, n, shell_batch_fields::ReadVector(view.kinematics.position_xyz, node),
          shell_batch_fields::ReadVector(view.kinematics.velocity_xyz, node));
    }
    family.status[p] = UpdateState<Traits>(parent,
        MaterialAt<Traits>(*storage, parent.material_index), family.slab[accepted][p],
        interval, family.slab[trial][p]);
  }
}
__global__ void Finalize(Storage* storage, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity, bool initial) {
  auto& state = *storage;
  state.control = {};
  if (initial) {
    identity.source_instance_id = state.source_instance_id;
    identity.owner_id = state.config.owner.owner_id;
    identity.configuration_id = state.config.configuration_id;
    identity.qualification_id = state.config.qualification_id;
    identity.phase = BatchPhase::Accepted;
  }
  identity.minimum_native_dt_s = std::numeric_limits<double>::max();
  state.control.diagnostics = identity;
  const auto* prepared = initial ? nullptr : &view;
  if (!MeasureFamily<Traits18>(state, accepted, trial, 0, prepared) ||
      !MeasureFamily<Traits24>(state, accepted, trial, 1, prepared) ||
      !MeasureFamily<Traits6z>(state, accepted, trial, 2, prepared)) return;
  state.control.diagnostics.valid = true;
}
} // namespace
void LaunchInitialize(Storage* storage, cudaStream_t stream) {
  Initialize<Traits18><<<1, 64, 0, stream>>>(storage);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Initialize<Traits24><<<1, 64, 0, stream>>>(storage);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Initialize<Traits6z><<<1, 64, 0, stream>>>(storage);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Finalize<<<1, 1, 0, stream>>>(storage, 0, 0, {}, {}, true);
}
void LaunchCandidate(Storage* storage, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity) {
  Evaluate<Traits18><<<64, 64, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Evaluate<Traits24><<<64, 64, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Evaluate<Traits6z><<<64, 64, 0, view.stream>>>(storage, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Finalize<<<1, 1, 0, view.stream>>>(storage, accepted, trial, view, identity, false);
}
} // namespace tl::fea::solids::batch_detail
