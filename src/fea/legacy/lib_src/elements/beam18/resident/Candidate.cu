// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Measure.h"
#include "measurement/Finalize.cuh"
#include <cfloat>

namespace tl::fea::beam18::batch_detail {
namespace {
__global__ void Initialize(Storage* s) {
  for (std::size_t p = blockIdx.x * blockDim.x + threadIdx.x; p < s->count; p += gridDim.x * blockDim.x) {
    const auto& parent = s->parents[p];
    s->status[p] = int(InitializeForce(parent.reference, s->materials[parent.material_index],
        s->config.startup.uniform_velocity, s->slab[0][p]));
    if (!s->status[p]) s->slab[1][p] = s->slab[0][p];
  }
}
__global__ void Evaluate(Storage* s, unsigned accepted, unsigned trial, NodalPreparedView view) {
  for (std::size_t p = blockIdx.x * blockDim.x + threadIdx.x; p < s->count; p += gridDim.x * blockDim.x) {
    const auto& parent = s->parents[p];
    beam_endpoint::Motion motion;
    if (!beam_endpoint::Gather(parent.domain_nodes, view.kinematics, motion)) {
      s->status[p] = int(Status::InvalidInput);
      continue;
    }
    PrescribedInterval interval;
    interval.base_time_s = view.base_time;
    interval.dt_s = s->config.owner.fixed_dt;
    interval.sample_index = view.kinematics.base_epoch + 1;
    for (unsigned n = 0; n < 2; ++n) {
      interval.position_endpoint_m[n] = motion.position[n];
      interval.velocity_midpoint_m_s[n] = motion.velocity[n];
      interval.angular_velocity_midpoint_rad_s[n] = motion.angular_velocity[n];
    }
    s->status[p] = int(EvaluateForce(parent.reference, s->materials[parent.material_index],
        s->slab[accepted][p].proposed_history, interval, s->slab[trial][p]));
  }
}
__global__ void Finalize(Storage* s, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity, bool initial) {
  __shared__ measurement::Tile tile;
  measurement::Finalize(*s,accepted,trial,view,identity,initial,tile);
}
} // namespace
void LaunchInitialize(Storage* s, cudaStream_t stream) {
  Initialize<<<64, 64, 0, stream>>>(s);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Finalize<<<1, measurement::Threads, 0, stream>>>(s, 0, 0, {}, {}, true);
}
void LaunchCandidate(Storage* s, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity) {
  Evaluate<<<64, 64, 0, view.stream>>>(s, accepted, trial, view);
  if (cudaPeekAtLastError() != cudaSuccess) return;
  Finalize<<<1, measurement::Threads, 0, view.stream>>>(s, accepted, trial, view, identity, false);
}
} // namespace tl::fea::beam18::batch_detail
