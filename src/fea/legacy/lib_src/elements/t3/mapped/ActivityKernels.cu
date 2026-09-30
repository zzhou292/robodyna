// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ActivityQuery.h"
namespace tl::fea::t3::mapped {
namespace {
__global__ void ValidateActivity(ActivityQuery query, ActivityPhase phase) {
  const auto parent = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (parent >= query.parents) return;
  const auto law = query.mixed->law[parent];
  const auto& reference = query.storage->model.element[parent].reference;
  const auto& force = query.storage->slab[query.slab].element[parent];
  const auto& plastic = query.mixed->plastic.section[query.slab][parent];
  auto& packet = query.storage->assembly.activity;
  packet.active[parent] = 0; // Every transferred byte is initialized in this query.
  ActivityError error = ActivityError::None;
  switch (phase) {
    case ActivityPhase::OnePoint:
      error = CheckPointActivity(law, query.point->section[query.slab][parent],
          query.mixed->plastic.parameters[parent], query.time);
      break;
    case ActivityPhase::Mixed:
      error = CheckMixedActivity(law, plastic, query.mixed->elastic_section[query.slab][parent],
          query.point != nullptr, query.execution);
      if (error == ActivityError::None) packet.active[parent] = static_cast<std::uint8_t>(law);
      break;
    case ActivityPhase::Failure: {
      const auto& failure = query.failure->state[query.slab][parent];
      error = CheckFailureActivity(law, query.failure->policy[parent], failure, plastic,
          query.point != nullptr, query.time);
      if (error == ActivityError::None) packet.active[parent] = law == ShellSectionLaw::Law44Nip1
          ? (query.point->section[query.slab][parent].point.failure.history.point_active ? 1 : 0)
          : (failure.active ? 1 : 0);
      break;
    }
    case ActivityPhase::Force:
      error = CheckForceActivity(reference, force, law, query.force_time, query.force_epoch);
      if (error == ActivityError::None)
        packet.active[parent] = static_cast<std::uint8_t>(force.proposed_history.data().active);
      break;
    case ActivityPhase::PointIdentity:
      error = CheckPointIdentity(reference, force, query.point->section[query.slab][parent],
          query.mixed->plastic.parameters[parent], law, query.time, query.epoch);
      break;
  }
  if (error != ActivityError::None)
    atomicMin(packet.first_invalid, ActivityKey(static_cast<std::uint32_t>(parent), error));
}
}
cudaError_t LaunchActivity(const ActivityQuery& query, ActivityPhase phase, cudaStream_t stream) {
  constexpr unsigned threads = 128;
  const auto blocks = static_cast<unsigned>((query.parents+threads-1)/threads);
  ValidateActivity<<<blocks,threads,0,stream>>>(query,phase);
  return cudaGetLastError();
}
} // namespace tl::fea::t3::mapped
