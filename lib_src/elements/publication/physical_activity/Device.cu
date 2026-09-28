// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Device.h"
#include "../../qeph/mapped/MixedActivityValues.h"
#include "../../qeph/mapped/FailureActivityValues.h"
#include "../../qeph/mapped/Result.h"
#include "../../t3/mapped/ActivityValues.h"
#include "../../ShellMixedSectionArenaLayout.h"
#include "../../failure/ShellFailureArenaLayout.h"
#include "../../one_point/ShellOnePointArenaLayout.h"
namespace tl::fea::physical_activity {
namespace {
using Stage = PhysicalActivityStage;
__device__ bool Error(DeviceOutput out, Stage stage, std::size_t parent, unsigned detail) {
  const auto key = (static_cast<unsigned long long>(stage) << 56) |
      (static_cast<unsigned long long>(parent) << 16) | detail;
  atomicMin(&out.control->first_error, key);
  return false;
}
__device__ void Finish(DeviceOutput out, std::size_t parent, std::uint8_t active) {
  if (active > 1 || (out.accepted && out.accepted[parent] > 1)) {
    Error(out, Stage::Transition, parent, 2); return;
  }
  if (out.accepted && active > out.accepted[parent]) {
    Error(out, Stage::Transition, parent, 1); return;
  }
  out.staging[parent] = active;
  if (active) atomicAdd(&out.control->active, 1u);
  else atomicMin(&out.control->first_inactive, static_cast<unsigned>(parent));
  if (out.accepted && out.accepted[parent] && !active) {
    atomicAdd(&out.control->removed, 1u);
    atomicMin(&out.control->first_removed, static_cast<unsigned>(parent));
  }
}
__global__ void Qeph(QephInput input, DeviceOutput out) {
  const auto parent = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  if (parent >= input.count) return;
  const auto law = input.mixed->law[parent];
  const auto& plastic = input.mixed->plastic.section[input.slab][parent];
  const auto mixed = qeph::mapped::CheckMixedActivity(law, plastic,
      input.mixed->elastic_section[input.slab][parent]);
  if (mixed != qeph::mapped::MixedActivityError::None) {
    Error(out, Stage::Mixed, parent, static_cast<unsigned>(mixed)); return;
  }
  const auto& failure = input.failure->state[input.slab][parent];
  if (!qeph::mapped::ValidFailureActivity(failure, input.failure->policy[parent],
      law == ShellSectionLaw::LayeredLaw44Nip3 ? &plastic : nullptr, input.time)) {
    Error(out, Stage::Failure, parent, 1); return;
  }
  const auto& force = input.results[parent];
  if (!qeph::mapped::ValidResult(input.elements[parent].reference, force,
      input.force_time, input.force_epoch, law == ShellSectionLaw::RigidSkin)) {
    Error(out, Stage::Force, parent, 1); return;
  }
  const auto active = static_cast<std::uint8_t>(force.proposed_history.data().active);
  if (out.expected_law[parent] != static_cast<std::uint8_t>(law) ||
      active != (failure.active ? 1 : 0)) {
    Error(out, Stage::Agreement, parent, 1); return;
  }
  Finish(out, parent, active);
}
__global__ void T3(T3Input input, DeviceOutput out) {
  const auto parent = std::size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  if (parent >= input.count) return;
  namespace a = t3::mapped;
  const auto law = input.mixed->law[parent];
  if (input.point) {
    const auto error = a::CheckPointActivity(law, input.point->section[input.slab][parent],
        input.mixed->plastic.parameters[parent], input.time);
    if (error != a::ActivityError::None) { Error(out, Stage::Point, parent, unsigned(error)); return; }
  }
  const auto& plastic = input.mixed->plastic.section[input.slab][parent];
  auto error = a::CheckMixedActivity(law, plastic, input.mixed->elastic_section[input.slab][parent],
      input.point != nullptr, input.execution);
  if (error != a::ActivityError::None) { Error(out, Stage::Mixed, parent, unsigned(error)); return; }
  const auto& failure = input.failure->state[input.slab][parent];
  error = a::CheckFailureActivity(law, input.failure->policy[parent], failure, plastic,
      input.point != nullptr, input.time);
  if (error != a::ActivityError::None) { Error(out, Stage::Failure, parent, unsigned(error)); return; }
  const auto& force = input.results[parent];
  const auto& reference = input.elements[parent].reference;
  error = a::CheckForceActivity(reference, force, law, input.force_time, input.force_epoch);
  if (error != a::ActivityError::None) { Error(out, Stage::Force, parent, unsigned(error)); return; }
  const bool expected = law == ShellSectionLaw::Law44Nip1
      ? input.point->section[input.slab][parent].point.failure.history.point_active : failure.active;
  const auto active = static_cast<std::uint8_t>(force.proposed_history.data().active);
  if (out.expected_law[parent] != static_cast<std::uint8_t>(law) || active != (expected ? 1 : 0)) {
    Error(out, Stage::Agreement, parent, 1); return;
  }
  if (input.point) {
    error = a::CheckPointIdentity(reference, force, input.point->section[input.slab][parent],
        input.mixed->plastic.parameters[parent], law, input.time, input.epoch);
    if (error != a::ActivityError::None) { Error(out, Stage::PointIdentity, parent, unsigned(error)); return; }
  }
  Finish(out, parent, active);
}
}
cudaError_t Capture(const QephInput& input, DeviceOutput out, cudaStream_t stream) noexcept {
  Qeph<<<unsigned((input.count + 127) / 128), 128, 0, stream>>>(input, out);
  return cudaGetLastError();
}
cudaError_t Capture(const T3Input& input, DeviceOutput out, cudaStream_t stream) noexcept {
  T3<<<unsigned((input.count + 127) / 128), 128, 0, stream>>>(input, out);
  return cudaGetLastError();
}
} // namespace tl::fea::physical_activity
