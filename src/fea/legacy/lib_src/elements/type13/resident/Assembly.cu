// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "AssemblyValues.h"
#include "../../mapped_connector/Kernels.cuh"
#include "../../ShellBatchFields.h"
#include "../../../solvers/NodalForceAssembly.h"

namespace tl::fea::type13::batch_detail {
namespace {
namespace contact = tlfea::contact;

__device__ bool CheckEndpoints(Storage& state, const NodalAssemblyView& view,
                               bool initial, bool mapped) {
  for (std::size_t e = 0; e < state.model.element_count; ++e) {
    const auto& element = state.model.elements[e];
    for (unsigned local = 0; local < 2; ++local) {
      if (!ValidEndpoint(state.model, element, local, view, initial, mapped)) {
        state.control.status = BatchStatus::InvalidInput;
        state.control.element = e;
        state.control.node = element.nodes[local];
        return false;
      }
    }
  }
  return true;
}

__device__ bool AddElement(const DeviceElement& element, const Evaluation& value,
                           NodalAssemblyView view, NodalCinAssemblyView cin,
                           bool stiffness) {
  double next_translation[2]{}, next_rotation[2]{};
  if (stiffness) {
    double translation = 0, rotation = 0;
    if (!EndpointStiffness(value, translation, rotation)) {
      return false;
    }
    for (unsigned local = 0; local < 2; ++local) {
      const auto node = element.nodes[local];
      const double old_translation = cin.translational_stiffness[node];
      const double old_rotation = cin.rotational_stiffness[node];
      next_translation[local] = old_translation + translation;
      next_rotation[local] = old_rotation + rotation;
      if (!detail::Nonnegative(old_translation) || !detail::Nonnegative(old_rotation) ||
          !detail::Nonnegative(next_translation[local]) ||
          !detail::Nonnegative(next_rotation[local])) {
        return false;
      }
    }
  }
  const Vec3 force[2] = {value.endpoints[0].force_N, value.endpoints[1].force_N};
  const Vec3 couple[2] = {value.endpoints[0].couple_Nm, value.endpoints[1].couple_Nm};
  if (AccumulateNodalForces<2>(element.nodes, force, couple, view.forces, +1) !=
      NodalForceAssemblyStatus::Success) {
    return false;
  }
  if (stiffness) {
    for (unsigned local = 0; local < 2; ++local) {
      const auto node = element.nodes[local];
      cin.translational_stiffness[node] = next_translation[local];
      cin.rotational_stiffness[node] = next_rotation[local];
    }
  }
  return true;
}

__global__ void Assemble(Storage* storage, unsigned accepted,
                         NodalAssemblyView view, NodalCinAssemblyView cin,
                         bool initial, bool mapped) {
  auto& state = *storage;
  state.control = {};
  if (view.result->base_epoch != view.accepted.base_epoch ||
      view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch ||
      view.bounds->attempt != view.attempt || !view.bounds->initialized ||
      !view.bounds->valid || view.bounds->sealed ||
      view.result->status != contact::Status::kOk) {
    state.control.status = BatchStatus::AssemblyFailure;
  } else if (CheckEndpoints(state, view, initial, mapped)) {
    const bool stiffness = state.model.config.assembly == BatchAssembly::CinNativeStiffness;
    // One writer preserves each node's model-then-endpoint scalar order. No
    // atomics, inverse-coefficient inference or runtime neighbor map is needed.
    for (std::size_t e = 0; e < state.model.element_count; ++e) {
      if (!AddElement(state.model.elements[e], state.slab[accepted][e], view, cin, stiffness)) {
        state.control.status = BatchStatus::AssemblyFailure;
        state.control.element = e;
        break;
      }
    }
  }
  if (state.control.status != BatchStatus::Success) {
    RecordNodalAssemblyFailure(view, contact::Status::kInvalidArgument,
                               static_cast<std::uint32_t>(state.control.node));
  }
}

__global__ void Failure(NodalAssemblyView view) {
  RecordNodalAssemblyFailure(view, contact::Status::kInvalidArgument);
}
} // namespace

void LaunchAssembly(Storage* storage, unsigned accepted, NodalAssemblyView view,
                     NodalCinAssemblyView cin, bool initial, bool mapped) {
  if (mapped) mapped_connector::Launch<AssemblyFamily>(storage, accepted, view, cin, initial);
  else Assemble<<<1, 1, 0, view.stream>>>(storage, accepted, view, cin, initial, false);
}
void LaunchFailure(NodalAssemblyView view) {
  Failure<<<1, 1, 0, view.stream>>>(view);
}
} // namespace tl::fea::type13::batch_detail
