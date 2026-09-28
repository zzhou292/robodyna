// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Values.h"
#include "../normal_activation/Values.h"
#include <cub/cub.cuh>
namespace tlfea::contact::radioss_type25::activity_operands::detail {
namespace {
unsigned Blocks(std::size_t n) { return unsigned((n+127)/128); }
__device__ void Fail(Device d, Failure failure, std::size_t row) {
  atomicMin(&d.control->failure, (static_cast<unsigned long long>(failure)<<56) | row);
}
__global__ void Parents(Device d, tl::fea::PhysicalActivityDeviceView activity) {
  const auto i = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (i >= d.shape.parents) return;
  d.parent_active[i] = 0; d.parent_removed[i] = 0;
  const auto parent = d.parents[i]; std::uint8_t base = 1, current = 1;
  if (parent.family == activity_source::Family::Qeph) {
    base = activity.qeph.base[parent.family_index]; current = activity.qeph.current[parent.family_index];
  } else if (parent.family == activity_source::Family::T3) {
    base = activity.t3.base[parent.family_index]; current = activity.t3.current[parent.family_index];
  }
  if (base > 1 || current > 1 || current > base) { Fail(d, Failure::Mask, i); return; }
  d.parent_active[i] = current; d.parent_removed[i] = base && !current;
}
__global__ void Nodes(Device d) {
  const auto node = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (node >= d.shape.nodes) return;
  bool active = false;
  for (auto j = d.node_offsets[node]; j < d.node_offsets[node+1]; ++j)
    active = active || d.parent_active[d.node_parents[j]];
  d.node_active[node] = active;
}
__global__ void Events(Device d) {
  if (d.control->failure != UINT64_MAX || d.controls.deletion == activity_source::Deletion::Disabled) return;
  const auto parent = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (parent >= d.shape.parents || !d.parent_removed[parent]) return;
  for (auto j = d.emitting_offsets[parent]; j < d.emitting_offsets[parent+1]; ++j) {
    // FIND_SURFACE_INTER produces ONE-based expanded registered main IDs.
    atomicAdd(d.events + d.emitting_mains[j]-1, 1u);
    atomicAdd(&d.control->affected, 1ull);
  }
}
__global__ void Mains(Device d, unsigned accepted) {
  const auto i = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (i >= d.shape.mains) return;
  const auto& old = d.slots[accepted]; auto& next = d.slots[accepted^1u];
  const auto primary = d.main_to_primary[i]; bool supported = false;
  for (auto j = d.containing_offsets[primary]; j < d.containing_offsets[primary+1]; ++j)
    supported = supported || d.parent_active[d.containing_parents[j]];
  const auto value = Main(old.mains[i].coefficient, old.connected[i], d.events[i], supported, d.controls);
  if (!value.valid) { Fail(d, Failure::Counter, i); return; }
  if (value.exposure) { Fail(d, Failure::Exposure, i); return; }
  next.mains[i] = old.mains[i]; next.mains[i].coefficient = value.coefficient;
  next.connected[i] = value.connected; d.removed[i] = value.removed;
  if (value.removed) {
    atomicAdd(&d.control->removed_events, static_cast<unsigned long long>(d.events[i]));
    if (old.mains[i].coefficient != 0) atomicAdd(&d.control->removed_mains, 1u);
  }
  if (!Same(value.coefficient, old.mains[i].coefficient) || value.connected != old.connected[i])
    atomicExch(&d.control->changed, 1u);
  if (d.shape.normals) next.normal_coefficients[i] = value.coefficient;
  if (i < d.shape.primaries && !Scale(value.coefficient, d.units.stiffness, next.main_stiffness_si[i]))
    Fail(d, Failure::Scale, i);
}
__global__ void Neighbors(Device d, unsigned accepted) {
  if (d.control->failure != UINT64_MAX) return;
  const auto i = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (i >= d.shape.mains) return;
  const auto& old = d.slots[accepted]; auto& next = d.slots[accepted^1u];
  for (unsigned k = 0; k < 4; ++k) {
    const int neighbor = old.mains[i].neighbors[k];
    if (neighbor > 0 && d.removed[neighbor-1]) {
      next.mains[i].neighbors[k] = 0; atomicExch(&d.control->changed, 1u);
    }
  }
  if (d.shape.normals) {
    next.normal_mains[i] = old.normal_mains[i];
    for (unsigned k = 0; k < 4; ++k) {
      next.normal_mains[i].neighbors[k] = next.mains[i].neighbors[k];
      if (!next.mains[i].neighbors[k]) next.normal_mains[i].neighbor_edges[k] = 0;
    }
  }
  if (d.shape.normals) {
    d.flags[i] = normal_activation::detail::FreeMain(next.mains[i]) ? 1 : 0;
    if (!i) d.flags[d.shape.mains] = 0;
  }
}
__global__ void Secondaries(Device d, unsigned accepted) {
  const auto i = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (i >= d.shape.secondaries) return;
  const double old = d.slots[accepted].secondary_coefficients[i];
  if (!tl::math::Finite(old) || old < 0) { Fail(d, Failure::Source, i); return; }
  const auto marked = MarkSecondary(old, d.node_active[d.secondary_nodes[i]], d.controls);
  const auto value = NormalizeSecondary(marked);
  auto& next = d.slots[accepted^1u]; next.secondary_coefficients[i] = value;
  if (!Scale(value, d.units.stiffness, next.secondary_stiffness_si[i])) { Fail(d, Failure::Scale, i); return; }
  if (marked < 0) atomicAdd(&d.control->orphans, 1u);
  if (!Same(old, value)) atomicExch(&d.control->changed, 1u);
}
__global__ void FreeRoster(Device d, unsigned trial) {
  if (d.control->failure != UINT64_MAX) return;
  const auto i = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (i < d.shape.mains && d.flags[i]) d.slots[trial].free_mains[d.offsets[i]] = std::uint32_t(i+1);
  if (!i) d.control->free_count = d.offsets[d.shape.mains];
}
}
cudaError_t ScanBytes(std::size_t mains, std::size_t& bytes) noexcept {
  return cub::DeviceScan::ExclusiveSum(nullptr, bytes, static_cast<std::uint32_t*>(nullptr),
      static_cast<std::uint32_t*>(nullptr), int(mains+1));
}
cudaError_t Launch(Device d, unsigned accepted, const tl::fea::PhysicalActivityDeviceView& activity,
    cudaStream_t stream) noexcept {
#define RUN(expr) expr; { const auto error = cudaGetLastError(); if (error != cudaSuccess) return error; }
  RUN((Parents<<<Blocks(d.shape.parents),128,0,stream>>>(d, activity)))
  RUN((Nodes<<<Blocks(d.shape.nodes),128,0,stream>>>(d)))
  RUN((Events<<<Blocks(d.shape.parents),128,0,stream>>>(d)))
  RUN((Mains<<<Blocks(d.shape.mains),128,0,stream>>>(d, accepted)))
  RUN((Neighbors<<<Blocks(d.shape.mains),128,0,stream>>>(d, accepted)))
  RUN((Secondaries<<<Blocks(d.shape.secondaries),128,0,stream>>>(d, accepted)))
  if (!d.shape.normals) return cudaSuccess;
  auto bytes = d.scan_bytes;
  const auto error = cub::DeviceScan::ExclusiveSum(d.scan, bytes, d.flags, d.offsets, int(d.shape.mains+1), stream);
  if (error != cudaSuccess) return error;
  RUN((FreeRoster<<<Blocks(d.shape.mains),128,0,stream>>>(d, accepted^1u)))
#undef RUN
  return cudaSuccess;
}
TransactionReport Decode(const Control& control) noexcept {
  if (control.failure == UINT64_MAX) return {TransactionStatus::Ok, "Contact operands staged"};
  const auto kind = static_cast<Failure>(control.failure>>56);
  const auto row = std::size_t(control.failure & 0x00ffffffffffffffull);
  if (kind == Failure::Exposure) return {TransactionStatus::UnsupportedProfile, "Native solid-surface exposure requires separate topology support", row};
  if (kind == Failure::Mask) return {TransactionStatus::ActivityChange, "Physical activity is invalid or reactivated", row};
  return {TransactionStatus::NumericalFailure, "Contact activity operand validation failed", row};
}
} // namespace tlfea::contact::radioss_type25::activity_operands::detail
