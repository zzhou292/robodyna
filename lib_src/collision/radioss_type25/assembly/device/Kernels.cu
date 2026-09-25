// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Launch.h"
#include "../Schedule.h"
#include <cub/device/device_radix_sort.cuh>
namespace tlfea::contact::radioss_type25::assembly::device_detail {
namespace {
unsigned Blocks(std::size_t size) { return static_cast<unsigned>((size+255)/256); }
__global__ void CheckCohorts(Device d, Schedule schedule) {
  const auto at = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (at >= schedule.cohort_count) return;
  const auto first = at ? schedule.cohort_ends[at-1] : 0u;
  const auto end = schedule.cohort_ends[at];
  if (end <= first || end > schedule.row_count ||
      (at+1 == schedule.cohort_count && end != schedule.row_count))
    atomicMin(d.failure,static_cast<unsigned long long>(at));
}
__global__ void Keys(Device d, DeviceConnectivity in) {
  const auto rank = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (rank >= 5*in.schedule.row_count) return;
  d.keys[rank] = UINT64_MAX;
  // Decode bounds every local read even when a malformed schedule already
  // failed validation. Do not mix ordinary reads with concurrent atomic writes
  // of the diagnostic word. The host rejects the entire result after draining.
  Occurrence at;
  if (!DecodeOccurrence(in.schedule,static_cast<std::uint32_t>(rank),&at)) {
    atomicMin(d.failure,(1ull<<56)|static_cast<unsigned long long>(rank)); return;
  }
  const auto node = EndpointNode(in.rows[at.row],at.slot);
  if (node >= in.nodes) {
    atomicMin(d.failure,(1ull<<56)|static_cast<unsigned long long>(rank)); return;
  }
  d.keys[rank] = (std::uint64_t(node)<<32) | std::uint32_t(rank);
}
__global__ void DecodeRanks(Device d, std::size_t occurrences) {
  const auto index = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (index < occurrences) d.occurrences[index] = static_cast<std::uint32_t>(d.sorted_keys[index]);
}
__global__ void Offsets(Device d, std::size_t nodes, std::size_t occurrences) {
  const auto node = std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if (node > nodes) return;
  std::size_t lo = 0, hi = occurrences;
  const auto key = std::uint64_t(node)<<32;
  while (lo < hi) {
    const auto mid = lo+(hi-lo)/2;
    if (d.sorted_keys[mid] < key) lo = mid+1;
    else hi = mid;
  }
  d.offsets[node] = static_cast<std::uint32_t>(lo);
}
}
cudaError_t QueryScratch(IncidenceLimits limits, std::size_t& bytes) noexcept {
  if (!limits.max_rows) { bytes = 0; return cudaSuccess; }
  return cub::DeviceRadixSort::SortKeys(nullptr,bytes,static_cast<std::uint64_t*>(nullptr),
      static_cast<std::uint64_t*>(nullptr),int(5*limits.max_rows));
}
cudaError_t Build(Device d, const DeviceConnectivity& in, cudaStream_t stream,
    IncidenceReport& report) noexcept {
  auto error = cudaMemsetAsync(d.failure,0xff,sizeof(*d.failure),stream);
  if (error != cudaSuccess) return error;
  if (in.schedule.cohort_count) {
    ++report.own_kernel_launches;
    CheckCohorts<<<Blocks(in.schedule.cohort_count),256,0,stream>>>(d,in.schedule);
    error = cudaPeekAtLastError(); if (error != cudaSuccess) return error;
  }
  const auto occurrences = 5*in.schedule.row_count;
  if (occurrences) {
    ++report.own_kernel_launches;
    Keys<<<Blocks(occurrences),256,0,stream>>>(d,in);
    error = cudaPeekAtLastError(); if (error != cudaSuccess) return error;
    auto bytes = d.cub_bytes;
    ++report.sort_calls;
    error = cub::DeviceRadixSort::SortKeys(d.cub,bytes,d.keys,d.sorted_keys,int(occurrences),0,64,stream);
    if (error != cudaSuccess) return error;
    ++report.own_kernel_launches;
    DecodeRanks<<<Blocks(occurrences),256,0,stream>>>(d,occurrences);
    error = cudaPeekAtLastError(); if (error != cudaSuccess) return error;
  }
  ++report.own_kernel_launches;
  Offsets<<<Blocks(in.nodes+1),256,0,stream>>>(d,in.nodes,occurrences);
  return cudaPeekAtLastError();
}
} // namespace tlfea::contact::radioss_type25::assembly::device_detail
