// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Access.h"
#include "Fixture.h"
#include <array>
namespace initial_source_test {
// Qualification-only transient inventory readback. Ordinary Prepare remains
// the seed producer; no pair buffer or callback is added to its public API.
inline std::vector<std::array<int,2>> CapturePairs(const src::PreparedSource& prepared,
    src::Limits limits,cudaStream_t stream) {
  namespace d=src::detail;
  const auto require=[](bool ok){if(!ok)throw std::runtime_error("Bounded qualification pair capture failed");};
  const auto check=[&](cudaError_t status){require(status==cudaSuccess);};
  const auto& owned=n::qualification::InitialSourceAccess::Host(prepared);
  const auto forecast=src::Preflight(owned.descriptor,limits);require(forecast.status==src::Status::Ok);
  d::Layout layout;require(d::MakeLayout(owned.descriptor,limits,forecast.cub_bytes,layout)==src::Status::Ok);
  struct Arena{void* data=nullptr;~Arena(){if(data)cudaFree(data);}};
  struct Drain{cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}};
  Arena work,sweep,seed;Drain drain{stream};
  check(cudaMalloc(&work.data,layout.forecast.source_device_bytes));
  check(cudaMalloc(&sweep.data,layout.forecast.sweep_device_bytes));check(cudaMalloc(&seed.data,layout.seed.bytes));
  check(cudaMemsetAsync(work.data,0,layout.forecast.source_device_bytes,stream));
  check(cudaMemsetAsync(sweep.data,0,layout.forecast.sweep_device_bytes,stream));
  const auto device=d::Bind(work.data,sweep.data,seed.data,owned.descriptor,owned,layout,limits);
  d::Control control;n::candidates::detail::Control counters;
  const auto read=[&]() {
    check(cudaMemcpyAsync(&control,device.control,sizeof(control),cudaMemcpyDeviceToHost,stream));
    check(cudaMemcpyAsync(&counters,device.sweep.control,sizeof(counters),cudaMemcpyDeviceToHost,stream));
    check(cudaStreamSynchronize(stream));require(control.failure==~0ull&&counters.failure==~0ull);
  };
  check(d::Upload(owned,device,stream));check(d::PrepareOperands(device,stream));
  check(d::BuildRanges(device,stream));read();require(counters.tasks<=limits.max_tasks);
  check(d::CountPairs(device,counters.tasks,stream));read();require(counters.pairs<=limits.max_pairs);
  check(d::FillPairs(device,counters.tasks,counters.pairs,stream));read();
  std::vector<n::candidates::Pair> raw(counters.pairs);
  if(!raw.empty())check(cudaMemcpyAsync(raw.data(),device.sweep.pairs,raw.size()*sizeof(raw[0]),cudaMemcpyDeviceToHost,stream));
  check(cudaStreamSynchronize(stream));std::vector<std::array<int,2>> result(raw.size());
  for(std::size_t i=0;i<raw.size();++i)result[i]={int(raw[i].secondary_row+1),int(raw[i].main_occurrence+1)};
  return result;
}
}
