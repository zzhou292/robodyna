// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FacetFilterCudaProbe.h"
#include <cuda_runtime_api.h>
#include <atomic>
namespace {
std::atomic<std::size_t> allocations{0}, copies{0};
std::atomic<bool> fail_copy{false};
}
extern "C" cudaError_t __real_cudaMalloc(void**,std::size_t);
extern "C" cudaError_t __wrap_cudaMalloc(void** output,std::size_t bytes) {
  const auto status=__real_cudaMalloc(output,bytes);
  if(status==cudaSuccess)++allocations;
  return status;
}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* out,const void* in,std::size_t bytes,
    cudaMemcpyKind kind,cudaStream_t stream) {
  ++copies;
  if(kind==cudaMemcpyHostToDevice && fail_copy.exchange(false)) return cudaErrorMemoryAllocation;
  return __real_cudaMemcpyAsync(out,in,bytes,kind,stream);
}
namespace facet_filter_cuda_probe {
std::size_t Allocations() noexcept{return allocations.load();}
std::size_t Copies() noexcept{return copies.load();}
void FailNextHostToDeviceCopy() noexcept{fail_copy=true;}
}
