#include "NodalWallCapacityCudaProbe.h"
#include <cstddef>
#include <cuda_runtime_api.h>

namespace {
bool count_allocations=false;
unsigned allocation_calls=0,device_reads=0;
int failure_after=-1;
}
namespace nodal_wall_capacity_probe {
void CountAllocations(bool enabled) noexcept { count_allocations=enabled; if(enabled) allocation_calls=0; }
unsigned AllocationCalls() noexcept { return allocation_calls; }
void FailDeviceReadAfter(int reads) noexcept { failure_after=reads; device_reads=0; }
unsigned DeviceReads() noexcept { return device_reads; }
}
extern "C" cudaError_t __real_cudaMalloc(void**,std::size_t);
extern "C" cudaError_t __wrap_cudaMalloc(void** pointer,std::size_t bytes) {
  if(count_allocations) ++allocation_calls;
  return __real_cudaMalloc(pointer,bytes);
}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* to,const void* from,std::size_t bytes,cudaMemcpyKind kind,cudaStream_t stream) {
  if(failure_after>=0 && kind==cudaMemcpyDeviceToHost) {
    ++device_reads;
    if(failure_after--==0) return cudaErrorInvalidValue;
  }
  return __real_cudaMemcpyAsync(to,from,bytes,kind,stream);
}
