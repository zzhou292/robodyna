#include "ObservationTransfer.h"
#include <cuda_runtime_api.h>
namespace {bool active=false;motion_transfer_probe::Counts counts;}
namespace motion_transfer_probe {
void Begin() noexcept {counts={};active=true;}
Counts End() noexcept {active=false;return counts;}
}
// Qualification-only linker observation. No production callback or hook.
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* to,const void* from,std::size_t bytes,cudaMemcpyKind kind,cudaStream_t stream) {
  if(active&&kind==cudaMemcpyDeviceToHost){++counts.host_reads;counts.host_bytes+=bytes;}
  return __real_cudaMemcpyAsync(to,from,bytes,kind,stream);
}
extern "C" cudaError_t __real_cudaMalloc(void**,std::size_t);
extern "C" cudaError_t __wrap_cudaMalloc(void** pointer,std::size_t bytes) {
  if(active)++counts.device_allocations;
  return __real_cudaMalloc(pointer,bytes);
}
