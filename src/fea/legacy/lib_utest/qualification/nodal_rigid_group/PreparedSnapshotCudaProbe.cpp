#include "PreparedSnapshotCudaProbe.h"
#include <cstddef>
#include <cuda_runtime_api.h>
namespace {bool armed=false,completed=false;}
namespace prepared_snapshot_probe {
void FailAfterNextDeviceRead() noexcept {armed=true;completed=false;}
bool CompletedReadBeforeFailure() noexcept {return completed;}
}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* to,const void* from,std::size_t bytes,cudaMemcpyKind kind,cudaStream_t stream) {
  if(armed&&kind==cudaMemcpyDeviceToHost) {
    armed=false;
    auto result=__real_cudaMemcpyAsync(to,from,bytes,kind,stream);if(result!=cudaSuccess)return result;
    result=cudaStreamSynchronize(stream);if(result!=cudaSuccess)return result;
    completed=true;return cudaErrorInvalidValue;
  }
  return __real_cudaMemcpyAsync(to,from,bytes,kind,stream);
}
