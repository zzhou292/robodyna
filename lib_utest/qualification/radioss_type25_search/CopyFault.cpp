#include "CopyFault.h"
#include <cuda_runtime_api.h>
namespace {bool armed=false;}
namespace type25_search_test::copy_fault {void Arm() noexcept {armed=true;}}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* dst,const void* src,size_t bytes,cudaMemcpyKind kind,cudaStream_t stream) {
  if(armed){armed=false;return cudaErrorUnknown;}
  return __real_cudaMemcpyAsync(dst,src,bytes,kind,stream);
}
