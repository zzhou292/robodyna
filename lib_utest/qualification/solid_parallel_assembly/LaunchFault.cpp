#include "LaunchFault.h"
#include <cuda_runtime_api.h>
namespace { unsigned fail_at=0,calls=0; }
namespace solid_parallel_test::launch_fault {
void Arm(unsigned boundary) noexcept { fail_at=boundary;calls=0; }
unsigned Observed() noexcept { return calls; }
}
extern "C" cudaError_t __real_cudaPeekAtLastError();
extern "C" cudaError_t __wrap_cudaPeekAtLastError() {
  if (fail_at && ++calls==fail_at) { fail_at=0;return cudaErrorLaunchFailure; }
  return __real_cudaPeekAtLastError();
}
