// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CopyFault.h"
#include <atomic>
#include <cstdint>
#include <cuda_runtime_api.h>
namespace {
std::atomic<std::size_t> remaining{SIZE_MAX};
}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*, const void*, std::size_t,
                                            cudaMemcpyKind, cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* output, const void* input,
    std::size_t bytes, cudaMemcpyKind kind, cudaStream_t stream) {
  if (kind == cudaMemcpyHostToDevice) {
    auto value = remaining.load(std::memory_order_relaxed);
    if (value != SIZE_MAX) {
      if (value == 0) {
        remaining.store(SIZE_MAX, std::memory_order_relaxed);
        return cudaErrorMemoryAllocation;
      }
      remaining.store(value - 1, std::memory_order_relaxed);
    }
  }
  return __real_cudaMemcpyAsync(output, input, bytes, kind, stream);
}
namespace native_gpu_copy_fault {
void Arm(std::size_t successful_copies) noexcept { remaining = successful_copies; }
void Reset() noexcept { remaining = SIZE_MAX; }
}  // namespace native_gpu_copy_fault
