// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cuda_runtime_api.h>
#include <cstddef>

namespace tlfea::contact::represented_interval_crossing::native_device {
// Introspection only. Querying a loaded CUDA function may initialize the CUDA
// context, so this belongs in the guarded GPU lane despite launching no kernel.
struct KernelResources {
  cudaFuncAttributes function{};
  cudaDeviceProp device{};
  int device_ordinal = -1;
  int active_blocks_per_multiprocessor = 0;
  unsigned worker_limit = 0;
  unsigned threads_per_block = 0;
  unsigned full_pool_blocks = 0;
  std::size_t device_stack_limit_bytes = 0;
  // Compiler-reported local memory times hardware resident-thread capacity.
  // This is a component bound, not total driver/context/stack memory usage.
  std::size_t resident_thread_local_bytes = 0;
};
// Never changes stack limits or launches work. Output is preserved on error.
cudaError_t QueryKernelResources(KernelResources*) noexcept;
}  // namespace tlfea::contact::represented_interval_crossing::native_device
