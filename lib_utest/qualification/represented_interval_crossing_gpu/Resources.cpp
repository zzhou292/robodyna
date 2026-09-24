// SPDX-License-Identifier: MIT
#include "lib_src/collision/represented_interval_crossing/native_device/Resources.h"
#include <iomanip>
#include <iostream>

int main() {
  namespace device = tlfea::contact::represented_interval_crossing::native_device;
  device::KernelResources value;
  const auto error = device::QueryKernelResources(&value);
  if (error != cudaSuccess) {
    std::cerr << "Native GPU resource query: " << cudaGetErrorString(error) << '\n';
    return 1;
  }
  std::cout << "{\"schema\":\"native_gpu_kernel_resources.v1\","
            << "\"device\":" << value.device_ordinal
            << ",\"name\":" << std::quoted(value.device.name)
            << ",\"multiprocessors\":" << value.device.multiProcessorCount
            << ",\"max_threads_per_multiprocessor\":" << value.device.maxThreadsPerMultiProcessor
            << ",\"active_blocks_per_multiprocessor\":" << value.active_blocks_per_multiprocessor
            << ",\"worker_limit\":" << value.worker_limit
            << ",\"threads_per_block\":" << value.threads_per_block
            << ",\"full_pool_blocks\":" << value.full_pool_blocks
            << ",\"registers_per_thread\":" << value.function.numRegs
            << ",\"function_max_threads_per_block\":" << value.function.maxThreadsPerBlock
            << ",\"shared_bytes_per_block\":" << value.function.sharedSizeBytes
            << ",\"local_bytes_per_thread\":" << value.function.localSizeBytes
            << ",\"resident_thread_local_bytes\":" << value.resident_thread_local_bytes
            << ",\"device_stack_limit_bytes\":" << value.device_stack_limit_bytes
            << ",\"ptx_version\":" << value.function.ptxVersion
            << ",\"binary_version\":" << value.function.binaryVersion
            << ",\"kernel_launched\":false,\"stack_limit_setter_called\":false}\n";
  return std::cout ? 0 : 1;
}
