#pragma once
#include <cuda_runtime_api.h>
#include <cstddef>

// These CUDA 13 internal ABI declarations are pinned to local
// crt/device_functions.h and crt/host_runtime.h. They are not inferred from
// public cudaLaunchKernel's different first-argument interpretation.
extern "C" cudaError_t CUDARTAPI __cudaGetKernel(cudaKernel_t*, const void*);
extern "C" cudaError_t CUDARTAPI __cudaLaunchKernel(
    cudaKernel_t, dim3, dim3, void**, std::size_t, cudaStream_t);
extern "C" void CUDARTAPI __cudaRegisterFunction(
    void**, const char*, char*, const char*, int, uint3*, uint3*, dim3*, dim3*, int*);
extern "C" void CUDARTAPI __cudaUnregisterFatBinary(void**);

namespace robo_dyna::cuda_kernel_timing {
struct Runtime {
    decltype(&cudaLaunchKernel) launch = nullptr;
    decltype(&__cudaLaunchKernel) launch_handle = nullptr;
    decltype(&__cudaGetKernel) get_kernel = nullptr;
    decltype(&__cudaRegisterFunction) register_function = nullptr;
    decltype(&__cudaUnregisterFatBinary) unregister_module = nullptr;
    decltype(&cudaStreamIsCapturing) is_capturing = nullptr;
    decltype(&cudaEventCreate) create = nullptr;
    decltype(&cudaEventRecord) record = nullptr;
    decltype(&cudaEventSynchronize) synchronize = nullptr;
    decltype(&cudaEventElapsedTime) elapsed = nullptr;
    decltype(&cudaEventDestroy) destroy = nullptr;
    bool CanMeasure() const noexcept {
        return is_capturing && create && record && synchronize && elapsed && destroy;
    }
};
const Runtime& Real() noexcept;
} // namespace robo_dyna::cuda_kernel_timing
