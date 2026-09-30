#include "ForceStageCaptureCudaProbe.h"
#include <cuda_runtime_api.h>
namespace {std::size_t allocations=0;}
namespace force_stage_capture_probe {std::size_t AllocationCalls() noexcept{return allocations;}}
extern "C" cudaError_t __real_cudaMalloc(void**,std::size_t);
extern "C" cudaError_t __wrap_cudaMalloc(void** p,std::size_t bytes) {++allocations;return __real_cudaMalloc(p,bytes);}
