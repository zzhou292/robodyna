#pragma once
#include "SourceNodalWallCuda.h"

namespace crash::qualification::source_contact::nodal {
struct ProfileClocks {
    unsigned long long kernel_begin=0,validation_begin=0,validation_end=0;
    unsigned long long node_scan_begin=0,node_scan_end=0,reduction_begin=0,reduction_end=0;
    unsigned long long query_begin[NodeCount]{},query_end[NodeCount]{};
    unsigned long long law_begin[NodeCount]{},law_end[NodeCount]{};
    unsigned long long parent_begin[ParentCount]{},parent_end[ParentCount]{};
    unsigned long long publish_begin[Workers]{},publish_end[Workers]{};
};
struct ProfileStorage { Storage work; ProfileClocks clocks; };
static_assert(sizeof(ProfileStorage)+sizeof(Storage)<=2*1024*1024,
              "Both instrumented and unchanged nodal allocations must fit the fixed cap");
struct KernelFacts {
    cudaFuncAttributes original{},instrumented{};
    int original_max_blocks_per_sm=0,instrumented_max_blocks_per_sm=0;
    int multiprocessors=0,max_threads_per_sm=0;
    std::size_t stack_limit_bytes=0;
};
// Read-only resource/estimated occupancy queries, not achieved occupancy.
// Attribute queries can cause CUDA module loading before the first timed call.
cudaError_t InspectKernels(KernelFacts&);

// Same arithmetic, 128 threads and barriers as Device, with compile-time
// optional clock64 reads/writes only. Per-thread spans include SM scheduling
// and can overlap; raw cycles are not exclusive hardware instruction costs or
// milliseconds. Separate CUDA events measure total instrumentation overhead.
class Profiler {
  public:
    Profiler();
    ~Profiler();
    Profiler(const Profiler&)=delete;
    Profiler& operator=(const Profiler&)=delete;
    cudaError_t initialization_status() const { return status_; }
    cudaError_t Evaluate(const Input&,Result&,Report&,Timing&,ProfileClocks&);
  private:
    cudaError_t Fail(cudaError_t error);
    ProfileStorage* storage_=nullptr;
    cudaEvent_t begin_=nullptr,end_=nullptr;
    cudaError_t status_=cudaSuccess;
};
} // namespace crash::qualification::source_contact::nodal
