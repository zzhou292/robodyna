#include "Runtime.h"
#include "Timing.h"
#include <cerrno>
#include <dlfcn.h>

namespace robo_dyna::cuda_kernel_timing {
namespace {
Runtime runtime;
pthread_once_t resolution = PTHREAD_ONCE_INIT;
thread_local bool resolving __attribute__((tls_model("initial-exec"))) = false;
template<class F> void FindSymbol(F& slot, const char* name) noexcept {
    slot = reinterpret_cast<F>(dlsym(RTLD_NEXT, name));
}
void Resolve() noexcept {
    resolving = true;
    FindSymbol(runtime.launch, "cudaLaunchKernel");
    FindSymbol(runtime.launch_handle, "__cudaLaunchKernel");
    FindSymbol(runtime.get_kernel, "__cudaGetKernel");
    FindSymbol(runtime.register_function, "__cudaRegisterFunction");
    FindSymbol(runtime.unregister_module, "__cudaUnregisterFatBinary");
    FindSymbol(runtime.is_capturing, "cudaStreamIsCapturing");
    FindSymbol(runtime.create, "cudaEventCreate");
    FindSymbol(runtime.record, "cudaEventRecord");
    FindSymbol(runtime.synchronize, "cudaEventSynchronize");
    FindSymbol(runtime.elapsed, "cudaEventElapsedTime");
    FindSymbol(runtime.destroy, "cudaEventDestroy");
    resolving = false;
}
bool Checked(cudaError_t error) noexcept {
    if (error == cudaSuccess) return true;
    InstrumentationError(error);
    return false;
}
struct Events {
    const Runtime& api;
    cudaEvent_t start = nullptr, stop = nullptr;
    explicit Events(const Runtime& runtime) : api(runtime) {}
    ~Events() {
        if (stop) Checked(api.destroy(stop));
        if (start) Checked(api.destroy(start));
    }
    bool Begin(cudaStream_t stream) {
        return Checked(api.create(&start)) && Checked(api.create(&stop)) &&
               Checked(api.record(start, stream));
    }
    bool End(cudaStream_t stream, float& elapsed) {
        return Checked(api.record(stop, stream)) && Checked(api.synchronize(stop)) &&
               Checked(api.elapsed(&elapsed, start, stop));
    }
};
template<class F>
cudaError_t Launch(LaunchKind kind, const void* key, dim3 grid, dim3 block,
                   std::size_t shared, cudaStream_t stream, int incoming_errno, F forward) {
    auto& state = Data();
    unsigned index = Unknown;
    bool collect = false;
    {
        Lock lock(state.mutex);
        if (state.enabled && !state.closed) {
            if (depth) Add(state.nested);
            else {
                collect = true;
                index = Find(kind, key);
                Add(state.active);
            }
        }
    }
    if (!collect) {
        errno = incoming_errno;
        return forward();
    }
    ++depth;
    cudaError_t result;
    int returned_errno;
    bool measured = false;
    float milliseconds = 0;
    {
        // This diagnostic deliberately serializes intercepted outer launches.
        // At most one temporary event pair exists, in the calling thread's
        // current CUDA context, and it is destroyed before that thread resumes.
        Lock lock(state.measurement);
        const auto& api = Real();
        Events events(api);
        bool ready = false;
        if (!api.CanMeasure()) InstrumentationError(cudaErrorNotSupported);
        else {
            cudaStreamCaptureStatus capture = cudaStreamCaptureStatusNone;
            if (Checked(api.is_capturing(stream, &capture))) {
                if (capture != cudaStreamCaptureStatusNone) {
                    Lock counters(state.mutex);
                    Add(state.capture_skipped);
                } else ready = events.Begin(stream);
            }
        }
        errno = incoming_errno;
        result = forward(); // Exactly once, even if every instrumentation step fails.
        returned_errno = errno;
        if (ready && result == cudaSuccess) measured = events.End(stream, milliseconds);
    }
    Record(index, kind, grid, block, shared, result, measured, milliseconds);
    {
        Lock lock(state.mutex);
        --state.active;
    }
    --depth;
    errno = returned_errno;
    return result;
}
}
const Runtime& Real() noexcept {
    if (resolving) MissingSymbol();
    pthread_once(&resolution, Resolve);
    return runtime;
}
} // namespace robo_dyna::cuda_kernel_timing

using namespace robo_dyna::cuda_kernel_timing;
#define KERNEL_TIMING_EXPORT extern "C" __attribute__((visibility("default")))
KERNEL_TIMING_EXPORT cudaError_t CUDARTAPI cudaLaunchKernel(
    const void* function, dim3 grid, dim3 block, void** args, std::size_t shared, cudaStream_t stream) {
    const int incoming_errno = errno;
    Initialize();
    const auto real = Real().launch;
    if (!real) MissingSymbol();
    return Launch(LaunchKind::Public, function, grid, block, shared, stream, incoming_errno,
                  [&] { return real(function, grid, block, args, shared, stream); });
}
KERNEL_TIMING_EXPORT cudaError_t CUDARTAPI __cudaLaunchKernel(
    cudaKernel_t kernel, dim3 grid, dim3 block, void** args, std::size_t shared, cudaStream_t stream) {
    const int incoming_errno = errno;
    Initialize();
    const auto real = Real().launch_handle;
    if (!real) MissingSymbol();
    return Launch(LaunchKind::Handle, kernel, grid, block, shared, stream, incoming_errno,
                  [&] { return real(kernel, grid, block, args, shared, stream); });
}
KERNEL_TIMING_EXPORT cudaError_t CUDARTAPI __cudaGetKernel(cudaKernel_t* output, const void* host) {
    const int incoming_errno = errno;
    Initialize();
    const auto real = Real().get_kernel;
    if (!real) MissingSymbol();
    errno = incoming_errno;
    const auto result = real(output, host);
    const int returned_errno = errno;
    if (result == cudaSuccess && output) Bind(*output, host);
    errno = returned_errno;
    return result;
}
KERNEL_TIMING_EXPORT void CUDARTAPI __cudaRegisterFunction(
    void** module, const char* host, char* device, const char* name, int limit,
    uint3* tid, uint3* bid, dim3* block, dim3* grid, int* warp) {
    const int incoming_errno = errno;
    Initialize();
    const auto real = Real().register_function;
    if (!real) MissingSymbol();
    errno = incoming_errno;
    real(module, host, device, name, limit, tid, bid, block, grid, warp);
    const int returned_errno = errno;
    Register(module, host, name);
    errno = returned_errno;
}
KERNEL_TIMING_EXPORT void CUDARTAPI __cudaUnregisterFatBinary(void** module) {
    const int incoming_errno = errno;
    Initialize();
    const auto real = Real().unregister_module;
    if (!real) MissingSymbol();
    errno = incoming_errno;
    real(module);
    const int returned_errno = errno;
    Unregister(module);
    errno = returned_errno;
}
#undef KERNEL_TIMING_EXPORT
