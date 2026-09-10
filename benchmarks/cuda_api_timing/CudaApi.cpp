#include "Timing.h"
#include <cuda_runtime_api.h>
#include <cerrno>
#include <dlfcn.h>
#include <pthread.h>

namespace robo_dyna::cuda_api_timing {
namespace {
std::atomic<decltype(&cudaMemcpyAsync)> memcpy_async{nullptr};
std::atomic<decltype(&cudaStreamSynchronize)> stream_synchronize{nullptr};
std::atomic<decltype(&cudaMemcpy)> memcpy_sync{nullptr};
std::atomic<decltype(&cudaDeviceSynchronize)> device_synchronize{nullptr};
pthread_once_t resolver_once=PTHREAD_ONCE_INIT;
thread_local bool resolving __attribute__((tls_model("initial-exec")))=false;
template<class F> void Find(std::atomic<F>& destination, const char* name) noexcept {
    const auto function = reinterpret_cast<F>(dlsym(RTLD_NEXT, name));
    if (!function) ++Data().missing_symbols;
    destination.store(function, std::memory_order_release);
}
template<class F, class... Args>
cudaError_t Call(const std::atomic<F>& slot, Api api, std::size_t bytes, int direction, Args... args) {
    auto real = slot.load(std::memory_order_acquire);
    if (!real) {
        // A dependency constructor can reach us before Start. Only this first
        // resolution may enter the loader/allocate, never a timed interval.
        const int incoming_errno = errno;
        Resolve();
        real = slot.load(std::memory_order_acquire);
        errno = incoming_errno;
    }
    if (!real) MissingSymbol();
    auto gate = Data().gate.load(std::memory_order_acquire);
    if (!(gate & OpenGate)) return real(args...);
    if (depth) {
        Add(Data().nested, 1);
        return real(args...);
    }
    for (;;) {
        if (!(gate & OpenGate)) return real(args...);
        if ((gate & ~OpenGate) == OpenGate-1) {
            Data().saturated.store(true, std::memory_order_relaxed);
            return real(args...);
        }
        if (Data().gate.compare_exchange_weak(gate, gate+1, std::memory_order_acquire,
                                             std::memory_order_relaxed)) break;
    }
    ++depth;
    const int incoming_errno = errno;
    const auto start = MonotonicNs();
    errno = incoming_errno;
    const auto result = real(args...);
    const int returned_errno = errno;
    const auto end = MonotonicNs();
    std::uint64_t elapsed = 0;
    if (start && end >= start) elapsed = end-start;
    else if (start && end) Add(Data().clock_failures, 1);
    Record(api, elapsed, result != cudaSuccess, bytes, direction);
    // Publish completed records to shutdown's acquire RMW. With no calls left
    // active, the final report must observe every admitted call's counters.
    Data().gate.fetch_sub(1, std::memory_order_release);
    --depth;
    errno = returned_errno;
    return result;
}
void ResolveOnce() noexcept {
    resolving = true;
    Find(memcpy_async, "cudaMemcpyAsync");
    Find(stream_synchronize, "cudaStreamSynchronize");
    Find(memcpy_sync, "cudaMemcpy");
    Find(device_synchronize, "cudaDeviceSynchronize");
    resolving = false;
}
}
void Resolve() noexcept {
    // A loader callback into an unresolved symbol cannot be forwarded. Detect
    // that recursion instead of recursively entering pthread_once and hanging.
    if (resolving) RecursiveResolution();
    pthread_once(&resolver_once, ResolveOnce);
}
} // namespace robo_dyna::cuda_api_timing

// Public runtime ABI only. The real result and every argument are forwarded
// unchanged; these wrappers do not inspect pointers or query the CUDA device.
using namespace robo_dyna::cuda_api_timing;
extern "C" __attribute__((visibility("default"))) cudaError_t CUDARTAPI
cudaMemcpyAsync(void* dst,const void* src,std::size_t count,cudaMemcpyKind kind,cudaStream_t stream) {
    return Call(memcpy_async,Api::MemcpyAsync,count,static_cast<int>(kind),dst,src,count,kind,stream);
}
extern "C" __attribute__((visibility("default"))) cudaError_t CUDARTAPI
cudaStreamSynchronize(cudaStream_t stream) {return Call(stream_synchronize,Api::StreamSynchronize,0,0,stream);}
extern "C" __attribute__((visibility("default"))) cudaError_t CUDARTAPI
cudaMemcpy(void* dst,const void* src,std::size_t count,cudaMemcpyKind kind) {
    return Call(memcpy_sync,Api::Memcpy,count,static_cast<int>(kind),dst,src,count,kind);
}
extern "C" __attribute__((visibility("default"))) cudaError_t CUDARTAPI
cudaDeviceSynchronize() {return Call(device_synchronize,Api::DeviceSynchronize,0,0);}
