#pragma once
#include <cuda_runtime_api.h>
#include <cstddef>
#include <cstdint>
#include <pthread.h>

namespace robo_dyna::cuda_kernel_timing {
inline constexpr std::size_t KernelCap = 2048, HandleCap = 4096;
inline constexpr std::size_t NameCap = 1024, PathCap = 4096, ReportCap = 16u << 20;
inline constexpr unsigned Unknown = KernelCap;
enum class LaunchKind : unsigned { Public, Handle };
struct Counters {
    std::uint64_t calls = 0, launch_failures = 0, timed_calls = 0;
    std::uint64_t total_ns = 0, maximum_ns = 0, first_timed_ns = 0;
    std::uint64_t unmeasured = 0, dimensions_changed = 0;
    unsigned grid[3]{}, block[3]{};
    std::size_t shared = 0;
};
struct Kernel {
    const void* host = nullptr;
    void** module = nullptr;
    char name[NameCap]{};
    bool active = false, truncated = false;
    Counters counters;
};
struct Handle {
    cudaKernel_t value = nullptr;
    unsigned kernel = Unknown;
};
struct State {
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t measurement = PTHREAD_MUTEX_INITIALIZER;
    bool enabled = false, closed = false, saturated = false;
    char output[PathCap]{};
    Kernel kernels[KernelCap]{};
    Handle handles[HandleCap]{};
    std::size_t kernel_count = 0, handle_count = 0;
    Counters unknown;
    std::uint64_t registrations_dropped = 0, handles_dropped = 0;
    std::uint64_t active = 0, nested = 0, capture_skipped = 0;
    std::uint64_t instrumentation_failures = 0, invalid_times = 0;
    std::uint64_t public_calls = 0, handle_calls = 0;
    int first_instrumentation_error = 0;
};
class Lock {
public:
    explicit Lock(pthread_mutex_t& mutex) : mutex_(mutex) { pthread_mutex_lock(&mutex_); }
    ~Lock() { pthread_mutex_unlock(&mutex_); }
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
private:
    pthread_mutex_t& mutex_;
};
State& Data() noexcept;
void Initialize() noexcept;
void Add(std::uint64_t&, std::uint64_t value = 1) noexcept; // Caller holds state mutex.
void Register(void** module, const void* host, const char* name) noexcept;
void Unregister(void** module) noexcept;
void Bind(cudaKernel_t handle, const void* host) noexcept;
unsigned Find(LaunchKind, const void*) noexcept; // Caller holds state mutex.
void Record(unsigned, LaunchKind, dim3 grid, dim3 block, std::size_t shared,
            cudaError_t result, bool measured, float milliseconds) noexcept;
void InstrumentationError(cudaError_t) noexcept;
void EmitReport() noexcept;
void Warning(const char*) noexcept;
[[noreturn]] void MissingSymbol() noexcept;
extern thread_local unsigned depth __attribute__((tls_model("initial-exec")));
} // namespace robo_dyna::cuda_kernel_timing
