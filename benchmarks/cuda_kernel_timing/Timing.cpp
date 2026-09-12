#include "Timing.h"
#include "Runtime.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace robo_dyna::cuda_kernel_timing {
namespace {
State state;
pthread_once_t initialization = PTHREAD_ONCE_INIT;
void Start() noexcept {
    const char* path = std::getenv("ROBO_DYNA_CUDA_KERNEL_TIMING_OUTPUT");
    if (!path || !*path) return;
    const auto length = strnlen(path, PathCap);
    if (length == PathCap) {
        Warning("output path exceeds bound; collection disabled");
        return;
    }
    std::memcpy(state.output, path, length+1);
    state.enabled = true;
}
}
thread_local unsigned depth __attribute__((tls_model("initial-exec"))) = 0;
State& Data() noexcept { return state; }
void Initialize() noexcept { pthread_once(&initialization, Start); }
void Warning(const char* message) noexcept {
    constexpr char prefix[] = "robo-dyna CUDA kernel timing: ";
    const auto a = write(STDERR_FILENO, prefix, sizeof(prefix)-1);
    const auto b = write(STDERR_FILENO, message, std::strlen(message));
    const auto c = write(STDERR_FILENO, "\n", 1);
    (void)a;
    (void)b;
    (void)c;
}
[[noreturn]] void MissingSymbol() noexcept {
    Warning("unresolved or recursively resolved runtime symbol; cannot forward call");
    _exit(127);
}
void Add(std::uint64_t& target, std::uint64_t amount) noexcept {
    if (amount > UINT64_MAX-target) {
        target = UINT64_MAX;
        state.saturated = true;
    } else target += amount;
}
void InstrumentationError(cudaError_t error) noexcept {
    Lock lock(state.mutex);
    Add(state.instrumentation_failures);
    if (!state.first_instrumentation_error)
        state.first_instrumentation_error = static_cast<int>(error);
}
void Record(unsigned index, LaunchKind kind, dim3 grid, dim3 block, std::size_t shared,
            cudaError_t result, bool measured, float milliseconds) noexcept {
    Lock lock(state.mutex);
    auto& counters = index == Unknown ? state.unknown : state.kernels[index].counters;
    if (!counters.calls) {
        counters.grid[0] = grid.x;
        counters.grid[1] = grid.y;
        counters.grid[2] = grid.z;
        counters.block[0] = block.x;
        counters.block[1] = block.y;
        counters.block[2] = block.z;
        counters.shared = shared;
    } else if (counters.grid[0] != grid.x || counters.grid[1] != grid.y || counters.grid[2] != grid.z ||
               counters.block[0] != block.x || counters.block[1] != block.y || counters.block[2] != block.z ||
               counters.shared != shared) Add(counters.dimensions_changed);
    Add(counters.calls);
    Add(kind == LaunchKind::Public ? state.public_calls : state.handle_calls);
    if (result != cudaSuccess) Add(counters.launch_failures);
    const double ns = static_cast<double>(milliseconds)*1000000.;
    if (measured && (!std::isfinite(ns) || ns < 0 || ns >= static_cast<double>(UINT64_MAX))) {
        Add(state.invalid_times);
        measured = false;
    }
    if (measured) {
        const auto elapsed = static_cast<std::uint64_t>(ns+.5);
        if (!counters.timed_calls) counters.first_timed_ns = elapsed;
        Add(counters.timed_calls);
        Add(counters.total_ns, elapsed);
        if (elapsed > counters.maximum_ns) counters.maximum_ns = elapsed;
    } else Add(counters.unmeasured);
}
__attribute__((constructor(101))) static void Loaded() noexcept { Initialize(); }
__attribute__((destructor(101))) static void Finished() noexcept {
    Initialize();
    Lock lock(state.mutex);
    if (!state.enabled) return;
    state.closed = true;
    EmitReport(); // No CUDA calls or waiting during shutdown; active count is explicit.
}
} // namespace robo_dyna::cuda_kernel_timing
