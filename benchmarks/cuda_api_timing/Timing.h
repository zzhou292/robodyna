#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace robo_dyna::cuda_api_timing {
enum class Api : unsigned { MemcpyAsync,StreamSynchronize,Memcpy,DeviceSynchronize,Count };
inline constexpr unsigned ApiCount=static_cast<unsigned>(Api::Count),DirectionCount=6;
inline constexpr std::size_t ReportCap=64*1024,PathCap=4096;
inline constexpr std::uint64_t OpenGate=std::uint64_t(1)<<63;
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,"Timing wrappers require lock-free fixed counters");
struct Counters {
    std::atomic<std::uint64_t> calls{0},failures{0},wall_ns{0},maximum_ns{0};
    std::atomic<std::uint64_t> requested[DirectionCount]{},successful[DirectionCount]{};
};
struct State {
    // Atomic admission combines enabled bit and in-flight count, so shutdown
    // cannot silently miss a wrapper racing its disabled transition.
    std::atomic<std::uint64_t> gate{0},nested{0},clock_failures{0};
    std::atomic<bool> saturated{false};
    Counters api[ApiCount];
    char output[PathCap]{};
    std::uint64_t started_ns=0,active_at_shutdown=0;
    unsigned missing_symbols=0;
};
State& Data() noexcept;
void Resolve() noexcept;
std::uint64_t MonotonicNs() noexcept;
void Add(std::atomic<std::uint64_t>&,std::uint64_t) noexcept;
void Record(Api,std::uint64_t elapsed,bool failed,std::size_t bytes,int direction) noexcept;
void EmitReport(const State&,std::uint64_t ended_ns) noexcept;
[[noreturn]] void MissingSymbol() noexcept;
[[noreturn]] void RecursiveResolution() noexcept;
// initial-exec TLS is intentional: this diagnostic is a startup LD_PRELOAD,
// not a late dlopen injection. Entering a wrapper must never allocate TLS.
extern thread_local unsigned depth __attribute__((tls_model("initial-exec")));
} // namespace robo_dyna::cuda_api_timing
