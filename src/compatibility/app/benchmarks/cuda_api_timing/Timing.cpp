#include "Timing.h"
#include <cstdlib>
#include <cstring>
#include <time.h>
#include <unistd.h>

namespace robo_dyna::cuda_api_timing {
namespace { State state; }
thread_local unsigned depth __attribute__((tls_model("initial-exec")))=0;
State& Data() noexcept {return state;}
std::uint64_t MonotonicNs() noexcept {
    timespec t{};
    if(clock_gettime(CLOCK_MONOTONIC,&t)!=0||t.tv_sec<0||t.tv_nsec<0||t.tv_nsec>=1000000000||
       static_cast<std::uint64_t>(t.tv_sec)>(UINT64_MAX-static_cast<std::uint64_t>(t.tv_nsec))/1000000000) {
        Add(state.clock_failures,1);return 0;
    }
    return static_cast<std::uint64_t>(t.tv_sec)*1000000000+static_cast<std::uint64_t>(t.tv_nsec);
}
void Add(std::atomic<std::uint64_t>& counter,std::uint64_t value) noexcept {
    auto old=counter.load(std::memory_order_relaxed);
    for(;;) {
        const bool overflow=value>UINT64_MAX-old;const auto next=overflow?UINT64_MAX:old+value;
        if(counter.compare_exchange_weak(old,next,std::memory_order_relaxed)) {
            if(overflow)state.saturated.store(true,std::memory_order_relaxed);return;
        }
    }
}
void Record(Api api,std::uint64_t elapsed,bool failed,std::size_t bytes,int direction) noexcept {
    auto& c=state.api[static_cast<unsigned>(api)];Add(c.calls,1);if(failed)Add(c.failures,1);Add(c.wall_ns,elapsed);
    auto maximum=c.maximum_ns.load(std::memory_order_relaxed);
    while(maximum<elapsed&&!c.maximum_ns.compare_exchange_weak(maximum,elapsed,std::memory_order_relaxed)) {}
    if(api==Api::MemcpyAsync||api==Api::Memcpy) {
        const unsigned d=direction>=0&&direction<5?static_cast<unsigned>(direction):5;
        Add(c.requested[d],bytes);if(!failed)Add(c.successful[d],bytes);
    }
}
[[noreturn]] void MissingSymbol() noexcept {
    constexpr char message[]="robo-dyna CUDA timing: unresolved runtime symbol; cannot forward CUDA call\n";
    const auto ignored=write(STDERR_FILENO,message,sizeof(message)-1);(void)ignored;
    _exit(127); // Never fabricate a CUDA success or silently skip a requested call.
}
[[noreturn]] void RecursiveResolution() noexcept {
    constexpr char message[]="robo-dyna CUDA timing: loader recursively requested unresolved CUDA symbol; cannot forward\n";
    const auto ignored=write(STDERR_FILENO,message,sizeof(message)-1);(void)ignored;
    _exit(127);
}
__attribute__((constructor(101))) static void Start() noexcept {
    // Normal resolution happens here. If a dependency constructor called CUDA
    // first, the same one-time resolver has already completed. Normal wrappers
    // need no loader call; programs that never use CUDA need no symbols.
    state.started_ns=MonotonicNs();Resolve();
    const char* path=std::getenv("ROBO_DYNA_CUDA_TIMING_OUTPUT");
    if(!path||!*path)return;
    const auto length=strnlen(path,PathCap);
    if(length==PathCap) {
        constexpr char message[]="robo-dyna CUDA timing: output path exceeds bound; collection disabled\n";
        const auto ignored=write(STDERR_FILENO,message,sizeof(message)-1);(void)ignored;return;
    }
    std::memcpy(state.output,path,length+1);state.gate.store(OpenGate,std::memory_order_release);
}
__attribute__((destructor(101))) static void Finish() noexcept {
    const auto gate=state.gate.fetch_and(~OpenGate,std::memory_order_acq_rel);
    if(!(gate&OpenGate))return;state.active_at_shutdown=gate&~OpenGate;
    EmitReport(state,MonotonicNs()); // No implicit synchronization or device operation.
}
} // namespace robo_dyna::cuda_api_timing
