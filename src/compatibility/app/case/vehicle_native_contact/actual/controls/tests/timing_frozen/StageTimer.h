#pragma once
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace crash::benchmarks {
// Per-instance injection is only a host qualification seam. The case uses the
// default monotonic clock. No timer operation allocates or calls CUDA.
struct StageClock {
    using Read=bool (*)(void*,std::uint64_t*) noexcept;
    Read read=nullptr;
    void* context=nullptr;
};
bool MonotonicNanoseconds(void*,std::uint64_t*) noexcept;
struct StageCounter {
    std::uint64_t calls=0,failures=0,valid_samples=0,wall_ns=0,maximum_ns=0;
};
template<std::size_t N> struct StageTimingSnapshot {
    bool enabled=false,counter_saturated=false;
    std::uint64_t clock_failures=0,backward_samples=0;
    std::array<StageCounter,N> total{},last_step{};
};
// Single serialized coordinator only; this is not a concurrent timer. Slot zero
// is the inclusive Step call. Remaining call sites must be non-overlapping.
template<std::size_t N> class StageTimer {
  public:
    explicit StageTimer(bool enabled=false,StageClock clock={}) noexcept
        :clock_{clock.read?clock.read:MonotonicNanoseconds,clock.context} { snapshot_.enabled=enabled; }
    StageTimingSnapshot<N> snapshot() const noexcept { return snapshot_; }
    template<class F> auto Step(F&& call) -> decltype(call()) {
        if(!snapshot_.enabled)return call();
        snapshot_.last_step={};
        return Measure<0>(std::forward<F>(call));
    }
    template<std::size_t I,class F> auto Measure(F&& call) -> decltype(call()) {
        static_assert(I<N,"Stage must be part of the declared fixed inventory");
        if(!snapshot_.enabled)return call();
        std::uint64_t start=0;const bool valid=Read(start);
        Sample sample{*this,I,start,valid};
        decltype(auto) result=call();sample.failed=!bool(result);return result;
    }
  private:
    struct Sample {
        StageTimer& timer;std::size_t index;std::uint64_t start;bool valid,failed=true;
        ~Sample() noexcept {timer.Finish(index,start,valid,failed);}
    };
    void Add(std::uint64_t& value,std::uint64_t increment) noexcept {
        if(increment>UINT64_MAX-value) {value=UINT64_MAX;snapshot_.counter_saturated=true;}
        else value+=increment;
    }
    bool Read(std::uint64_t& value) noexcept {
        const int saved=errno;const bool valid=clock_.read(clock_.context,&value);errno=saved;
        if(!valid)Add(snapshot_.clock_failures,1);
        return valid;
    }
    void Finish(std::size_t i,std::uint64_t start,bool valid,bool failed) noexcept {
        std::uint64_t end=0;const bool end_valid=Read(end);
        valid=valid&&end_valid;
        if(valid&&end<start) {Add(snapshot_.backward_samples,1);valid=false;}
        for(auto* counter:{&snapshot_.total[i],&snapshot_.last_step[i]}) {
            Add(counter->calls,1);if(failed)Add(counter->failures,1);
            if(valid) {
                Add(counter->valid_samples,1);Add(counter->wall_ns,end-start);
                if(end-start>counter->maximum_ns)counter->maximum_ns=end-start;
            }
        }
    }
    const StageClock clock_;
    StageTimingSnapshot<N> snapshot_;
};
} // namespace crash::benchmarks
