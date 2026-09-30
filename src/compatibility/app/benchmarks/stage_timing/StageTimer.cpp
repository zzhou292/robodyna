#include "StageTimer.h"
#include <ctime>

namespace crash::benchmarks {
bool MonotonicNanoseconds(void*,std::uint64_t* output) noexcept {
    timespec t{};
    if(clock_gettime(CLOCK_MONOTONIC,&t)!=0||t.tv_sec<0||t.tv_nsec<0||t.tv_nsec>=1000000000)return false;
    const auto seconds=static_cast<std::uint64_t>(t.tv_sec),fraction=static_cast<std::uint64_t>(t.tv_nsec);
    if(seconds>(UINT64_MAX-fraction)/1000000000)return false;
    *output=seconds*1000000000+fraction;return true;
}
} // namespace crash::benchmarks
