#pragma once
#include <cmath>
#include <cstdint>
#include <initializer_list>
namespace crash::output::full_shell {
// Mathematical planning only. Saved times still come from the actual owner.
inline bool MatchesFixedStepHorizon(std::uint64_t intervals,double fixed_dt,double duration) noexcept {
    if(!intervals || intervals>UINT64_MAX/2 || !std::isfinite(fixed_dt) || fixed_dt<=0 ||
        !std::isfinite(duration) || duration<=0) return false;
    const long double h=fixed_dt,target=duration;
    return std::isfinite(fixed_dt*static_cast<double>(intervals)) &&
        std::isfinite(h*intervals) && h*intervals>=target && h*(intervals-1)<target;
}
// Exact count is authoritative. Select a representable descriptive duration
// inside ((count-1)*h,count*h], then verify with the same archive contract.
// At most the rounded product and its immediate predecessor are considered;
// if the interval contains no representable duration, reject rather than round
// the requested number of physical steps or silently change h.
inline bool PlanExactStepHorizon(double fixed_dt,std::uint64_t intervals,double& duration) noexcept {
    if(!intervals || intervals>UINT64_MAX/2 || !std::isfinite(fixed_dt) || fixed_dt<=0) return false;
    const long double endpoint=static_cast<long double>(fixed_dt)*intervals;
    const double rounded=static_cast<double>(endpoint);
    if(!std::isfinite(rounded) || rounded<=0) return false;
    for(const double candidate:{rounded,std::nextafter(rounded,0.)}) {
        if(MatchesFixedStepHorizon(intervals,fixed_dt,candidate)) {
            duration=candidate;
            return true;
        }
    }
    return false;
}
inline bool PlanFixedStepHorizon(double fixed_dt,double duration,std::uint64_t& output) noexcept {
    if(!std::isfinite(fixed_dt) || fixed_dt<=0 || !std::isfinite(duration) || duration<=0) return false;
    const long double count=std::ceil(static_cast<long double>(duration)/fixed_dt);
    if(!std::isfinite(count) || count<1 || count>static_cast<long double>(UINT64_MAX/2)) return false;
    auto intervals=static_cast<std::uint64_t>(count);
    // Division can round exactly onto an integer. The original multiplication
    // predicate is authoritative, including its strict previous-endpoint test.
    if(!MatchesFixedStepHorizon(intervals,fixed_dt,duration)) {
        if(intervals>1 && MatchesFixedStepHorizon(intervals-1,fixed_dt,duration)) --intervals;
        else if(intervals<UINT64_MAX/2 && MatchesFixedStepHorizon(intervals+1,fixed_dt,duration)) ++intervals;
        else return false;
    }
    output=intervals;
    return true;
}
} // namespace crash::output::full_shell
