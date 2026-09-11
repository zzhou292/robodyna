#include "Config.h"
#include "output/full_shell/FixedStepHorizon.h"
#include <stdexcept>
namespace crash::cases::vehicle_run {
Horizon Plan(const Config& config) {
    if((config.duration_s!=.005 && config.duration_s!=.02 && config.duration_s!=.05) ||
        config.samples<2 || config.samples>1000 ||
        (config.resources!=ResourceProfile::Normal && config.resources!=ResourceProfile::ConditionalExpandedFull))
        throw std::invalid_argument("Run requires explicit 5, 20 or 50 ms and 2..1000 samples");
    Horizon result;
    if(!output::full_shell::PlanFixedStepHorizon(config.fixed_dt_s,config.duration_s,result.intervals) ||
        result.intervals<config.samples-1)
        throw std::invalid_argument("Run fixed horizon cannot contain its declared samples");
    result.requested_duration_s=config.duration_s;
    result.fixed_dt_s=config.fixed_dt_s;
    result.nominal_endpoint_s=static_cast<long double>(config.fixed_dt_s)*result.intervals;
    return result;
}
ResourceCaps SelectCaps(ResourceProfile profile,std::size_t host,std::size_t archive) {
    constexpr std::size_t normal_host=20ull*1000*1000*1000,normal_archive=2ull<<30;
    constexpr std::size_t full_host=60ull*1000*1000*1000,full_archive=6ull<<30;
    if(profile!=ResourceProfile::Normal && profile!=ResourceProfile::ConditionalExpandedFull)
        throw std::invalid_argument("Unknown conditional full-run resource profile");
    if(!host || !archive || host>full_host || archive>full_archive)
        throw std::invalid_argument("Complete run exceeds the maximum explicit resource allowance");
    const bool expanded=host>normal_host || archive>normal_archive;
    if(expanded && profile!=ResourceProfile::ConditionalExpandedFull)
        throw std::invalid_argument("Complete run forecast exceeds normal resource limits");
    return {host>normal_host?full_host:normal_host,archive>normal_archive?full_archive:normal_archive,expanded};
}
} // namespace crash::cases::vehicle_run
