#include "Config.h"
#include "output/full_shell/FixedStepHorizon.h"
#include <stdexcept>
namespace crash::cases::vehicle_run {
const char* PhysicalProfileName(PhysicalProfile profile) {
    switch(profile) {
        case PhysicalProfile::RetainedShellAssembliesV1:return "retained-shell-v1";
        case PhysicalProfile::ExtendedSolidsV4:return "extended-solids-v4";
        case PhysicalProfile::VehicleSupportsV5:return "vehicle-supports-v5";
    }
    throw std::invalid_argument("Unknown Yaris physical profile");
}
Horizon Plan(const Config& config) {
    PhysicalProfileName(config.physical_profile);
    ContactProfileName(config.contact_profile);
    if(config.self_contact_cuda_native_crossing && config.contact_profile!=ContactProfile::WallSelfContactV1)
        throw std::invalid_argument("CUDA native crossing requires the explicit wall+self profile");
    if(config.self_contact_cuda_facet_filters && config.contact_profile!=ContactProfile::WallSelfContactV1)
        throw std::invalid_argument("CUDA facet filters require the explicit wall+self profile");
    if(config.self_contact_diagnostics && config.contact_profile!=ContactProfile::WallSelfContactV1)
        throw std::invalid_argument("Self-contact diagnostics require the explicit wall+self profile");
    if(config.contact_profile==ContactProfile::WallSelfContactV1 &&
        config.physical_profile!=PhysicalProfile::VehicleSupportsV5)
        throw std::invalid_argument("Wall+self contact requires vehicle-supports-v5");
    if(config.samples<2 || config.samples>1000 ||
        (config.resources!=ResourceProfile::Normal && config.resources!=ResourceProfile::ConditionalExpandedFull))
        throw std::invalid_argument("Run requires 2..1000 samples and a valid resource profile");
    Horizon result;
    if(config.exact_steps) {
        result.intervals=config.exact_steps;
        if(!output::full_shell::PlanExactStepHorizon(config.fixed_dt_s,result.intervals,result.requested_duration_s))
            throw std::invalid_argument("Exact step count has no admissible finite fixed horizon");
    } else {
        if(config.duration_s!=.0005 && config.duration_s!=.005 && config.duration_s!=.02 && config.duration_s!=.05)
            throw std::invalid_argument("Run requires explicit 0.5, 5, 20 or 50 ms, or an exact step count");
        result.requested_duration_s=config.duration_s;
        if(!output::full_shell::PlanFixedStepHorizon(config.fixed_dt_s,config.duration_s,result.intervals))
            throw std::invalid_argument("Run fixed horizon is invalid");
    }
    if(result.requested_duration_s>.05)
        throw std::invalid_argument("Run horizon exceeds the supported 50 ms wall envelope");
    if(result.intervals<config.samples-1)
        throw std::invalid_argument("Run fixed horizon cannot contain its declared samples");
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
