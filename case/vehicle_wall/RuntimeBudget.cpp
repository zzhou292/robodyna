#include "RuntimeBudget.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::cases::vehicle_wall::detail {
RuntimeForecast ComposeForecast(const vehicle_dynamics::Forecast& dynamics, const SetupForecast& setup,
    const tl::fea::ShellMappedFootprint& contact,
    const tl::fea::ShellPhysicalScratchParticipationForecast& participation,
    std::size_t fixed_bytes, RuntimeLimits limits) {
    output::Require(limits.host_bytes && limits.host_bytes<=std::size_t{20}*1000*1000*1000 &&
        limits.device_bytes && limits.device_bytes<=std::size_t{8}<<30,"Invalid complete wall runtime cap");
    output::Require(participation.publication_host_bytes &&
        participation.total_host_bytes>=participation.publication_host_bytes,
        "Invalid fixed wall scratch participation forecast");
    output::Require(contact.source_host_bytes<=dynamics.startup.retained_source_upper_bound &&
        setup.shared_source_upper_bound==dynamics.startup.retained_source_upper_bound,
        "Exact shared wall source is not contained in the owner source reservation");
    tl::util::BoundedArenaLayout host(limits.host_bytes), device(limits.device_bytes);
    tl::util::ArenaRegion unused;
    for (auto bytes : {dynamics.startup.retained_host_upper_bound,dynamics.workspace_bytes,
                       setup.retained_setup_bytes,contact.participant_host_bytes,
                       participation.publication_host_bytes,fixed_bytes}) {
        output::Require(host.Append<std::byte>(bytes,unused),"Complete retained wall runtime exceeds host cap");
    }
    RuntimeForecast result;
    result.contact=contact;
    result.participation=participation;
    result.retained_host_upper_bound=host.bytes();
    const auto scratch=std::max({dynamics.startup.peak_temporary_bytes,
                                setup.geometry.temporary_bytes,contact.startup_scratch_bytes});
    output::Require(host.Append<std::byte>(scratch,unused),"Complete wall runtime peak exceeds host cap");
    result.peak_host_upper_bound=host.bytes();
    for (auto bytes : {dynamics.startup.device_bytes,contact.device_bytes}) {
        output::Require(device.Append<std::byte>(bytes,unused),"Complete wall runtime exceeds device cap");
    }
    result.device_bytes=device.bytes();
    return result;
}
} // namespace crash::cases::vehicle_wall::detail
